#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LittleFS.h> // Added for internal flash memory

// ================== Credentials & Config ==================
const char* ssid          = "MyHotspot";
const char* password      = "MyHotspot316";
const char* mqtt_server   = "116.203.53.123";
const int   mqtt_port     = 1883;
const char* mqtt_user     = "m.ijaz1";
const char* mqtt_password = "NodeRedUdemyCourse#123";
const char* topic         = "knee_flexion_data";

const int ONBOARD_LED      = 2;   // Built-in LED for Safety Alert
const int DANGER_THRESHOLD  = 120;  // Medical Flexion > 120° triggers alert

// ================== Objects & RTOS Handles ==================
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BNO055 bnoThigh = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bnoShin  = Adafruit_BNO055(56, 0x29);

QueueHandle_t dataQueue;     
QueueHandle_t alertQueue;    
float angleOffset = 0.0;

// ================== Task 1: Medical Sensor Logic (Core 1) ==================
void TaskSensors(void *pvParameters) {
  for (;;) {
    imu::Vector<3> t = bnoThigh.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> s = bnoShin.getVector(Adafruit_BNO055::VECTOR_EULER);

    float currentDiff = t.y() - s.y();
    float medicalFlexion = fabs(currentDiff - angleOffset);
    if (medicalFlexion > 180) medicalFlexion = 360 - medicalFlexion;
    medicalFlexion = constrain(medicalFlexion, 0, 140);
    int finalAngle = round(medicalFlexion);

    xQueueSend(dataQueue, &finalAngle, 0);
    xQueueSend(alertQueue, &finalAngle, 0);

    vTaskDelay(pdMS_TO_TICKS(20)); // High-speed 50Hz sampling
  }
}

// ================== Task 2: Safety Monitor (Core 1) ==================
void TaskSafetyAlert(void *pvParameters) {
  pinMode(ONBOARD_LED, OUTPUT);
  int currentAngle = 0;

  for (;;) {
    if (xQueueReceive(alertQueue, &currentAngle, portMAX_DELAY)) {
      if (currentAngle >= DANGER_THRESHOLD) {
        digitalWrite(ONBOARD_LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(ONBOARD_LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(100));
      } else {
        digitalWrite(ONBOARD_LED, LOW);
      }
    }
  }
}

// ================== Task 3: MQTT / LittleFS Gateway (Core 0) ==================
void TaskCommunication(void *pvParameters) {
  unsigned long lastReconnectAttempt = 0;

  for (;;) {
    bool isConnected = (WiFi.status() == WL_CONNECTED && client.connected());

    // 1. Non-Blocking Reconnect Logic
    if (!isConnected) {
        if (millis() - lastReconnectAttempt > 5000) {
            lastReconnectAttempt = millis();
            if (WiFi.status() != WL_CONNECTED) {
                WiFi.begin(ssid, password);
            } else {
                client.connect("ESP32KneeMedicalClient", mqtt_user, mqtt_password);
            }
        }
    } else {
        // 2. We ARE connected. Recover offline data from LittleFS first!
        File file = LittleFS.open("/offline.txt", FILE_READ);
        if (file && file.size() > 0) {
            Serial.println("Uploading offline backlog...");
            while (file.available()) {
                String line = file.readStringUntil('\n');
                line.trim();
                if (line.length() > 0) {
                    // Reconstruct the JSON only at transmission to save flash space
                    String payload = "{ \"knee_flexion_angle\": " + line + " }";
                    client.publish(topic, payload.c_str());
                    client.loop();
                    vTaskDelay(pdMS_TO_TICKS(5)); // Prevents flooding the MQTT broker
                }
            }
            file.close();
            LittleFS.remove("/offline.txt"); // Clear the file after successful upload
            Serial.println("Backlog cleared.");
        } else if (file) {
            file.close();
        }
    }

    // 3. Empty the live queue (use 'while' to completely drain it every tick)
    int angleToSend;
    while (xQueueReceive(dataQueue, &angleToSend, 0) == pdPASS) {
        if (isConnected) {
            String payload = "{ \"knee_flexion_angle\": " + String(angleToSend) + " }";
            client.publish(topic, payload.c_str());
        } else {
            // Offline: Append raw integer to internal flash
            File file = LittleFS.open("/offline.txt", FILE_APPEND);
            if (file) {
                file.println(angleToSend);
                file.close();
            }
        }
    }

    if (isConnected) client.loop();
    vTaskDelay(pdMS_TO_TICKS(10)); // Yield to system watchdog
  }
}

// ================== Setup (Orchestrator) ==================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  if (!bnoThigh.begin() || !bnoShin.begin()) {
    Serial.println("Critical Error: IMU not found!");
    while (1);
  }

  // Initialize Internal Flash
  if (!LittleFS.begin(true)) {
    Serial.println("Critical Error: LittleFS Mount Failed!");
    while (1);
  }

  // MEDICAL ZEROING:
  Serial.println("MEDICAL CALIBRATION: Stand perfectly straight...");
  delay(4000); 
  
  imu::Vector<3> t_start = bnoThigh.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> s_start = bnoShin.getVector(Adafruit_BNO055::VECTOR_EULER);
  angleOffset = t_start.y() - s_start.y();

  Serial.println("Calibration complete. Medical 0° established.");

  // Initialize Queues
  dataQueue = xQueueCreate(10, sizeof(int));
  alertQueue = xQueueCreate(5, sizeof(int));

  // Launch Tasks on specific cores
  xTaskCreatePinnedToCore(TaskSensors, "Sensors", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskSafetyAlert, "Safety", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskCommunication, "Comm", 8192, NULL, 1, NULL, 0); // Increased stack size for File IO
}

void loop() {
  vTaskDelete(NULL); 
}
