#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ================== Credentials & Config ==================
const char* ssid          = "MyHotspot";
const char* password      = "MyHotspot316";
const char* mqtt_server   = "116.203.53.123";
const int   mqtt_port     = 1883;
const char* mqtt_user     = "m.ijaz1";
const char* mqtt_password = "NodeRedUdemyCourse#123";
const char* topic         = "knee_flexion_data";

const int ONBOARD_LED      = 2;   // Built-in LED for Safety Alert
const int DANGER_THRESHOLD  = 120;  // Medical Flexion > 120° triggers alert， 
//prevents user from doing tasks that bends the knee too much, such as tying shoe lace or squatting.

// ================== Objects & RTOS Handles ==================
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BNO055 bnoThigh = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bnoShin  = Adafruit_BNO055(56, 0x29);

QueueHandle_t dataQueue;     // To Communication Task
QueueHandle_t alertQueue;    // To Safety Task
float angleOffset = 0.0;     // Stores the "Zero" point

// ================== Task 1: Medical Sensor Logic (Core 1) ==================
void TaskSensors(void *pvParameters) {
  for (;;) {
    // Read Euler angles (Roll/Y-axis)
    imu::Vector<3> t = bnoThigh.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> s = bnoShin.getVector(Adafruit_BNO055::VECTOR_EULER);

    // MEDICAL CALCULATION:
    // 1. Calculate the raw relative difference
    float currentDiff = t.y() - s.y();
    
    // 2. Subtract the standing offset to force 0° at extension
    float medicalFlexion = fabs(currentDiff - angleOffset);

    // 3. Handle 360-degree orientation wrap-around
    if (medicalFlexion > 180) medicalFlexion = 360 - medicalFlexion;

    // 4. Constrain to human physiological limits (0 to 140 degrees)
    medicalFlexion = constrain(medicalFlexion, 0, 140);
    int finalAngle = round(medicalFlexion);

    // Send copies to both queues
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
    // Wait for a new reading from the sensors
    if (xQueueReceive(alertQueue, &currentAngle, portMAX_DELAY)) {
      if (currentAngle >= DANGER_THRESHOLD) {
        // Medical danger: Rapid Blink
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

// ================== Task 3: MQTT / IoT Gateway (Core 0) ==================
void TaskCommunication(void *pvParameters) {
  for (;;) {
    // Handle WiFi/MQTT connectivity in the background
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(ssid, password);
        vTaskDelay(pdMS_TO_TICKS(2000));
    } else if (!client.connected()) {
        client.connect("ESP32KneeMedicalClient", mqtt_user, mqtt_password);
    }

    int angleToSend;
    // Check for data to publish
    if (xQueueReceive(dataQueue, &angleToSend, 0) == pdPASS) {
        if (client.connected()) {
            String payload = "{ \"knee_flexion_angle\": " + String(angleToSend) + " }";
            client.publish(topic, payload.c_str());
        }
    }
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(10)); // Yield to system
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

  // MEDICAL ZEROING:
  // Ask user to stand with a perfectly straight leg (0° Extension)
  Serial.println("MEDICAL CALIBRATION: Stand perfectly straight...");
  delay(4000); 
  
  // Capture the difference between sensors in a neutral state
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
  xTaskCreatePinnedToCore(TaskCommunication, "Comm", 4096, NULL, 1, NULL, 0);
}

void loop() {
  // Standard loop is not used in RTOS structure
  vTaskDelete(NULL); 
}
