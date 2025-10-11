#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ================== WiFi Credentials ==================
const char* ssid        = "vodafoneA54964";
const char* password    = "xGJ9NykgCKxqTZGC";

// ================== MQTT Configuration ==================
const char* mqtt_server   = "116.203.53.123";  // e.g. your broker
const int   mqtt_port     = 1883;
const char* mqtt_user     = "m.ijaz1";
const char* mqtt_password = "NodeRedUdemyCourse#123";

// MQTT Topic for knee flexion data
const char* topic = "knee_flexion_data";

// ESP32 WiFi/MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// ================ BNO055 Sensors =================
// Thigh sensor at address 0x28 (ADR pin unconnected)
// Shin sensor at address 0x29 (ADR pin tied to 3.3V)
Adafruit_BNO055 bnoThigh = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bnoShin  = Adafruit_BNO055(56, 0x29);

// We'll use the ROLL component (the 'y' from Euler angles).
// baselineThighRoll is set during calibration (when the leg is extended).
float baselineThighRoll = 0.0;
bool calibrated = false;

// --------------------------------------------------------------------------
// WiFi & MQTT Routines
// --------------------------------------------------------------------------
void setupWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected!");
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32KneeFlexionClient", mqtt_user, mqtt_password)) {
      Serial.println(" connected");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

// --------------------------------------------------------------------------
// Setup
// --------------------------------------------------------------------------
void setup() {
  // IMPORTANT:
  // For battery-powered operation, ensure that your battery switch fully cuts power.
  // Each time power is applied, setup() will run and the sensors will be recalibrated.
  
  Serial.begin(115200);
  Serial.println("Starting Knee Flexion Measurement Setup...");

  // Initialize I2C on ESP32 default pins: SDA=21, SCL=22
  Wire.begin(21, 22);

  // Initialize the thigh sensor.
  if (!bnoThigh.begin()) {
    Serial.println("Error: Thigh sensor (0x28) not detected! Check wiring.");
    while (1) { delay(10); }
  } else {
    Serial.println("Thigh sensor detected.");
  }

  // Initialize the shin sensor.
  if (!bnoShin.begin()) {
    Serial.println("Error: Shin sensor (0x29) not detected! Check wiring.");
    while (1) { delay(10); }
  } else {
    Serial.println("Shin sensor detected.");
  }

  // Allow sensors to stabilize.
  delay(1000);

  Serial.println("Calibration: Hold your leg straight for 5 seconds...");
  delay(5000);  // Wait 5 seconds while the leg is extended

  // Capture the baseline roll from the thigh sensor (the 'y' in Euler angles).
  imu::Vector<3> eulerThigh = bnoThigh.getVector(Adafruit_BNO055::VECTOR_EULER);
  baselineThighRoll = eulerThigh.y(); // Using the roll component
  calibrated = true;
  
  Serial.print("Baseline Thigh Roll: ");
  Serial.println(baselineThighRoll);
  Serial.println("Calibration complete. Now measuring knee flexion angle...");

  // Setup WiFi and MQTT
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);
}

// --------------------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------------------
void loop() {
  if (!calibrated) return;

  // Ensure MQTT is connected
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Get the current Euler angles from the shin sensor (roll = 'y')
  imu::Vector<3> eulerShin = bnoShin.getVector(Adafruit_BNO055::VECTOR_EULER);
  float currentShinRoll = eulerShin.y();

  // Calculate knee flexion angle as the absolute difference between
  // the shin sensor's roll and the baseline thigh roll.
  //   - 0° when extended (roll values nearly equal)
  //   - ~90° when the segments are perpendicular.
  float kneeAngle = fabs(currentShinRoll - baselineThighRoll);

  // Constrain the knee angle to 0°..140°
  kneeAngle = constrain(kneeAngle, 0, 140);

  // Round to the nearest whole degree.
  int displayAngle = round(kneeAngle);

  // Create JSON payload.
  String payload = "{ \"knee_flexion_angle\": " + String(displayAngle) + " }";

  // Publish to MQTT.
  client.publish(topic, payload.c_str());
  Serial.print("Published Knee Flexion Angle: ");
  Serial.print(displayAngle);
  Serial.println("°");

  // Short delay for fast updates.
  delay(10);
}
