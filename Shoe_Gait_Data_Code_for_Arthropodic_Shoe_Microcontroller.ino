#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ================== WiFi Credentials ==================
const char* ssid        = "MyHotspot";
const char* password    = "MyHotspot316";

// ================== MQTT Configuration ==================
const char* mqtt_server   = "116.203.53.123"; // your broker
const int   mqtt_port     = 1883;
const char* mqtt_user     = "m.ijaz1";
const char* mqtt_password = "NodeRedUdemyCourse#123";

// MQTT Topic for shoe-based gait data
const char* topic = "shoe_gait_data";

// ESP32 WiFi/MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// ================ BNO055 (Shoe IMU) =================
// The IMU is mounted horizontally and upside down on the shoe.
// Adjust axis inversions in getCorrectedAcceleration() as needed.
Adafruit_BNO055 bnoShoe = Adafruit_BNO055(60, 0x28); // or 0x29 if ADR is tied high

// -------------------- Gait Parameters --------------------
bool  stancePhase       = false; 
bool  prevStance        = false;
float lastContactTime   = 0.0f;   // Time of last foot contact (s)
float lastLiftTime      = 0.0f;   // Time of last foot lift (s)
float lastStrideTime    = 0.0f;   // Time of previous foot contact (s)

// Stride metrics for the current stride:
float gaitCycleTime     = 0.0f;   // Time between contacts (s)
float singleSupportTime = 0.0f;   // Time from foot lift to contact (s)
float cycleDistance     = 0.0f;   // Horizontal displacement in current stride (cm)
float walkingSpeed      = 0.0f;   // Walking speed (m/s)

// For naive integration of horizontal acceleration:
float velocityX = 0.0f; // cm/s
float positionX = 0.0f; // cm

// Idle timeout threshold (seconds) – if no foot contact occurs in this time, assume stationary.
const float IDLE_TIMEOUT = 2.5f;

// Threshold to determine if movement is significant (if walking speed is below this, assume stationary)
const float SPEED_THRESHOLD = 0.05f; // m/s

// Time step tracking
unsigned long lastUpdate = 0;

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
    if (client.connect("ShoeIMUClient", mqtt_user, mqtt_password)) {
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
  // For battery-powered operation, ensure that your battery switch fully cuts power,
  // so that each time you turn the battery pack on, setup() runs fresh and recalibrates.

  Serial.begin(115200);
  Serial.println("Starting Shoe IMU Gait Measurement Setup...");

  // Initialize I2C on ESP32 default pins: SDA=21, SCL=22
  Wire.begin(21, 22);

  // Initialize the shoe IMU
  if (!bnoShoe.begin()) {
    Serial.println("Error: Shoe IMU not detected! Check wiring.");
    while (1) { delay(10); }
  } else {
    Serial.println("Shoe IMU detected.");
  }

  // Optionally enable external crystal
  bnoShoe.setExtCrystalUse(true);

  // 5-second stabilization window
  Serial.println("Stabilizing for 5 seconds. Please stand still...");
  delay(5000);

  // Setup WiFi & MQTT
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);

  lastUpdate = millis();
  // Initialize stride parameters to zero
  lastStrideTime = millis() / 1000.0f;
  velocityX = 0.0f;
  positionX = 0.0f;
  cycleDistance = 0.0f;
  gaitCycleTime = 0.0f;
  singleSupportTime = 0.0f;
  walkingSpeed = 0.0f;
  
  Serial.println("Setup complete. Measuring gait parameters...");
}

// --------------------------------------------------------------------------
// Helper: Convert acceleration from m/s^2 to cm/s^2, apply orientation corrections
// --------------------------------------------------------------------------
void getCorrectedAcceleration(float& ax, float& ay, float& az) {
  // Read linear acceleration (excluding gravity) from BNO055
  imu::Vector<3> linacc = bnoShoe.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Because the sensor is mounted horizontally and upside down,
  // invert axes as needed:
  ax =  linacc.x();    // Adjust sign if needed
  ay = -linacc.y();    // Invert y
  az = -linacc.z();    // Invert z

  // Convert from m/s^2 to cm/s^2
  ax *= 100.0f;
  ay *= 100.0f;
  az *= 100.0f;
}

// --------------------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------------------
void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0f; // dt in seconds
  lastUpdate = now;
  
  float currentTime = now / 1000.0f; // current time in seconds

  // 1) Read corrected acceleration
  float ax, ay, az;
  getCorrectedAcceleration(ax, ay, az);

  // 2) Simple stance detection using vertical acceleration threshold
  float stanceThreshold = 200.0f; // ~2 m/s^2 in cm/s^2
  if (az > stanceThreshold) {
    stancePhase = true;  // Foot is contacting the ground
  } else {
    stancePhase = false; // Foot is in swing
  }

  // 3) Detect foot contact (transition from swing to stance) – end of a stride
  if (stancePhase && !prevStance) {
    float contactTime = currentTime;
    
    // Compute stride metrics for the current stride
    float finalGaitCycleTime = (contactTime - lastStrideTime);
    float finalSingleSupportTime = (contactTime - lastLiftTime);
    float finalCycleDistance = fabs(positionX); // in cm
    float finalWalkingSpeed = 0.0f;
    if (finalGaitCycleTime > 0.01f) {
      finalWalkingSpeed = (finalCycleDistance / 100.0f) / finalGaitCycleTime; // m/s
    }
    
    // If walking speed is below threshold, assume stationary; force all values to zero.
    if (finalWalkingSpeed < SPEED_THRESHOLD) {
      finalGaitCycleTime = 0.0f;
      finalSingleSupportTime = 0.0f;
      finalCycleDistance = 0.0f;
      finalWalkingSpeed = 0.0f;
    }
    
    // Publish the stride data as a JSON payload
    String payload = "{";
    payload += "\"gait_cycle_time\":"    + String(finalGaitCycleTime, 2) + ",";
    payload += "\"single_support_time\":" + String(finalSingleSupportTime, 2) + ",";
    payload += "\"cycle_distance\":"     + String(finalCycleDistance, 1) + ",";
    payload += "\"walking_speed\":"      + String(finalWalkingSpeed, 2);
    payload += "}";
    client.publish(topic, payload.c_str());
    Serial.println(payload);
    
    // Reset stride parameters for the next stride (zero-velocity update)
    lastStrideTime = contactTime;
    velocityX = 0.0f;
    positionX = 0.0f;
  }

  // 4) Detect foot lift (transition from stance to swing)
  if (!stancePhase && prevStance) {
    lastLiftTime = currentTime;
  }
  prevStance = stancePhase;

  // 5) If idle too long (no new foot contact), assume stationary and reset all metrics.
  if ((currentTime - lastStrideTime) > IDLE_TIMEOUT) {
    gaitCycleTime = 0.0f;
    singleSupportTime = 0.0f;
    cycleDistance = 0.0f;
    walkingSpeed = 0.0f;
    velocityX = 0.0f;
    positionX = 0.0f;
  }

  // 6) Integrate horizontal acceleration for naive stride distance (during swing)
  velocityX += (ax * dt);       // cm/s
  positionX += velocityX * dt;  // cm

  // Short delay for data rate (50 ms)
  delay(50);
}