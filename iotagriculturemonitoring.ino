#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// WiFi Config
#define WIFI_SSID     "HUAWEI"
#define WIFI_PASSWORD "tragedijasjus"

// Pin Setup
#define DHTPIN 26
#define SOIL_PIN 35
#define GAS_PIN 33
#define RELAY_PIN 14

#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// MQTT Config
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/relay";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// NTP Client Setup
WiFiUDP udp;
NTPClient timeClient(udp, "pool.ntp.org", 3600, 60000);  // Timezone and update every 60 seconds

// Threshold Defaults
const float DEFAULT_SOIL_MIN = 30.0;
const float DEFAULT_TEMP_MIN = 25.0;
const float DEFAULT_TEMP_MAX = 32.0;
const float DEFAULT_HUMIDITY_MIN = 60.0;
const float DEFAULT_HUMIDITY_MAX = 70.0;
const float DEFAULT_GAS_MAX = 1000.0;
const bool  DEFAULT_AUTO_MODE = true;

// Active Thresholds
float SOIL_MIN = DEFAULT_SOIL_MIN;
float TEMP_MIN = DEFAULT_TEMP_MIN;
float TEMP_MAX = DEFAULT_TEMP_MAX;
float HUMIDITY_MIN = DEFAULT_HUMIDITY_MIN;
float HUMIDITY_MAX = DEFAULT_HUMIDITY_MAX;
float GAS_MAX = DEFAULT_GAS_MAX;
bool autoMode = DEFAULT_AUTO_MODE;

unsigned long lastSensorRead = 0;
unsigned long lastConfigFetch = 0;
unsigned long lastWateringCheck = 0;
unsigned long relayOnTime = 0; // Used to track how long relay has been ON

float latestTemperature = 0.0;
float latestHumidity = 0.0;
float latestGas = 0.0;
float latestSoilRaw = 0.0;
float latestSoilPercent = 0.0;

const String BASE_URL = "https://iot-agriculture-monitoring.vercel.app/api";

unsigned long lastSensorUpdate = 0;  // untuk update nilai sensor
unsigned long lastSensorPost = 0;    // untuk post log sensor setiap 5 menit

// Connect WiFi
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Connected to WiFi");
}

// MQTT Reconnection
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("esp32-client-001")) {
      Serial.println("✅ MQTT connected");
      mqttClient.subscribe(mqtt_topic);
    } else {
      Serial.print("❌ Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 sec");
      delay(5000);
    }
  }
}

// Post Water Log
void postWaterLog(String mode, int durationMs, float temperature, float humidity, float gas, float soilRaw, float soilPercent) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(BASE_URL + "/watering-logs");
    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<512> doc;
    doc["deviceId"] = "esp32-001";
    doc["mode"] = mode;
    doc["durationMs"] = durationMs;

    // Timestamp with NTP time
    timeClient.update();
    unsigned long now = timeClient.getEpochTime();
    doc["startedAt"] = now;
    doc["endedAt"] = now + durationMs;

    JsonObject sensor = doc.createNestedObject("sensorSnapshot");
    sensor["temperature"] = temperature;
    sensor["humidity"] = humidity;
    sensor["gasLevel"] = gas;
    sensor["soilMoistureRaw"] = soilRaw;
    sensor["soilMoisturePercent"] = soilPercent;

    String body;
    serializeJson(doc, body);
    int httpCode = http.POST(body);
    if (httpCode == 200 || httpCode == 201) {
      Serial.println("✅ Watering log sent (with sensors)");
    } else {
      Serial.printf("❌ Failed to send watering log: %d\n", httpCode);
    }
    http.end();
  }
}

// Callback Function for MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == mqtt_topic) {
    Serial.printf("📨 MQTT Message: %s\n", message.c_str());
    if (message == "ON") {
      digitalWrite(RELAY_PIN, HIGH);
      relayOnTime = millis();  // Set the relay ON time to current millis
      Serial.println("🚿 Relay ON (MQTT)");
    } else if (message == "OFF") {
      digitalWrite(RELAY_PIN, LOW);
      unsigned long duration = millis() - relayOnTime;
      Serial.println("💤 Relay OFF (MQTT)");
      // Send Watering Log for Manual mode
      postWaterLog("manual", duration, latestTemperature, latestHumidity, latestGas, latestSoilRaw, latestSoilPercent);
    }
  }
}

// Fetch Thresholds from Server
void fetchThresholds() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(BASE_URL + "/device-config?device_id=esp32-001");

    int httpCode = http.GET();
    if (httpCode == 200) {
      String payload = http.getString();
      StaticJsonDocument<768> doc;
      DeserializationError err = deserializeJson(doc, payload);
      if (!err) {
        JsonObject data = doc["data"];
        if (!data.isNull()) {
          SOIL_MIN = data["soil_min"] | DEFAULT_SOIL_MIN;
          TEMP_MIN = data["temp_min"] | DEFAULT_TEMP_MIN;
          TEMP_MAX = data["temp_max"] | DEFAULT_TEMP_MAX;
          HUMIDITY_MIN = data["humidity_min"] | DEFAULT_HUMIDITY_MIN;
          HUMIDITY_MAX = data["humidity_max"] | DEFAULT_HUMIDITY_MAX;
          GAS_MAX = data["gas_max"] | DEFAULT_GAS_MAX;
          autoMode = data["auto_mode"] | DEFAULT_AUTO_MODE;
          Serial.println("✅ Thresholds updated from server");
        }
      } else {
        Serial.println("⚠️ Failed to parse JSON");
      }
    } else {
      Serial.printf("⚠️ Failed to fetch thresholds: %d\n", httpCode);
    }
    http.end();
  }
}

// Post Sensor Log
void postSensorLog(float temperature, float humidity, float gasValue, float soilRaw, float soilPercent) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(BASE_URL + "/sensor-logs");
    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<256> jsonDoc;
    jsonDoc["deviceId"] = "esp32-001";
    jsonDoc["temperature"] = temperature;
    jsonDoc["humidity"] = humidity;
    jsonDoc["gasLevel"] = gasValue;
    jsonDoc["soilMoistureRaw"] = soilRaw;
    jsonDoc["soilMoisturePercent"] = soilPercent;

    String requestBody;
    serializeJson(jsonDoc, requestBody);

    int httpCode = http.POST(requestBody);
    if (httpCode == 200 || httpCode == 201) {
      Serial.println("✅ Sensor log sent");
    } else {
      Serial.printf("❌ Sensor log failed: %d\n", httpCode);
    }

    http.end();
  }
}

// ✅ Kirim Sensor Terbaru (format payload sesuai API)
void postLatestSensor(float temperature, float humidity, float gasValue, float soilPercent) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(BASE_URL + "/sensor-latest");
    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<256> jsonDoc;
    jsonDoc["device_id"] = "esp32-001";
    jsonDoc["temperature"] = temperature;
    jsonDoc["humidity"] = humidity;
    jsonDoc["soil_moisture"] = soilPercent;
    jsonDoc["gas_level"] = gasValue;

    String requestBody;
    serializeJson(jsonDoc, requestBody);

    int httpCode = http.POST(requestBody);
    if (httpCode == 200 || httpCode == 201) {
      Serial.println("✅ Latest sensor log sent successfully");
    } else {
      Serial.printf("❌ Failed to send latest sensor log: %d\n", httpCode);
    }

    http.end();
  } else {
    Serial.println("⚠️ WiFi not connected, cannot send latest sensor log");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  dht.begin();

  connectWiFi();
  timeClient.begin();  // Start NTP client
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);
  fetchThresholds();
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  unsigned long currentMillis = millis();

   // Update sensor setiap 5 detik
  if (currentMillis - lastSensorUpdate >= 5000) {
    lastSensorUpdate = currentMillis;

    float soilRaw = analogRead(SOIL_PIN);
    float soilPercent = map(soilRaw, 2545, 880, 0, 100); // Pakai versi kapasitive
    // float soilPercent = map(soilRaw, 4095, 1002, 0, 100);
    float gasValue = analogRead(GAS_PIN);
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    latestTemperature = temperature;
    latestHumidity = humidity;
    latestGas = gasValue;
    latestSoilRaw = soilRaw;
    latestSoilPercent = soilPercent;

    Serial.printf("\n📊 Soil: %.1f%% | Gas: %.0f | Temp: %.1f°C | Hum: %.1f%%\n",
                  soilPercent, gasValue, temperature, humidity);
Serial.printf("soil raw %.1f", soilRaw);
    postLatestSensor(latestTemperature, latestHumidity, latestGas, latestSoilPercent);

    // Logic Auto Watering
    bool soilOK = soilPercent < SOIL_MIN;
    bool tempOK = temperature >= TEMP_MIN && temperature <= TEMP_MAX;
    bool humidityOK = humidity >= HUMIDITY_MIN && humidity <= HUMIDITY_MAX;
    bool gasOK = gasValue <= GAS_MAX;

    bool needWater = soilOK && tempOK && humidityOK && gasOK;

    if (autoMode) {
      if (needWater) {
        digitalWrite(RELAY_PIN, HIGH);
        Serial.println("🚿 Relay ON (Auto Mode)");
        postWaterLog("auto", 5000, latestTemperature, latestHumidity, latestGas, latestSoilRaw, latestSoilPercent);
      } else {
        digitalWrite(RELAY_PIN, LOW);
        Serial.println("💤 Relay OFF (Auto Mode)");
      }
    } else {
      Serial.println("🔘 Manual mode, relay hanya dikendalikan oleh MQTT");
    }
  }

  // Post sensor log setiap 5 menit
  // if (currentMillis - lastSensorPost >= 300000) {
  if (currentMillis - lastSensorPost >= 300000) {
    lastSensorPost = currentMillis;
    postSensorLog(latestTemperature, latestHumidity, latestGas, latestSoilRaw, latestSoilPercent);
  }

  // Fetch and update the device config every 5 seconds
  if (currentMillis - lastConfigFetch >= 5000) {
    lastConfigFetch = currentMillis;
    fetchThresholds();
  }
}
