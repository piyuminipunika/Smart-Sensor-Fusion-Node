#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- WiFi & MQTT Configuration ---
const char* ssid = "iPhone";
const char* password = "11223344";
const char* mqtt_server = "172.20.10.3";
const int mqtt_port = 1883;
const char* mqtt_clientid = "ESP01";
const char* mqtt_user = "esp01";
const char* mqtt_password = "stud1";

const char* temp_topic = "EE2120/ESP01/temp";
const char* led_topic  = "EE2120/ESP01/LED/cmd";
const char* led_status_topic  = "EE2120/ESP01/LED/status";
const char* gps_topic = "EE2120/ESP01/gps"; 

WiFiClient espClient;
PubSubClient client(espClient);

// Built-in LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// --- Sensor Configuration ---
#define DHTPIN 15
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// --- Kalman Filter Tuning Parameters ---
float processNoise = 0.05; 
float varianceDHT = 5.00;  
float varianceDS = 0.01;   
float currentEstimate = 25.0; 
float errorCovariance = 1.0;  

// ------------------------------------------------
// Kalman Filter Function
// ------------------------------------------------
float getKalmanFusedTemperature(float dhtTemp, float dsTemp) {
    float predictedEstimate = currentEstimate;
    float predictedErrorCov = errorCovariance + processNoise;

    float kalmanGain1 = predictedErrorCov / (predictedErrorCov + varianceDS);
    float estimate1 = predictedEstimate + kalmanGain1 * (dsTemp - predictedEstimate);
    float errorCov1 = (1.0 - kalmanGain1) * predictedErrorCov;

    float kalmanGain2 = errorCov1 / (errorCov1 + varianceDHT);
    currentEstimate = estimate1 + kalmanGain2 * (dhtTemp - estimate1);
    errorCovariance = (1.0 - kalmanGain2) * errorCov1;

    return currentEstimate;
}

// ------------------------------------------------
// MQTT Callback
// ------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Message received: ");
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(message);

  if (String(topic) == led_topic) {
    if (message == "1") {
      digitalWrite(LED_BUILTIN, HIGH);
      client.publish(led_status_topic, "1");
      Serial.println("LED ON");
    }
    if (message == "0") {
      digitalWrite(LED_BUILTIN, LOW);
      client.publish(led_status_topic, "0");
      Serial.println("LED OFF");
    }
  }
}

// ------------------------------------------------
// Connect to Wi-Fi
// ------------------------------------------------
void setup_wifi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

// ------------------------------------------------
// Connect to MQTT
// ------------------------------------------------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(mqtt_clientid, mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(led_topic);
      Serial.println("Subscribed to EE2120/ESP01/LED/cmd");
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// ------------------------------------------------
// Setup
// ------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  dht.begin();
  ds18b20.begin();

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

// ------------------------------------------------
// Main loop
// ------------------------------------------------
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // 1. Read Sensors
  float rawDHT = dht.readTemperature();
  ds18b20.requestTemperatures(); 
  float rawDS = ds18b20.getTempCByIndex(0);

  if (isnan(rawDHT) || rawDS == DEVICE_DISCONNECTED_C) {
    Serial.println("Sensor Error: Skipping Transmission");
    delay(2000);
    return;
  }

  // 2. Apply Calibration Curves
  float calibratedDHT = (rawDHT - 0.9000) / 0.9892;
  float calibratedDS = (rawDS + 0.0600) / 0.9989;

  // 3. Execute Kalman Fusion
  float fusedTemp = getKalmanFusedTemperature(calibratedDHT, calibratedDS);

  // 4. Publish Fused Data to MQTT
  char tempString[8];
  dtostrf(fusedTemp, 1, 2, tempString); // Converts float to string with 2 decimal places

  client.publish(temp_topic, tempString);
  Serial.print("Published Temp: ");
  Serial.println(tempString);
  
  // 5. Publish Simulated GPS (Placeholder)
  const char* fakeGpsData = "7.2450, 80.6850"; 
  client.publish(gps_topic, fakeGpsData);
  Serial.print("Published GPS: ");
  Serial.println(fakeGpsData);

  // Wait 2 seconds (DHT22 hardware limit)
  delay(2000);
}