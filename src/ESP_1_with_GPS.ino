#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// ============================================================
// WiFi & MQTT Configuration
// ============================================================
const char* ssid = "sasanda's Tab S7 FE";
const char* password = "zmvz6300";

const char* mqtt_server = "10.15.0.3";
const int mqtt_port = 1883;

const char* mqtt_clientid = "ESP01";
const char* mqtt_user = "esp01";
const char* mqtt_password = "stud1";

// MQTT Topics
const char* temp_topic       = "EE2120/ESP01/temp";
const char* led_topic        = "EE2120/ESP01/LED/cmd";
const char* led_status_topic = "EE2120/ESP01/LED/status";
const char* gps_topic        = "EE2120/ESP01/gps";

// ============================================================
// Objects (WiFi, MQTT, GPS)
// ============================================================
WiFiClient espClient;
PubSubClient client(espClient);

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// Timer for non-blocking loop
unsigned long lastPublishTime = 0;
const long publishInterval = 2000; // 2 seconds

// ============================================================
// Built-in LED
// ============================================================
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ============================================================
// Sensor Configuration
// ============================================================
#define DHTPIN 15
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// ============================================================
// SENSOR CALIBRATION
// ============================================================
float calibrateDHT22(float rawTemperature) {
    return (rawTemperature - 0.9000) / 0.9892;
}

float calibrateDS18B20(float rawTemperature) {
    return (rawTemperature + 0.0600) / 0.9989;
}

// ============================================================
// KALMAN FILTER PARAMETERS & STATE
// ============================================================
float processNoise = 0.05;
float varianceDHT = 0.00246;
float varianceDS  = 0.00070;

float currentEstimate = 0.0;
float errorCovariance = 1.0;
bool kalmanInitialized = false;

float getKalmanFusedTemperature(float dhtTemp, float dsTemp) {
    if (!kalmanInitialized) {
        currentEstimate = (dhtTemp + dsTemp) / 2.0;
        errorCovariance = 1.0;
        kalmanInitialized = true;
        return currentEstimate;
    }

    float predictedEstimate = currentEstimate;
    float predictedErrorCovariance = errorCovariance + processNoise;

    float kalmanGainDS = predictedErrorCovariance / (predictedErrorCovariance + varianceDS);
    float estimateAfterDS = predictedEstimate + kalmanGainDS * (dsTemp - predictedEstimate);
    float errorCovarianceAfterDS = (1.0 - kalmanGainDS) * predictedErrorCovariance;

    float kalmanGainDHT = errorCovarianceAfterDS / (errorCovarianceAfterDS + varianceDHT);
    currentEstimate = estimateAfterDS + kalmanGainDHT * (dhtTemp - estimateAfterDS);
    errorCovariance = (1.0 - kalmanGainDHT) * errorCovarianceAfterDS;

    return currentEstimate;
}

// ============================================================
// MQTT CALLBACK
// ============================================================
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
        else if (message == "0") {
            digitalWrite(LED_BUILTIN, LOW);
            client.publish(led_status_topic, "0");
            Serial.println("LED OFF");
        }
    }
}

// ============================================================
// SETUP WIFI & MQTT
// ============================================================
void setup_wifi() {
    Serial.println();
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP());
}

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

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    
    // Start GPS hardware serial (RX=16, TX=17)
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    dht.begin();
    ds18b20.begin();

    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    Serial.println("\nSystem initialization complete.\nWaiting for GPS lock...");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
    // 1. Maintain MQTT Connection
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // 2. Continually Feed GPS Parser (Non-Blocking)
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    // 3. Sensor Reading & Publishing Timer (Triggers every 2 seconds)
    if (millis() - lastPublishTime >= publishInterval) {
        lastPublishTime = millis(); // Reset timer

        // --- READ SENSORS ---
        float rawDHT = dht.readTemperature();
        ds18b20.requestTemperatures();
        float rawDS = ds18b20.getTempCByIndex(0);

        if (isnan(rawDHT) || rawDS == DEVICE_DISCONNECTED_C) {
            Serial.println("ERROR: Sensor reading failed. Skipping this cycle.");
            return; // Escapes the timer block early if a sensor fails
        }

        // --- CALIBRATE & FUSE ---
        float calibratedDHT = calibrateDHT22(rawDHT);
        float calibratedDS = calibrateDS18B20(rawDS);
        float fusedTemp = getKalmanFusedTemperature(calibratedDHT, calibratedDS);

        // --- SERIAL MONITOR PRINTING ---
        Serial.println("\n--------------------------------------");
        Serial.print("DHT22 Calibrated: "); Serial.print(calibratedDHT, 2); Serial.println(" °C");
        Serial.print("DS18B20 Calibrated: "); Serial.print(calibratedDS, 2); Serial.println(" °C");
        Serial.print("Kalman Fused    : "); Serial.print(fusedTemp, 2); Serial.println(" °C");

        // --- PUBLISH TEMPERATURE ---
        char tempString[16];
        dtostrf(fusedTemp, 1, 2, tempString);
        client.publish(temp_topic, tempString);
        Serial.print("MQTT Temperature: ");
        Serial.println(tempString);

        // --- PUBLISH GPS ---
        char gpsString[32];
        
        // Check if GPS has a valid satellite fix
        if (gps.location.isValid()) {
            // Format valid coordinates into "Lat,Lng"
            snprintf(gpsString, sizeof(gpsString), "%.6f,%.6f", gps.location.lat(), gps.location.lng());
        } else {
            // If no fix yet, publish a status message
            snprintf(gpsString, sizeof(gpsString), "Searching...");
        }
        
        client.publish(gps_topic, gpsString);
        Serial.print("MQTT GPS        : ");
        Serial.println(gpsString);
        Serial.println("--------------------------------------");
    }
}