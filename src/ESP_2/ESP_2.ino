#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// ============================================================
// WiFi & MQTT Configuration (ESP02)
// ============================================================
const char* ssid = "Vishwani_iPhone";
const char* password = "Vlm@2002";

const char* mqtt_server = "172.20.10.6";
const int mqtt_port = 1883;

const char* mqtt_clientid = "ESP02";
const char* mqtt_user = "esp02";
const char* mqtt_password = "stud2";

// MQTT Topics (Updated for Ignition folder hierarchy)
const char* temp_topic         = "EE2120/ESP02/temp/fused";
const char* temp_raw_dht_topic = "EE2120/ESP02/temp/raw_dht";
const char* temp_raw_ds_topic  = "EE2120/ESP02/temp/raw_ds";
const char* led_topic          = "EE2120/ESP02/LED/cmd";
const char* led_status_topic   = "EE2120/ESP02/LED/status";
const char* gps_topic          = "EE2120/ESP02/gps";

// ============================================================
// Objects (WiFi, MQTT, GPS)
// ============================================================
WiFiClient espClient;
PubSubClient client(espClient);

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// Timer for non-blocking loop
unsigned long lastPublishTime = 0;
const long publishInterval = 2000;

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
// SENSOR CALIBRATION & KALMAN FILTER (ESP02 Specific)
// ============================================================
float calibrateDHT22(float rawTemperature) {
    return rawTemperature;
}

float calibrateDS18B20(float rawTemperature) {
    return (1.0318 * rawTemperature) - 0.6398;
}

float processNoise = 0.05;
float varianceDHT = 0.00246; 
float varianceDS  = 0.00070; 
float currentEstimate = 0.0;
float errorCovariance = 1.0;
bool kalmanInitialized = false;

float getKalmanFusedTemperature(float dhtTemp, float dsTemp) {
    if (!kalmanInitialized) {
        currentEstimate = dsTemp; 
        errorCovariance = 1.0;
        kalmanInitialized = true;
        return currentEstimate;
    }

    float predictedEstimate = currentEstimate;
    float predictedErrorCovariance = errorCovariance + processNoise;
    float rateOfChange = abs(dsTemp - currentEstimate);
    float dynamicVarianceDHT = varianceDHT + (rateOfChange * 0.5); 

    float kalmanGainDS = predictedErrorCovariance / (predictedErrorCovariance + varianceDS);
    float estimateAfterDS = predictedEstimate + kalmanGainDS * (dsTemp - predictedEstimate);
    float errorCovarianceAfterDS = (1.0 - kalmanGainDS) * predictedErrorCovariance;

    float kalmanGainDHT = errorCovarianceAfterDS / (errorCovarianceAfterDS + dynamicVarianceDHT);
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
    
    if (String(topic) == led_topic) {
        if (message == "1") {
            digitalWrite(LED_BUILTIN, HIGH);
            client.publish(led_status_topic, "1");
        }
        else if (message == "0") {
            digitalWrite(LED_BUILTIN, LOW);
            client.publish(led_status_topic, "0");
        }
    }
}

// ============================================================
// SETUP & LOOP
// ============================================================
void setup_wifi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
}

void reconnect() {
    while (!client.connected()) {
        if (client.connect(mqtt_clientid, mqtt_user, mqtt_password)) {
            client.subscribe(led_topic);
        } else {
            delay(2000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    dht.begin();
    ds18b20.begin();

    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
}

void loop() {
    if (!client.connected()) reconnect();
    client.loop();

    while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());

    if (millis() - lastPublishTime >= publishInterval) {
        lastPublishTime = millis();

        float rawDHT = dht.readTemperature();
        ds18b20.requestTemperatures();
        float rawDS = ds18b20.getTempCByIndex(0);

        if (isnan(rawDHT) || rawDS == DEVICE_DISCONNECTED_C) return;

        float calibratedDHT = calibrateDHT22(rawDHT);
        float calibratedDS = calibrateDS18B20(rawDS);
        float fusedTemp = getKalmanFusedTemperature(calibratedDHT, calibratedDS);

        // --- PUBLISH FUSED TEMP ---
        char tempString[16];
        dtostrf(fusedTemp, 1, 2, tempString);
        client.publish(temp_topic, tempString);
        
        // --- PUBLISH RAW DHT TEMP ---
        char rawDhtString[16];
        dtostrf(rawDHT, 1, 2, rawDhtString);
        client.publish(temp_raw_dht_topic, rawDhtString);

        // --- PUBLISH RAW DS TEMP ---
        char rawDsString[16];
        dtostrf(rawDS, 1, 2, rawDsString);
        client.publish(temp_raw_ds_topic, rawDsString);

        // --- PUBLISH GPS ---
        char gpsString[32];
        if (gps.location.isValid()) {
            snprintf(gpsString, sizeof(gpsString), "%.6f,%.6f", gps.location.lat(), gps.location.lng());
        } else {
            snprintf(gpsString, sizeof(gpsString), "Searching...");
        }
        client.publish(gps_topic, gpsString);
        
        Serial.print("ESP02 Fused Temp: "); Serial.println(tempString);
        Serial.print("ESP02 Raw DHT   : "); Serial.println(rawDhtString);
        Serial.print("ESP02 Raw DS    : "); Serial.println(rawDsString);
        Serial.print("ESP02 GPS       : "); Serial.println(gpsString);
    }
}