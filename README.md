# Distributed Smart Temperature Monitoring and Sensor Fusion Network

This repository contains the source code and documentation for a distributed Internet of Things (IoT) temperature-monitoring node based on an ESP32 microcontroller. 

## Overview
The system tackles sensor reliability by combining a fast-responding DS18B20 digital probe with a stable DHT22 ambient temperature and humidity reference. These complementary measurements are calibrated and fused at the edge using a response-aware 1D Sequential Kalman Filter. The resulting optimized temperature estimate, along with humidity and NEO-6M GPS location data, is transmitted via Wi-Fi using the MQTT protocol to a central Ignition SCADA dashboard for real-time visualization.

## Key Features
*   **Multi-Sensor Data Acquisition:** Integrates DS18B20 for fast transient tracking and DHT22 for a stable ambient baseline.
*   **Edge Processing & Calibration:** Performs real-time linear calibration on the raw DS18B20 sensor data to align with the DHT22 baseline before fusion.
*   **Advanced Sensor Fusion:** Utilizes a 1D Sequential Kalman Filter to dynamically adjust sensor trust based on the current rate of change and established thermal time constants (9.3s for DS18B20 and 80.9s for DHT22).
*   **Location Tracking:** Integrates a NEO-6M GPS module for spatial mapping.
*   **Wireless Telemetry:** Transmits structured JSON payloads over Wi-Fi using MQTT.
*   **SCADA Integration:** Interfaces with Ignition SCADA for real-time temperature tracking, historical data logging, and remote LED actuator control.

## Mathematical Formulation
**Sensor Calibration:** The DS18B20 is mathematically corrected to align with the DHT22 using the following derived regression formula:
`y = 1.0318x - 0.6398`

**Sensor Fusion:** The Kalman filter assigns dynamic fractional weights (w_D and w_H) to calculate the final fused temperature (T_f):
`T_f = w_D * T_D + w_H * T_H`

**Distributed Spatial Estimation:** The central SCADA system can estimate temperatures at unknown locations using Inverse Distance Weighting (IDW):
`T(x,y) = Σ(T_i / d_i^p) / Σ(1 / d_i^p)`

## Hardware & Pin Mapping
| Component | Pin on Module | ESP32 GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **DHT22** | DATA | GPIO 15 | Requires 10kΩ pull-up resistor to 3.3V |
| **DS18B20** | DATA (Yellow) | GPIO 4 | Requires 4.7kΩ pull-up resistor to 3.3V |
| **NEO-6M GPS** | TX | GPIO 16 (RX2) | Serial receive |
| **NEO-6M GPS** | RX | GPIO 17 (TX2) | Serial transfer |
| **Status LED** | Positive Anode | GPIO 2 | Built-in ESP32 LED |

## Software Dependencies
To compile this project, the following Arduino libraries are required:
*   `WiFi.h` (Built-in)
*   `PubSubClient.h` (MQTT communication)
*   `DHT.h` and Adafruit Unified Sensor
*   `OneWire.h` and `DallasTemperature.h`
*   `TinyGPS++.h`

## Setup and Configuration
1.  Clone this repository to your local machine.
2.  Open the source code located in the `src` directory using the Arduino IDE.
3.  Install the required dependencies via the Arduino Library Manager.
4.  Update the network configuration variables at the top of the `.ino` file with your specific credentials (e.g., `WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_BROKER_HOST`).
5.  Wire the hardware according to the pin mapping table above.
6.  Upload the code to your ESP32.

## Team (Group 18)
Developed by Group 18 members: 
*   E/22/158
*   E/22/238
*   E/22/258
*   E/22/345
*   E/22/348
*   E/22/357
