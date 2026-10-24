# Sentrix: ESP32 Firmware Implementation Specification

This document defines the exact architectural requirements, security protocols, and data schemas the ESP32 firmware must implement to successfully communicate with the Sentrix Go backend and Python ML pipeline.

---

## 1. Core Configuration Constants

To ensure the firmware is ready for deployment and easily adjustable for different HiveMQ or Mosquitto environments, the following constants must be declared at the top of the sketch.

```cpp
// ===== NETWORK & BROKER CONFIGURATION =====
constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";

// Broker Endpoint (Adjustable for HiveMQ Cloud or local Mosquitto)
constexpr char MQTT_BROKER_HOST[] = "xxxx.s1.eu.hivemq.cloud";
constexpr uint16_t MQTT_BROKER_PORT = 8883; // MUST be 8883 for TLS

constexpr char MQTT_CLIENT_ID[] = "sentrix-root-node";
constexpr char MQTT_USERNAME[] = "your_device_username";
constexpr char MQTT_PASSWORD[] = "your_device_password";

// ===== HARDWARE IDENTITY =====
constexpr uint32_t DEVICE_PIN = 123456; // Required for Go API claiming
constexpr char MESH_ID[] = "MESH_89AB"; // Assigned by Go API upon group creation
```

---

## 2. Security & Connectivity (TLS Implementation)

The Go backend requires encrypted communication. The ESP32 cannot use the standard `WiFiClient`.

- **`WiFiClientSecure`:** You must implement `WiFiClientSecure` to handle the TLS connection over port 8883.
- **CA Certificate:** You must load the Root CA Certificate (e.g., Let's Encrypt ISRG Root X1 for HiveMQ Cloud) into the `WiFiClientSecure` object using `client.setCACert(root_ca)` before attempting to connect to the MQTT broker.

---

## 3. MQTT Topic Architecture

The firmware must publish to the exact topics expected by the Go API's subscription handlers.

| Action | Required MQTT Topic | Frequency |
| --- | --- | --- |
| Handshake / Auth | `sentrix/auth/{mac}/connect` | Once upon successful broker connection |
| Telemetry Batch | `sentrix/dt/{meshId}/batch` | Once every 60 seconds |

> **Note:** Replace `{mac}` with the device's actual MAC address, formatted as `AA:BB:CC:DD:EE:FF`, and `{meshId}` with the assigned group mesh ID.

---

## 4. JSON Payload Schemas

The ESP32 must use **ArduinoJson** to construct the following exact payload structures. Missing fields (like `device_pin` or `mesh_id`) will cause the Go API to reject the database insertion.

### A. The Handshake Payload (Published Once on Connect)

This authenticates the hardware against the database when a user claims the device in the Next.js dashboard.

```json
{
  "mac_address": "AA:BB:CC:DD:EE:FF",
  "device_pin": "123456",
  "mesh_id": "MESH_89AB",
  "role": "root",
  "firmware_version": "1.0.0",
  "timestamp": 1729584000
}
```

`timestamp` must be the current Unix epoch time retrieved via an NTP server pool (e.g., `pool.ntp.org`).

### B. The Telemetry Batch Payload (Published Every 60 Seconds)

To prevent overwhelming the broker and Go API with per-second writes, the ESP32 must batch readings.

```json
{
  "mesh_id": "MESH_89AB",
  "timestamp": 1729584060,
  "readings": [
    {
      "mac_address": "AA:BB:CC:DD:EE:FF",
      "rms_current": 15.4,
      "online": true
    }
  ]
}
```

- **Data Aggregation:** The ESP32 should sample the SCT-013 sensor continuously. The `rms_current` sent in the batch should be the calculated Root Mean Square (RMS) current at the moment the batch is generated, or an averaged reading over the 60-second window.
- **Mesh Scalability:** If operating as a standalone node, the `readings` array will only contain one object (itself). If operating as a true root node, it must append the MAC addresses and readings of its children to this array.

---

## 5. Execution Flow

1. **Boot & Initialize:** Start serial, set analog resolution (12-bit for ESP32), and initialize the SCT-013 pin.
2. **Time Sync:** Connect to Wi-Fi and immediately sync the internal RTC using an NTP server. Valid timestamps are mandatory for the Auto-ARIMA time-series database.
3. **Secure Connect:** Establish TLS connection to `MQTT_BROKER_HOST:8883`.
4. **Handshake:** Publish the Auth payload to `sentrix/auth/{mac}/connect`.
5. **Main Loop:**
   - Maintain MQTT connection (`mqtt.loop()`).
   - Sample the CT sensor to calculate AC RMS current.
   - Every 60,000 milliseconds, assemble the batch JSON payload and publish to `sentrix/dt/{meshId}/batch`.
