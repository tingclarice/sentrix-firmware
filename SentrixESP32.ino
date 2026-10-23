/* Sentrix sensor node: classic ESP32 DevKit V1 / ESP32-WROOM.
 * Tested: ESP32 core 3.3.12; PubSubClient 2.8, IRremoteESP8266 2.9.0, ArduinoJson 7.4.3.
 * Single-file firmware: all settings and logic are in this sketch.
 * Reads the physical CT circuit on GPIO34. See README.md for wiring.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRac.h>
#include <IRutils.h>
#include <time.h>
#include <esp_system.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLESecurity.h>
#include <Preferences.h>
#include <atomic>
#include <Ticker.h>
#include <mbedtls/ecdh.h>
#include <mbedtls/md.h>
#include <mbedtls/gcm.h>
#include <mbedtls/platform_util.h>

// ===== USER SETTINGS: edit here, then upload =====
// Change this per device before deployment; enter it only in the website PIN form.
constexpr uint32_t HARDWARE_PIN = 715204;
static_assert(HARDWARE_PIN <= 999999, "HARDWARE_PIN must fit six digits");
constexpr char SERVICE_UUID[] = "12345678-1234-5678-1234-56789abcdef0";
constexpr char IDENTITY_UUID[] = "12345678-1234-5678-1234-56789abcdef1";
constexpr char CONFIG_UUID[] = "12345678-1234-5678-1234-56789abcdef2";
constexpr char STATUS_UUID[] = "12345678-1234-5678-1234-56789abcdef3";
constexpr char SESSION_UUID[] = "12345678-1234-5678-1234-56789abcdef4";
constexpr char WIFI_SSID[] = "";
constexpr char WIFI_PASSWORD[] = "";
// Local PicoMQTT or Mosquitto broker IP. Empty = Serial-only operation.
constexpr char MQTT_HOST[] = "";
constexpr uint16_t MQTT_PORT = 1883;
constexpr char MQTT_USER[] = "";
constexpr char MQTT_PASSWORD[] = "";

constexpr uint8_t CURRENT_PIN = 34;
constexpr uint8_t IR_PIN = 35;
constexpr uint8_t RED_LED_PIN = 4;
constexpr uint8_t GREEN_LED_PIN = 2;
constexpr bool RGB_LED_ENABLED = true;
constexpr bool RGB_COMMON_ANODE = false; // false: common to GND; true: common to 3.3V.
constexpr uint8_t RGB_RED_PIN = 25;
constexpr uint8_t RGB_GREEN_PIN = 26;
constexpr uint8_t RGB_BLUE_PIN = 27;

// Bits 0..2 are R/G/B; bit 3 blinks slowly, bit 4 blinks quickly.
constexpr uint8_t rgbState(bool fault, bool busy, bool bluetooth, bool configured,
                           bool wifi, bool broker) {
  return fault ? 17 : busy ? 11 : bluetooth ? 4 : !configured ? 12
       : !wifi ? 9 : broker ? 2 : 3;
}
static_assert(rgbState(true, true, true, true, true, true) == 17, "Fault overrides setup");
static_assert(rgbState(false, true, true, true, true, true) == 11, "Setup blinks yellow");
static_assert(rgbState(false, false, true, true, true, true) == 4, "BLE is blue");
static_assert(rgbState(false, false, false, false, false, false) == 12, "Unconfigured blinks blue");
static_assert(rgbState(false, false, false, true, false, false) == 9, "Wi-Fi loss blinks red");
static_assert(rgbState(false, false, false, true, true, false) == 3, "No broker is yellow");
static_assert(rgbState(false, false, false, true, true, true) == 2, "Connected is green");
std::atomic<uint8_t> rgbPattern{12};
std::atomic<bool> bluetoothConnected{false};
Ticker rgbTicker;

void renderRgb() {
  const uint8_t pattern = rgbPattern.load();
  const bool lit = pattern & 16 ? (millis() / 150) % 2 == 0
                 : pattern & 8 ? (millis() / 500) % 2 == 0 : true;
  digitalWrite(RGB_RED_PIN, ((pattern & 1) && lit) != RGB_COMMON_ANODE);
  digitalWrite(RGB_GREEN_PIN, ((pattern & 2) && lit) != RGB_COMMON_ANODE);
  digitalWrite(RGB_BLUE_PIN, ((pattern & 4) && lit) != RGB_COMMON_ANODE);
}

// Assumes SCT-013-000: 100 A primary / 50 mA secondary (2000:1).
// These values DO NOT apply to SCT-013 variants with a built-in burden.
constexpr double CT_RATIO = 2000.0;
constexpr double BURDEN_OHMS = 22.0;
constexpr double CURRENT_CALIBRATION = 1.0;
constexpr double NOMINAL_VOLTAGE = 220.0;
constexpr double STANDBY_CURRENT_A = 0.15;
constexpr double ACTIVE_CURRENT_A = 0.25;
// Demonstration warning threshold, not an electrical protection device.
constexpr double HIGH_CURRENT_A = 10.0;
constexpr uint32_t REPORT_INTERVAL_MS = 1000;
constexpr uint32_t SAMPLE_INTERVAL_US = 500; // nominal 2 kHz
constexpr uint32_t SAMPLE_WINDOW_US = 200000; // 10 x 50 Hz or 12 x 60 Hz

// ===== MEASUREMENT TYPES =====
#include <math.h>
#include <stdint.h>

struct Measurement {
  double current = 0;
  double bias = 0;
  double minimum = 0;
  double maximum = 0;
  double sampleRate = 0;
  uint32_t samples = 0;
  uint32_t missedSlots = 0;
  bool valid = false;
};

// Welford variance removes the measured DC bias without a fixed ADC offset.
struct RmsAccumulator {
  uint32_t count = 0;
  double mean = 0;
  double m2 = 0;
  double minimum = 1e9;
  double maximum = -1e9;
  void add(double millivolts) {
    ++count;
    const double delta = millivolts - mean;
    mean += delta / count;
    m2 += delta * (millivolts - mean);
    if (millivolts < minimum) minimum = millivolts;
    if (millivolts > maximum) maximum = millivolts;
  }
  double rmsVolts() const {
    return count ? sqrt(fmax(0.0, m2 / count)) / 1000.0 : 0.0;
  }
};

// Explicit declarations keep Arduino preprocessing independent of type order.
Measurement readCurrent();
void reportMeasurement(const Measurement &value);

WiFiClient transport;
PubSubClient mqtt(transport);
String deviceId, telemetryTopic, statusTopic, irTopic;
char bootId[9];
uint32_t sequence = 0, nextReport = 0, nextWiFi = 0, nextMqtt = 0;
uint32_t lastIr = 0;
bool seenIr = false, active = false, sensorFault = false, highCurrent = false;
bool wifiWasConnected = false;
// Keep the last decoded setpoint; NAN means no usable temperature received yet.
float remoteTemperatureC = NAN;
IRrecv irReceiver(IR_PIN, 1024, 50, true);
decode_results irResults;

constexpr float setpointCelsius(float degrees, bool celsius) {
  const float value = celsius ? degrees : (degrees - 32.0f) * 5.0f / 9.0f;
  return value >= 5.0f && value <= 40.0f ? value : NAN;
}
static_assert(setpointCelsius(24.0f, true) == 24.0f, "Celsius conversion");
static_assert(setpointCelsius(77.0f, false) == 25.0f, "Fahrenheit conversion");
static_assert(!(setpointCelsius(0.0f, true) == setpointCelsius(0.0f, true)),
              "Reject missing/sentinel temperature");
static_assert(!(setpointCelsius(100.0f, true) == setpointCelsius(100.0f, true)),
              "Reject invalid temperature");
String wifiSsid = WIFI_SSID, wifiPassword = WIFI_PASSWORD, groupId;
BLECharacteristic *bleStatus = nullptr;
QueueHandle_t provisioningQueue;
std::atomic<bool> provisioningBusy{false};
struct ProvisionRequest {
  char ssid[33];
  char password[65];
  char group[65];
};
String bleInput;

BLECharacteristic *bleSession = nullptr;
BLECharacteristic *bleNetworks = nullptr;
std::atomic<bool> wifiScanPending(false);
mbedtls_ecp_group sessionGroup;
mbedtls_mpi sessionPrivate;
mbedtls_ecp_point sessionPublic;
uint8_t sessionChallenge[16], sessionKey[32];
bool sessionReady = false, sessionAuthenticated = false;
uint8_t pinFailures = 0;
uint32_t pinLockedUntil = 0;

int cryptoRandom(void *, unsigned char *output, size_t length) {
  esp_fill_random(output, length);
  return 0;
}

bool readHex(const String &value, uint8_t *output, size_t length) {
  if (value.length() != length * 2) return false;
  for (size_t i = 0; i < length; ++i) {
    auto digit = [](char c) -> int {
      if (c >= '0' && c <= '9') return c - '0';
      if (c >= 'a' && c <= 'f') return c - 'a' + 10;
      return -1;
    };
    const int high = digit(value[i * 2]), low = digit(value[i * 2 + 1]);
    if (high < 0 || low < 0) return false;
    output[i] = (high << 4) | low;
  }
  return true;
}

bool sessionProof(const char *label, uint8_t *output) {
  return mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
      sessionKey, sizeof(sessionKey), (const uint8_t *)label, strlen(label), output) == 0;
}

void beginSession() {
  sessionReady = sessionAuthenticated = false;
  mbedtls_platform_zeroize(sessionKey, sizeof(sessionKey));
  mbedtls_ecp_group_free(&sessionGroup);
  mbedtls_mpi_free(&sessionPrivate);
  mbedtls_ecp_point_free(&sessionPublic);
  mbedtls_ecp_group_init(&sessionGroup);
  mbedtls_mpi_init(&sessionPrivate);
  mbedtls_ecp_point_init(&sessionPublic);
  uint8_t hello[81];
  size_t publicLength = 0;
  if (mbedtls_ecp_group_load(&sessionGroup, MBEDTLS_ECP_DP_SECP256R1) ||
      mbedtls_ecp_gen_keypair(&sessionGroup, &sessionPrivate, &sessionPublic, cryptoRandom, nullptr) ||
      mbedtls_ecp_point_write_binary(&sessionGroup, &sessionPublic, MBEDTLS_ECP_PF_UNCOMPRESSED,
                                    &publicLength, hello, 65) || publicLength != 65) {
    bleStatus->setValue("crypto_failed");
    return;
  }
  esp_fill_random(sessionChallenge, sizeof(sessionChallenge));
  memcpy(hello + 65, sessionChallenge, sizeof(sessionChallenge));
  bleSession->setValue(hello, sizeof(hello));
  sessionReady = true;
  bleStatus->setValue("ready");
}

class ProvisionCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    const String chunk = characteristic->getValue();
    characteristic->setValue(""); // Do not retain credentials in the GATT value.
    if (provisioningBusy.load()) return;
    if (chunk == "\x01") {
      bleInput = "";
      bleStatus->setValue("receiving");
      return;
    }
    if (bleInput.length() + chunk.length() > 1024) {
      bleInput = "";
      bleStatus->setValue("invalid_config");
      return;
    }
    bleInput += chunk;
    if (!bleInput.endsWith("\n")) return;
    JsonDocument envelope;
    const auto error = deserializeJson(envelope, bleInput);
    bleInput = "";
    if (error || !envelope["op"].is<String>() || !sessionReady) {
      bleStatus->setValue("invalid_config");
      return;
    }
    if (envelope["op"] == "auth") {
      sessionAuthenticated = false;
      if (pinLockedUntil && static_cast<int32_t>(millis() - pinLockedUntil) < 0) {
        bleStatus->setValue("pin_locked");
        return;
      }
      uint8_t publicBytes[65], suppliedProof[32], expectedProof[32], keyMaterial[54] = {};
      mbedtls_ecp_point peer;
      mbedtls_mpi shared;
      mbedtls_ecp_point_init(&peer);
      mbedtls_mpi_init(&shared);
      bool valid = readHex(envelope["public"].as<String>(), publicBytes, sizeof(publicBytes)) &&
          readHex(envelope["proof"].as<String>(), suppliedProof, sizeof(suppliedProof)) &&
          mbedtls_ecp_point_read_binary(&sessionGroup, &peer, publicBytes, sizeof(publicBytes)) == 0 &&
          mbedtls_ecp_check_pubkey(&sessionGroup, &peer) == 0 &&
          mbedtls_ecdh_compute_shared(&sessionGroup, &shared, &peer, &sessionPrivate, cryptoRandom, nullptr) == 0 &&
          mbedtls_mpi_write_binary(&shared, keyMaterial, 32) == 0;
      mbedtls_ecp_point_free(&peer);
      mbedtls_mpi_free(&shared);
      memcpy(keyMaterial + 32, sessionChallenge, 16);
      char pin[7];
      snprintf(pin, sizeof(pin), "%06lu", (unsigned long)HARDWARE_PIN);
      memcpy(keyMaterial + 48, pin, 6);
      valid = valid && mbedtls_md(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                                  keyMaterial, sizeof(keyMaterial), sessionKey) == 0 &&
                      sessionProof("sentrix-client-v1", expectedProof);
      uint8_t mismatch = 0;
      if (valid) for (size_t i = 0; i < 32; ++i) mismatch |= suppliedProof[i] ^ expectedProof[i];
      mbedtls_platform_zeroize(keyMaterial, sizeof(keyMaterial));
      if (!valid || mismatch) {
        mbedtls_platform_zeroize(sessionKey, sizeof(sessionKey));
        if (++pinFailures >= 5) { pinFailures = 0; pinLockedUntil = millis() + 60000; }
        bleStatus->setValue("invalid_pin");
        return;
      }
      if (!sessionProof("sentrix-device-v1", expectedProof)) {
        bleStatus->setValue("crypto_failed");
        return;
      }
      bleSession->setValue(expectedProof, sizeof(expectedProof));
      sessionAuthenticated = true;
      pinFailures = 0;
      bleStatus->setValue("authenticated");
      return;
    }
    if (envelope["op"] == "scan_wifi" && sessionAuthenticated) {
      provisioningBusy.store(true);
      bleStatus->setValue("scanning_wifi");
      wifiScanPending.store(true);
      return;
    }
    if (envelope["op"] != "config" || !sessionAuthenticated) {
      bleStatus->setValue("auth_required");
      return;
    }
    sessionAuthenticated = false;
    const String data = envelope["data"].as<String>();
    uint8_t iv[12], encrypted[400], plaintext[385];
    const size_t length = data.length() / 2;
    if (length < 16 || length > sizeof(encrypted) ||
        !readHex(envelope["iv"].as<String>(), iv, sizeof(iv)) ||
        !readHex(data, encrypted, length)) {
      bleStatus->setValue("invalid_config");
      return;
    }
    mbedtls_gcm_context aes;
    mbedtls_gcm_init(&aes);
    int rc = mbedtls_gcm_setkey(&aes, MBEDTLS_CIPHER_ID_AES, sessionKey, 256);
    if (!rc) rc = mbedtls_gcm_auth_decrypt(&aes, length - 16, iv, sizeof(iv),
        sessionChallenge, sizeof(sessionChallenge), encrypted + length - 16, 16, encrypted, plaintext);
    mbedtls_gcm_free(&aes);
    mbedtls_platform_zeroize(sessionKey, sizeof(sessionKey));
    sessionReady = false; // One configuration per handshake; no replay after commit.
    if (rc) {
      bleStatus->setValue("invalid_config");
      return;
    }
    JsonDocument doc;
    const auto configError = deserializeJson(doc, static_cast<const uint8_t *>(plaintext), length - 16);
    mbedtls_platform_zeroize(plaintext, sizeof(plaintext));
    if (configError || !doc["ssid"].is<String>() || !doc["password"].is<String>() ||
        !doc["group_id"].is<String>()) {
      bleStatus->setValue("invalid_config");
      return;
    }
    const String ssid = doc["ssid"], password = doc["password"], group = doc["group_id"];
    if (!ssid.length() || ssid.length() > 32 || password.length() > 63 ||
        (password.length() && password.length() < 8) || !group.length() || group.length() > 64 ||
        strlen(ssid.c_str()) != ssid.length() || strlen(password.c_str()) != password.length() ||
        strlen(group.c_str()) != group.length()) {
      bleStatus->setValue("invalid_config");
      return;
    }
    ProvisionRequest request{};
    ssid.toCharArray(request.ssid, sizeof(request.ssid));
    password.toCharArray(request.password, sizeof(request.password));
    group.toCharArray(request.group, sizeof(request.group));
    provisioningBusy.store(true);
    bleStatus->setValue("connecting");
    if (xQueueSend(provisioningQueue, &request, 0) != pdTRUE) {
      provisioningBusy.store(false);
      bleStatus->setValue("busy");
    }
  }
};

class SentrixServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bluetoothConnected.store(true); beginSession(); }
  void onDisconnect(BLEServer *) override {
    bluetoothConnected.store(false);
    bleInput = "";
    sessionReady = sessionAuthenticated = false;
    mbedtls_platform_zeroize(sessionKey, sizeof(sessionKey));
    BLEDevice::startAdvertising();
  }
};

void startBluetooth() {
  provisioningQueue = xQueueCreate(1, sizeof(ProvisionRequest));
  if (!provisioningQueue) {
    while (true) delay(1000);
  }
  String name = "Sentrix-" + deviceId.substring(12);
  name.replace(":", "");
  BLEDevice::init(name);
  // PIN authentication and credential encryption happen above GATT, without OS bonding.
  BLESecurity::setForceAuthentication(false);
  BLESecurity::setAuthenticationMode(false, false, false);
  mbedtls_ecp_group_init(&sessionGroup);
  mbedtls_mpi_init(&sessionPrivate);
  mbedtls_ecp_point_init(&sessionPublic);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new SentrixServerCallbacks());
  BLEService *service = server->createService(SERVICE_UUID);
  auto *identity = service->createCharacteristic(IDENTITY_UUID, BLECharacteristic::PROPERTY_READ);
  identity->setValue(deviceId);
  auto *config = service->createCharacteristic(CONFIG_UUID, BLECharacteristic::PROPERTY_WRITE);
  config->setAccessPermissions(ESP_GATT_PERM_WRITE);
  config->setCallbacks(new ProvisionCallbacks());
  bleStatus = service->createCharacteristic(STATUS_UUID, BLECharacteristic::PROPERTY_READ);
  bleStatus->setValue("ready");
  bleSession = service->createCharacteristic(SESSION_UUID, BLECharacteristic::PROPERTY_READ);
  bleNetworks = service->createCharacteristic("12345678-1234-5678-1234-56789abcdef5", BLECharacteristic::PROPERTY_READ);
  bleNetworks->setValue("[]");
  service->start();
  auto *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();
}

void loadProvisioning() {
  Preferences prefs;
  if (!prefs.begin("sentrix", true)) return;
  String saved = prefs.getString("network", "");
  prefs.end();
  JsonDocument doc;
  if (deserializeJson(doc, saved) || !doc["ssid"].is<String>() ||
      !doc["password"].is<String>() || !doc["group_id"].is<String>()) return;
  wifiSsid = doc["ssid"].as<String>();
  wifiPassword = doc["password"].as<String>();
  groupId = doc["group_id"].as<String>();
}

void serviceWifiScan() {
  if (!wifiScanPending.exchange(false)) return;
  updateLeds();
  // Run in the main task so BLE callbacks remain responsive during the scan.
  const bool autoReconnect = WiFi.getAutoReconnect();
  WiFi.setAutoReconnect(false);
  // ESP-IDF rejects scans while connecting to a saved AP. Preserve a healthy link.
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(false, false);
    delay(100);
  }
  esp_wifi_scan_stop();
  WiFi.scanDelete();
  WiFi.setScanTimeout(12000);
  const int count = WiFi.scanNetworks(false, false);
  JsonDocument doc;
  JsonArray networks = doc.to<JsonArray>();
  for (int i = 0; i < count && networks.size() < 8; ++i) {
    const String ssid = WiFi.SSID(i);
    if (!ssid.length()) continue;
    bool duplicate = false;
    for (JsonVariant network : networks) if (network.as<String>() == ssid) duplicate = true;
    if (duplicate) continue;
    networks.add(ssid);
    // Keep the result within one GATT attribute (512 bytes, including JSON escapes).
    if (measureJson(doc) > 500) { networks.remove(networks.size() - 1); break; }
  }
  String result;
  serializeJson(doc, result);
  bleNetworks->setValue(result);
  if (count < 0) esp_wifi_scan_stop();
  WiFi.scanDelete();
  WiFi.setAutoReconnect(autoReconnect);
  nextWiFi = millis() + 15000;
  bleStatus->setValue(count < 0 ? "wifi_scan_failed" : "wifi_scan_done");
  provisioningBusy.store(false);
}

void serviceProvisioning() {
  ProvisionRequest request{};
  if (xQueueReceive(provisioningQueue, &request, 0) != pdTRUE) return;
  updateLeds();
  if (mqtt.connected()) mqtt.disconnect();
  WiFi.disconnect();
  WiFi.begin(request.ssid, request.password);
  const uint32_t started = millis();
  // Provisioning pauses acquisition so Wi-Fi/BLE work cannot contaminate a sample window.
  while (WiFi.status() != WL_CONNECTED && millis() - started < 25000) {
    delay(20);
  }
  const char *result = "wifi_failed";
  if (WiFi.status() == WL_CONNECTED) {
    JsonDocument doc;
    doc["ssid"] = request.ssid;
    doc["password"] = request.password;
    doc["group_id"] = request.group;
    String saved;
    serializeJson(doc, saved);
    Preferences prefs;
    bool stored = false;
    if (prefs.begin("sentrix", false)) {
      stored = prefs.putString("network", saved) == saved.length();
      prefs.end();
    }
    if (stored) {
      wifiSsid = request.ssid;
      wifiPassword = request.password;
      groupId = request.group;
      result = "connected";
    } else result = "storage_failed";
  }
  if (strcmp(result, "connected") != 0) {
    WiFi.disconnect();
    if (wifiSsid.length()) WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
  }
  wifiWasConnected = false;
  nextWiFi = millis() + 15000;
  nextMqtt = 0;
  bleStatus->setValue(result);
  provisioningBusy.store(false);
}

bool due(uint32_t now, uint32_t target) {
  return static_cast<int32_t>(now - target) >= 0;
}

bool publishJson(const char *topic, const JsonDocument &doc, bool retained = false) {
  char buffer[1536];
  if (measureJson(doc) >= sizeof(buffer)) return false;
  const size_t length = serializeJson(doc, buffer, sizeof(buffer));
  return mqtt.publish(topic, reinterpret_cast<const uint8_t *>(buffer), length, retained);
}

void serviceNetwork() {
  const uint32_t now = millis();
  if (!wifiSsid.length()) return;
  if (WiFi.status() != WL_CONNECTED) {
    wifiWasConnected = false;
    if (mqtt.connected()) mqtt.disconnect();
    if (due(now, nextWiFi)) {
      nextWiFi = now + 15000;
      WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
    }
    return;
  }
  if (!wifiWasConnected) {
    wifiWasConnected = true;
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  }
  if (!MQTT_HOST[0]) return;
  if (!mqtt.connected() && due(now, nextMqtt)) {
    nextMqtt = now + 5000;
    String clientId = "sentrix-" + deviceId;
    // Retained LWT allows the gateway to notice an unexpected disconnect.
    const bool connected = mqtt.connect(clientId.c_str(),
        MQTT_USER[0] ? MQTT_USER : nullptr,
        MQTT_USER[0] ? MQTT_PASSWORD : nullptr,
        statusTopic.c_str(), 1, true, "offline");
    if (connected) {
      mqtt.publish(statusTopic.c_str(), "online", true);
      JsonDocument hello;
      hello["device_id"] = deviceId;
      hello["boot_id"] = bootId;
      hello["group_id"] = groupId;
      hello["firmware"] = "sentrix-arduino-2.3.0";
      publishJson("sentrix/handshake", hello);
    }
  }
  mqtt.loop();
}

Measurement readCurrent() {
  Measurement result;
  RmsAccumulator acc;
  const uint32_t start = micros();
  uint32_t nextSample = start;
  uint32_t missed = 0;
  // Network/Serial operations are deliberately outside the acquisition window.
  // This is periodic window sampling, not uninterrupted waveform recording.
  while (static_cast<uint32_t>(micros() - start) < SAMPLE_WINDOW_US) {
    const uint32_t now = micros();
    if (!due(now, nextSample)) {
      delayMicroseconds(20);
      continue;
    }
    const uint32_t skipped = (now - nextSample) / SAMPLE_INTERVAL_US;
    missed += skipped;
    nextSample += (skipped + 1) * SAMPLE_INTERVAL_US;
    acc.add(analogReadMilliVolts(CURRENT_PIN));
  }
  const uint32_t elapsed = micros() - start;
  result.current = acc.rmsVolts() * CT_RATIO / BURDEN_OHMS * CURRENT_CALIBRATION;
  result.bias = acc.mean;
  result.minimum = acc.minimum;
  result.maximum = acc.maximum;
  result.samples = acc.count;
  result.sampleRate = elapsed ? acc.count * 1000000.0 / elapsed : 0;
  result.missedSlots = missed;
  // Conservative guards inside the classic ESP32 11 dB calibrated range.
  result.valid = acc.count >= 350 && missed <= 20 &&
      acc.minimum > 200 && acc.maximum < 3000 &&
      acc.mean > 1200 && acc.mean < 2100;
  return result;
}

void serviceIR() {
  if (!irReceiver.decode(&irResults)) return;
  if (!irResults.overflow && !irResults.repeat) {
    lastIr = millis();
    seenIr = true;
    stdAc::state_t state{};
    const String description = IRAcUtils::resultAcToString(&irResults);
    // Some partial commands yield default state values: require an explicit setpoint.
    const bool hasTemperature = description.startsWith("Temp: ") ||
                                description.indexOf(", Temp: ") >= 0;
    bool temperatureDecoded = false;
    if (hasTemperature && IRAcUtils::decodeToState(&irResults, &state) &&
        state.command == stdAc::ac_command_t::kControlCommand) {
      const float degrees = setpointCelsius(state.degrees, state.celsius);
      if (isfinite(degrees)) {
        remoteTemperatureC = degrees;
        temperatureDecoded = true;
      }
    }
    JsonDocument event;
    event["device_id"] = deviceId;
    event["boot_id"] = bootId;
    event["uptime_ms"] = lastIr;
    event["protocol"] = typeToString(irResults.decode_type);
    if (!hasACState(irResults.decode_type)) {
      event["address"] = irResults.address;
      event["command"] = irResults.command;
    } else {
      event["address"] = nullptr;
      event["command"] = nullptr;
    }
    event["bits"] = irResults.bits;
    event["overflow"] = false;
    event["temperature_decoded"] = temperatureDecoded;
    if (temperatureDecoded) event["temperature_setpoint_c"] = remoteTemperatureC;
    else event["temperature_setpoint_c"] = nullptr;
    // Keep decoded AC details available for unsupported-model diagnosis via MQTT.
    event["ac_state"] = description;
    if (mqtt.connected()) publishJson(irTopic.c_str(), event);
  }
  // Saved-buffer decoding already resumes capture; do not discard the next frame.
}

void reportMeasurement(const Measurement &value) {
  sensorFault = !value.valid;
  if (value.valid) {
    if (value.current >= ACTIVE_CURRENT_A) active = true;
    else if (value.current <= STANDBY_CURRENT_A) active = false;
  }
  highCurrent = value.valid && value.current >= HIGH_CURRENT_A;
  JsonDocument doc;
  doc["schema_version"] = 2;
  doc["group_id"] = groupId;
  doc["device_id"] = deviceId;
  doc["boot_id"] = bootId;
  doc["sequence"] = ++sequence;
  doc["uptime_ms"] = millis();
  const time_t now = time(nullptr);
  if (now > 1700000000) doc["timestamp_unix"] = static_cast<int64_t>(now);
  else doc["timestamp_unix"] = nullptr;
  doc["valid"] = value.valid;
  doc["state"] = !value.valid ? "sensor_fault" : active ? "active" : "standby";
  if (value.valid) {
    doc["current_rms_a"] = value.current;
    doc["apparent_power_va"] = value.current * NOMINAL_VOLTAGE;
  } else {
    doc["current_rms_a"] = nullptr;
    doc["apparent_power_va"] = nullptr;
  }
  doc["nominal_voltage_v"] = NOMINAL_VOLTAGE;
  doc["high_current_warning"] = highCurrent;
  doc["adc_bias_mv"] = value.bias;
  doc["adc_min_mv"] = value.minimum;
  doc["adc_max_mv"] = value.maximum;
  doc["sample_count"] = value.samples;
  doc["sample_rate_hz"] = value.sampleRate;
  doc["missed_sample_slots"] = value.missedSlots;
  doc["first_minute"] = millis() < 60000;
  if (seenIr) doc["last_ir_age_ms"] = millis() - lastIr;
  else doc["last_ir_age_ms"] = nullptr;
  if (isfinite(remoteTemperatureC)) doc["temperature_setpoint_c"] = remoteTemperatureC;
  else doc["temperature_setpoint_c"] = nullptr;
  Serial.printf("Current_rms:%.4f rs_sinyal:%.1f\n",
                value.valid ? value.current : NAN, remoteTemperatureC);
  if (mqtt.connected()) publishJson(telemetryTopic.c_str(), doc);
}

void updateLeds() {
  if (RGB_LED_ENABLED) {
    rgbPattern.store(rgbState(sensorFault || highCurrent, provisioningBusy.load(),
        bluetoothConnected.load(), wifiSsid.length() > 0, WiFi.status() == WL_CONNECTED,
        MQTT_HOST[0] && mqtt.connected()));
  }
  // Common cathode: HIGH turns on each LED.
  const bool networkConfigured = wifiSsid.length() || MQTT_HOST[0];
  const bool networkOK = (!wifiSsid.length() || WiFi.status() == WL_CONNECTED) &&
                         (!MQTT_HOST[0] || mqtt.connected());
  const bool warning = sensorFault || highCurrent;
  digitalWrite(RED_LED_PIN, warning ||
      (networkConfigured && !networkOK && (millis() / 500) % 2));
  digitalWrite(GREEN_LED_PIN, !warning && networkOK);
}

void setup() {
  Serial.begin(115200);
  if (RGB_LED_ENABLED) {
    pinMode(RGB_RED_PIN, OUTPUT);
    pinMode(RGB_GREEN_PIN, OUTPUT);
    pinMode(RGB_BLUE_PIN, OUTPUT);
    renderRgb();
    // Timer-driven blinking continues while the main task scans or connects Wi-Fi.
    rgbTicker.attach_ms(50, renderRgb);
  }
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(CURRENT_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(CURRENT_PIN, ADC_11db);
  // GPIO35 has no internal pull-up; the powered TSOP drives this input.
  irReceiver.enableIRIn(false);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  // Read the factory-derived station MAC without relying on Wi-Fi startup timing.
  uint8_t mac[6] = {};
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK ||
      !(mac[0] | mac[1] | mac[2] | mac[3] | mac[4] | mac[5])) {
    while (true) { digitalWrite(RED_LED_PIN, HIGH); delay(1000); }
  }
  char macText[18];
  snprintf(macText, sizeof(macText), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  deviceId = macText;
  snprintf(bootId, sizeof(bootId), "%08lx", static_cast<unsigned long>(esp_random()));
  telemetryTopic = "sentrix/telemetry/" + deviceId;
  statusTopic = "sentrix/status/" + deviceId;
  irTopic = "sentrix/ir/" + deviceId;
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setKeepAlive(20);
  mqtt.setSocketTimeout(2);
  if (!mqtt.setBufferSize(1536)) {
    while (true) { digitalWrite(RED_LED_PIN, HIGH); delay(1000); }
  }
  loadProvisioning();
  startBluetooth();
}

void loop() {
  serviceWifiScan();
  serviceProvisioning();
  serviceIR();
  serviceNetwork();
  if (Serial.available()) {
    const char key = Serial.read();
    if (key == 'r') nextReport = millis();
  }
  if (due(millis(), nextReport)) {
    const uint32_t started = millis();
    const Measurement value = readCurrent();
    serviceIR();
    reportMeasurement(value);
    nextReport = started + REPORT_INTERVAL_MS;
    // No burst of catch-up measurements after a long network delay.
    if (due(millis(), nextReport)) nextReport = millis() + REPORT_INTERVAL_MS;
  }
  updateLeds();
  delay(1);
}
