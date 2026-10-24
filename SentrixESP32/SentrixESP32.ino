/* Sentrix sensor node: classic ESP32 DevKit V1 / ESP32-WROOM.
 * Build dependencies are pinned in platformio.ini; see README.md for setup.
 * Single-file firmware: all settings and logic are in this sketch.
 * Reads the physical CT circuit on GPIO34. See README.md for wiring.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
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
// ===== NETWORK & BROKER CONFIGURATION =====
constexpr char WIFI_SSID[] = "";
constexpr char WIFI_PASSWORD[] = "";
// Empty host disables MQTT until configured. Use the same broker as sentrix-api.
constexpr char MQTT_BROKER_HOST[] = "";
constexpr uint16_t MQTT_BROKER_PORT = 8883;
constexpr char MQTT_CLIENT_ID[] = "sentrix-root-node"; // Station MAC is appended.
constexpr char MQTT_USERNAME[] = "";
constexpr char MQTT_PASSWORD[] = "";

// ===== HARDWARE IDENTITY =====
// Must match the factory-seeded devices.pin_code; also authenticates BLE setup.
constexpr uint32_t DEVICE_PIN = 715204;
constexpr char MESH_ID[] = ""; // groups.mesh_id returned by Go, not groups.id (UUID).
constexpr char FIRMWARE_VERSION[] = "sentrix-arduino-3.0.0";
static_assert(DEVICE_PIN <= 999999, "DEVICE_PIN must fit six digits");
static_assert(MQTT_BROKER_PORT == 8883, "MQTT must use TLS port 8883");

// ISRG Root X1: https://letsencrypt.org/certs/isrgrootx1.pem
// Replace with your broker's issuing root CA if its chain uses a different root.
constexpr char MQTT_ROOT_CA[] = R"PEM(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)PEM";

constexpr char SERVICE_UUID[] = "12345678-1234-5678-1234-56789abcdef0";
constexpr char IDENTITY_UUID[] = "12345678-1234-5678-1234-56789abcdef1";
constexpr char CONFIG_UUID[] = "12345678-1234-5678-1234-56789abcdef2";
constexpr char STATUS_UUID[] = "12345678-1234-5678-1234-56789abcdef3";
constexpr char SESSION_UUID[] = "12345678-1234-5678-1234-56789abcdef4";
constexpr uint8_t CURRENT_PIN = 34;
constexpr uint8_t IR_PIN = 35;
constexpr uint8_t RED_LED_PIN = 4;
constexpr uint8_t GREEN_LED_PIN = 2;
constexpr bool STATUS_LED_ACTIVE_HIGH = true; // Set false for an active-low board LED.
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

// GPIO2 states: 0=fatal, 1=unconfigured, 2=setup, 3=Wi-Fi, 4=NTP,
// 5=MQTT, 6=connected, 7=measurement warning. Only this timer writes GPIO2.
std::atomic<uint8_t> statusPattern{1};
constexpr bool statusLedOn(uint8_t state, uint32_t elapsed) {
  return state == 0 ? true
       : state == 1 ? elapsed % 1000 < 500
       : state == 2 ? elapsed % 300 < 150
       : state == 6 ? elapsed % 3000 < 100
       : state == 7 ? elapsed % 100 < 50
       : elapsed % 3000 < (state - 1) * 300U && elapsed % 300 < 100;
}
static_assert(statusLedOn(3, 300) && !statusLedOn(3, 600), "Wi-Fi: two flashes");
static_assert(statusLedOn(4, 600) && !statusLedOn(4, 900), "NTP: three flashes");
static_assert(statusLedOn(5, 900) && !statusLedOn(5, 1200), "MQTT: four flashes");
static_assert(statusLedOn(6, 0) && !statusLedOn(6, 100), "Connected heartbeat");

void renderStatus() {
  static uint8_t previous = 255;
  static uint32_t started = 0;
  const uint32_t now = millis();
  const uint8_t state = statusPattern.load();
  if (state != previous) { previous = state; started = now; }
  digitalWrite(GREEN_LED_PIN, statusLedOn(state, now - started) == STATUS_LED_ACTIVE_HIGH);
  if (RGB_LED_ENABLED) renderRgb();
}

void fatalError(const char *reason) {
  Serial.println(reason);
  statusPattern.store(0);
  rgbPattern.store(17);
  while (true) delay(1000);
}

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
constexpr uint32_t BATCH_INTERVAL_MS = 60000;
constexpr uint32_t MAX_READING_AGE_MS = 2500;
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

WiFiClientSecure transport;
PubSubClient mqtt(transport);
String deviceId, handshakeTopic;
uint32_t nextReport = 0, nextWiFi = 0, nextMqtt = 0, lastBatch = 0;
uint32_t measuredAt = 0;
Measurement latestMeasurement;
uint32_t lastIr = 0;
bool seenIr = false, active = false, sensorFault = false, highCurrent = false;
bool wifiWasConnected = false;
bool handshakePublished = false;
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
String wifiSsid = WIFI_SSID, wifiPassword = WIFI_PASSWORD, meshId = MESH_ID;
BLECharacteristic *bleStatus = nullptr;
QueueHandle_t provisioningQueue;
std::atomic<bool> provisioningBusy{false};
struct ProvisionRequest {
  char ssid[33];
  char password[65];
  char mesh[65];
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

bool validMeshId(const String &value) {
  if (!value.length() || value.length() > 64) return false;
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
          (c >= '0' && c <= '9') || c == '_' || c == '-')) return false;
  }
  return true;
}

String provisionedMesh(const JsonDocument &doc) {
  // Keep older BLE clients/NVS records readable; group_id must contain mesh_id.
  if (!doc["mesh_id"].isNull() && !doc["mesh_id"].is<String>()) return "";
  const String value = doc["mesh_id"].is<String>() ? doc["mesh_id"].as<String>()
                                                   : doc["group_id"].as<String>();
  if (!doc["group_id"].isNull() &&
      (!doc["group_id"].is<String>() || doc["group_id"].as<String>() != value)) return "";
  return validMeshId(value) ? value : String();
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
      snprintf(pin, sizeof(pin), "%06lu", (unsigned long)DEVICE_PIN);
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
    if (configError || !doc["ssid"].is<String>() || !doc["password"].is<String>()) {
      bleStatus->setValue("invalid_config");
      return;
    }
    const String ssid = doc["ssid"], password = doc["password"], mesh = provisionedMesh(doc);
    if (!ssid.length() || ssid.length() > 32 || password.length() > 63 ||
        (password.length() && password.length() < 8) || !mesh.length() ||
        strlen(ssid.c_str()) != ssid.length() || strlen(password.c_str()) != password.length() ||
        strlen(mesh.c_str()) != mesh.length()) {
      bleStatus->setValue("invalid_config");
      return;
    }
    ProvisionRequest request{};
    ssid.toCharArray(request.ssid, sizeof(request.ssid));
    password.toCharArray(request.password, sizeof(request.password));
    mesh.toCharArray(request.mesh, sizeof(request.mesh));
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
  if (!provisioningQueue) fatalError("Provisioning queue allocation failed");
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
      !doc["password"].is<String>()) return;
  const String mesh = provisionedMesh(doc);
  if (!mesh.length()) return;
  wifiSsid = doc["ssid"].as<String>();
  wifiPassword = doc["password"].as<String>();
  meshId = mesh;
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
  handshakePublished = false;
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
    doc["mesh_id"] = request.mesh;
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
      meshId = request.mesh;
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
    handshakePublished = false;
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
  // A real clock is required for both TLS certificate dates and Go time series.
  if (time(nullptr) <= 1700000000 || !MQTT_BROKER_HOST[0] || !validMeshId(meshId)) return;
  if (!mqtt.connected() && due(now, nextMqtt)) {
    nextMqtt = now + 5000;
    handshakePublished = false;
    updateLeds(); // Timer keeps the MQTT pattern running throughout TLS negotiation.
    String clientId = String(MQTT_CLIENT_ID) + "-" + deviceId;
    const bool connected = mqtt.connect(clientId.c_str(),
        MQTT_USERNAME[0] ? MQTT_USERNAME : nullptr,
        MQTT_USERNAME[0] ? MQTT_PASSWORD : nullptr);
    if (!connected) {
      Serial.printf("MQTT connect failed: %d\n", mqtt.state());
      return;
    }
  }
  if (mqtt.connected() && !handshakePublished) {
    JsonDocument hello;
    char pin[7];
    snprintf(pin, sizeof(pin), "%06lu", (unsigned long)DEVICE_PIN);
    hello["mac_address"] = deviceId;
    hello["device_pin"] = pin;
    hello["mesh_id"] = meshId;
    hello["role"] = "root";
    hello["firmware_version"] = FIRMWARE_VERSION;
    hello["local_ip"] = WiFi.localIP().toString();
    hello["wifi_rssi"] = WiFi.RSSI();
    hello["timestamp"] = static_cast<int64_t>(time(nullptr));
    handshakePublished = publishJson(handshakeTopic.c_str(), hello);
    if (!handshakePublished) {
      Serial.println("Handshake publish failed; reconnecting");
      mqtt.disconnect();
      nextMqtt = millis() + 5000;
    } else Serial.println("Handshake sent to broker; API acceptance is not acknowledged");
  }
  mqtt.loop();
}

void serviceBatch() {
  const uint32_t now = millis();
  if (now - lastBatch < BATCH_INTERVAL_MS || !mqtt.connected() ||
      !handshakePublished || !validMeshId(meshId) || time(nullptr) <= 1700000000) return;
  lastBatch = now;
  if (!latestMeasurement.valid || !isfinite(latestMeasurement.current) ||
      latestMeasurement.current < 0 || now - measuredAt > MAX_READING_AGE_MS) {
    Serial.println("Telemetry skipped: no fresh valid current measurement");
    return;
  }
  JsonDocument batch;
  batch["mesh_id"] = meshId;
  batch["timestamp"] = static_cast<int64_t>(time(nullptr));
  batch["root_mac"] = deviceId;
  JsonObject reading = batch["readings"].to<JsonArray>().add<JsonObject>();
  reading["mac"] = deviceId; // Go expects mac here, not mac_address.
  reading["node_type"] = "root";
  reading["rms_current"] = latestMeasurement.current;
  // IR gives a target setpoint, not a measured ambient/coil temperature.
  if (isfinite(remoteTemperatureC)) reading["target_temperature"] = remoteTemperatureC;
  batch["offline_nodes"].to<JsonArray>(); // Standalone root has no children.
  const String topic = "sentrix/dt/" + meshId + "/batch";
  if (!publishJson(topic.c_str(), batch)) {
    Serial.println("Telemetry publish failed; reconnecting");
    handshakePublished = false;
    mqtt.disconnect();
    nextMqtt = now + 5000;
  }
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
    // Go has no IR event subscriber. Preserve diagnosis locally and send the
    // latest decoded setpoint as target_temperature in the next minute's batch.
    Serial.printf("IR protocol:%s temperature_decoded:%d state:%s\n",
                  typeToString(irResults.decode_type).c_str(), temperatureDecoded,
                  description.c_str());
  }
  // Saved-buffer decoding already resumes capture; do not discard the next frame.
}

void reportMeasurement(const Measurement &value) {
  sensorFault = !value.valid;
  latestMeasurement = value;
  measuredAt = millis();
  if (value.valid) {
    if (value.current >= ACTIVE_CURRENT_A) active = true;
    else if (value.current <= STANDBY_CURRENT_A) active = false;
  }
  highCurrent = value.valid && value.current >= HIGH_CURRENT_A;
  Serial.printf("Current_rms:%.4f rs_sinyal:%.1f\n",
                value.valid ? value.current : NAN, remoteTemperatureC);
}

void updateLeds() {
  if (RGB_LED_ENABLED) {
    rgbPattern.store(rgbState(sensorFault || highCurrent, provisioningBusy.load(),
        bluetoothConnected.load(), wifiSsid.length() > 0, WiFi.status() == WL_CONNECTED,
        MQTT_BROKER_HOST[0] && mqtt.connected() && handshakePublished));
  }
  // Common cathode: HIGH turns on each LED.
  const bool networkConfigured = wifiSsid.length() || MQTT_BROKER_HOST[0];
  const bool networkOK = (!wifiSsid.length() || WiFi.status() == WL_CONNECTED) &&
                         (!MQTT_BROKER_HOST[0] || (mqtt.connected() && handshakePublished));
  const bool warning = sensorFault || highCurrent;
  digitalWrite(RED_LED_PIN, warning ||
      (networkConfigured && !networkOK && (millis() / 500) % 2));
  statusPattern.store(warning ? 7
      : provisioningBusy.load() || bluetoothConnected.load() ? 2
      : !wifiSsid.length() || !validMeshId(meshId) ? 1
      : WiFi.status() != WL_CONNECTED ? 3
      : time(nullptr) <= 1700000000 ? 4
      : !MQTT_BROKER_HOST[0] || !mqtt.connected() || !handshakePublished ? 5 : 6);
}

void setup() {
  Serial.begin(115200);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  if (RGB_LED_ENABLED) {
    pinMode(RGB_RED_PIN, OUTPUT);
    pinMode(RGB_GREEN_PIN, OUTPUT);
    pinMode(RGB_BLUE_PIN, OUTPUT);
    renderRgb();
  }
  // Independent of RGB: GPIO2 keeps blinking during sampling, Wi-Fi scans and TLS.
  renderStatus();
  rgbTicker.attach_ms(25, renderStatus);
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
    fatalError("Cannot read station MAC");
  }
  char macText[18];
  snprintf(macText, sizeof(macText), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  deviceId = macText;
  handshakeTopic = "sentrix/auth/" + deviceId + "/connect";
  transport.setCACert(MQTT_ROOT_CA);
  transport.setHandshakeTimeout(5);
  mqtt.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
  mqtt.setKeepAlive(20);
  mqtt.setSocketTimeout(2);
  if (!mqtt.setBufferSize(1536)) {
    fatalError("MQTT buffer allocation failed");
  }
  loadProvisioning();
  startBluetooth();
  updateLeds();
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
  serviceBatch();
  updateLeds();
  delay(1);
}
