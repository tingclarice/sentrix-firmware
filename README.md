# sentrix-firmware
Hardware Code Repository for Sentrix, an IoT-based AC energy monitoring and predictive maintenance platform.

## Configure and build

The maintained sketch is `SentrixESP32.ino`. It targets a classic ESP32 DevKit V1 / ESP32-WROOM, with the CT circuit on GPIO34 and IR receiver on GPIO35. Keep the existing biased SCT-013-000 / 22-ohm burden circuit; this calibration is not suitable for SCT-013 variants with an internal burden.

Set these constants at the top of the sketch before uploading:

| Constant | Value |
|---|---|
| `WIFI_SSID`, `WIFI_PASSWORD` | Wi-Fi credentials, or leave empty for BLE provisioning |
| `MQTT_BROKER_HOST` | Broker hostname only; use the same broker as `sentrix-api` |
| `MQTT_BROKER_PORT` | 8883; TLS is mandatory |
| `MQTT_USERNAME`, `MQTT_PASSWORD` | Device broker credentials |
| `MQTT_ROOT_CA` | Broker's issuing root CA, in PEM format |
| `DEVICE_PIN` | Factory-seeded six-digit database PIN; also used for BLE authentication |
| `MESH_ID` | Go's group `mesh_id`, or leave empty for BLE provisioning |
| `STATUS_LED_ACTIVE_HIGH` | `true` for the usual GPIO2 LED; set `false` for an active-low board |

The bundled CA is [ISRG Root X1](https://letsencrypt.org/certs/isrgrootx1.pem). Verify that your broker's certificate chain uses it; replace the PEM for other issuing roots. The sketch uses `WiFiClientSecure.setCACert()` and never disables certificate verification. An empty broker host disables MQTT; local sensor and BLE operation remain available. Allow NTP access to `pool.ntp.org` or `time.nist.gov`; MQTT connection attempts and publishing wait for a valid Unix clock.

Create the group through Go, then copy its returned `mesh_id` exactly (normally 12 hex characters). Do not use the group's UUID, dashboard label, or a made-up mesh ID. The device MAC and PIN must already exist in `devices`; the handshake updates an existing row rather than registering a new device. Claim that device into the same group to store telemetry; Go skips readings from `UNCLAIMED` devices.

Authenticated BLE provisioning accepts the existing encrypted configuration envelope. Its decrypted JSON is now:

```json
{"ssid":"your-network","password":"your-password","mesh_id":"0123456789AB"}
```

Legacy `group_id` is accepted as an alias only when its value is the actual mesh ID. Previously saved Wi-Fi credentials load from the same NVS namespace; reprovision any device that saved a UUID instead of a mesh ID. New records use `mesh_id`. Conflicting aliases, empty IDs, IDs longer than 64 characters, and characters outside letters, digits, `_`, and `-` are rejected. The firmware cannot verify group membership locally.

PlatformIO dependencies and the 3 MB Huge APP partition are pinned in `platformio.ini` (Arduino ESP32 core 3.3.9, PubSubClient 2.8, IRremoteESP8266 2.9.0, ArduinoJson 7.4.3):

```powershell
pio run -e esp32dev
pio run -e esp32dev -t upload --upload-port COM5
pio device monitor -b 115200
```

Replace COM5 with your board's port. The upload command flashes the connected board; the build command does not. Arduino IDE users can place the sketch in a `SentrixESP32` folder, install those libraries, select the classic ESP32 board, and select Huge APP (3MB No OTA).

## MQTT contract with sentrix-api

On each broker connection, publish a non-retained handshake to `sentrix/auth/{station-MAC}/connect`:

```json
{
  "mac_address": "AA:BB:CC:DD:EE:FF",
  "device_pin": "715204",
  "mesh_id": "0123456789AB",
  "role": "root",
  "firmware_version": "sentrix-arduino-3.0.0",
  "local_ip": "192.168.1.45",
  "wifi_rssi": -62,
  "timestamp": 1792800000
}
```

The PIN is a string, including leading zeros. MQTT client IDs append the station MAC to `MQTT_CLIENT_ID` so multiple nodes do not disconnect one another.

Every 60 seconds, publish a non-retained batch to `sentrix/dt/{mesh_id}/batch`:

```json
{
  "mesh_id": "0123456789AB",
  "timestamp": 1792800060,
  "root_mac": "AA:BB:CC:DD:EE:FF",
  "readings": [
    {"mac":"AA:BB:CC:DD:EE:FF","node_type":"root","rms_current":1.25,"target_temperature":24.0}
  ],
  "offline_nodes": []
}
```

Go requires `readings[].mac` and `rms_current`. The older AGENTS.md example's `mac_address` and `online` fields do not match its actual batch parser. The handshake still uses `mac_address`.

This firmware is a standalone root: one reading and no children. It does not implement mesh transport. It measures a 200 ms AC waveform window approximately once per second, reports to Serial at that cadence, and sends the latest valid RMS current when the minute batch is generated. This is periodic window sampling, not continuous waveform recording or a whole-minute average. Serial `r` requests a local reading without sending an extra MQTT batch.

An invalid, negative, nonfinite, or stale (>2.5 seconds) reading skips that minute's batch; it is never replaced by zero. A real valid zero is sent. Missing IR setpoints are omitted; a decoded AC setpoint becomes `target_temperature`. There is no physical temperature sensor, so `current_temperature` is omitted. IR details remain available on Serial. The obsolete `sentrix/handshake`, `sentrix/telemetry/...`, `sentrix/status/...`, and `sentrix/ir/...` publications are removed because Go has no handlers for them.

PubSubClient publishes at QoS 0. A successful local publish is not an application acknowledgment or a guarantee of delivery. Measurements are not buffered across network outages; the next eligible minute uses a fresh reading. A failed local publish triggers MQTT reconnection and another handshake.

## Built-in GPIO2 status LED

No external LED is required. GPIO2 (also named `GREEN_LED_PIN`) has these patterns, independent of `RGB_LED_ENABLED`:

| Pattern | State |
|---|---|
| 500 ms on / 500 ms off | Missing Wi-Fi credentials or mesh ID; needs configuration |
| 150 ms on / 150 ms off | BLE connected, Wi-Fi scan, or provisioning in progress |
| 2 short flashes, pause | Wi-Fi disconnected/connecting/retrying |
| 3 short flashes, pause | Wi-Fi connected; waiting for a valid NTP clock |
| 4 short flashes, pause | Broker host missing, MQTT connecting/retrying, or handshake not sent |
| One 100 ms pulse every 3 seconds | Broker connected and handshake sent |
| 50 ms on / 50 ms off | Invalid sensor reading or high-current warning |
| Solid on | Fatal initialization failure; power-cycle and inspect Serial if persistent |

Counted flashes are 100 ms on / 200 ms off in a 3-second cycle. Priority: fatal error > sensor/high-current warning > BLE/setup > missing configuration > Wi-Fi > NTP > MQTT > connected. An unwired CT input can override connection indications with the sensor warning. The warning threshold defaults to `HIGH_CURRENT_A = 10.0`; it is an indication, not an electrical protection device. A timer drives GPIO2 throughout blocking sensor, scan, provisioning, and TLS operations. Any external LED already attached to GPIO2 follows the same blink pattern.

The connected pulse does not confirm that Go accepted the PIN, mesh membership, or telemetry. Check Go's logs and dashboard `last_seen` for that. The API currently sends no acknowledgment back to devices.

## Optional four-pin RGB status LED

RGB support is enabled with `RGB_LED_ENABLED = true`; set it to false to disable RGB outputs. GPIO4 keeps its existing red warning/network indication. GPIO2 uses the built-in status patterns above.

| LED connection | ESP32 connection |
|---|---|
| Red leg | GPIO25 through a 470-ohm resistor |
| Green leg | GPIO26 through a separate 470-ohm resistor |
| Blue leg | GPIO27 through a separate 470-ohm resistor |
| Common cathode | GND; leave `RGB_COMMON_ANODE = false` |
| Common anode (alternative type) | 3.3V; set `RGB_COMMON_ANODE = true` |

Use the LED datasheet to identify the legs; leg order varies. Connect the common leg according to your LED type, never both alternatives. Do not use 5V on these directly driven LED connections. Each color needs its own resistor. GPIO34/35 remain the CT/IR inputs.

Priority: sensor fault/high current > setup activity > Bluetooth session > normal network status.

| Color | State |
|---|---|
| Fast blinking red | Invalid sensor reading or high-current warning |
| Slow blinking yellow | Scanning Wi-Fi or provisioning Wi-Fi |
| Solid blue | Browser Bluetooth connection open |
| Slow blinking blue | No saved Wi-Fi; ready for setup |
| Slow blinking red | Configured Wi-Fi disconnected/retrying |
| Solid yellow | Wi-Fi connected; MQTT missing or disconnected |
| Solid green | Wi-Fi and configured MQTT broker connected |

Slow blink: 500 ms on/off. Fast blink: 150 ms on/off. Timer-driven blinking continues during blocking Wi-Fi operations. Green does not prove cloud ingestion. Sensor warnings override setup colors, including when the CT input is unwired. This indicator is not an electrical protection device.

## Backend review

See `docs/firmware-alignment-2026-10-24.md` for the approved scope, verification results, and API issues found. The API source is unchanged.
