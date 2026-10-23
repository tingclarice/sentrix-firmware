# sentrix-firmware
Hardware Code Repository for Sentrix, an IoT-based AC energy monitoring and predictive maintenance platform.


## Optional four-pin RGB status LED

The maintained sketch is `SentrixESP32.ino` in this folder. RGB support is enabled with `RGB_LED_ENABLED = true`; set it to false to disable the RGB outputs. Existing GPIO4 red and GPIO2 green indicators are unchanged.

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

Compile from `SentrixArduino/Compile.ps1`; it stages this exact source in a temporary folder matching Arduino naming rules. In Arduino IDE, opening this file may ask to place it in a matching `SentrixESP32` folder: the repository source here remains the maintained copy. Use Huge APP (3MB No OTA).
