# PHMS updated firmware

## Hardware
- ESP32-S3 N16R8
- ADS1293 ECG module
- MAX30105 pulse oximeter
- MSP3526 3.5-inch ST7796 480x320 display with FT6336U touch controller
- NEO-6M GPS (optional)
- Firebase Realtime Database (optional)

## ECG channel interpretation — important
With the four limb electrodes RA, LA, LL and RL wired to ADS1293 IN1, IN2, IN3 and IN4:
- ADS channel 1 measures Lead I = LA − RA
- ADS channel 2 measures Lead II = LL − RA
- Firmware calculates Lead III = Lead II − Lead I (Einthoven's law).

Therefore the display shows three limb-lead traces, but only two independent analog channels are acquired. Lead III is mathematically derived, not a third independent ADC acquisition. This is the correct standard limb-lead relationship.

## Pin map

### ST7796 display (from supplied displayTest.ino)
| Signal | ESP32-S3 |
|---|---:|
| TFT CS / RST / DC | 14 / 16 / 15 |
| TFT MOSI / SCK / MISO | 11 / 12 / 13 |
| TFT backlight | 3 |

### FT6336U touch
| Signal | ESP32-S3 |
|---|---:|
| SDA / SCL | 5 / 6 |
| RST / INT | 7 / 4 |
| I2C address | 0x38 |

Touch input is initialized but not used for controls in this firmware.

### MAX30105
- SDA = GPIO5, SCL = GPIO6, address = 0x57.
- Ensure the MAX30105 breakout is compatible with the same 3.3 V I2C bus.
- The display touch controller and MAX30105 share I2C.

### ADS1293 (from supplied medilink_ecg_v3.ino)
| Signal | ESP32-S3 |
|---|---:|
| SCLK / MOSI / MISO | 12 / 11 / 13 |
| CS | 10 |
| DRDY | 9 |
| ALARM | 8 |

The TFT and ADS1293 share SCLK/MOSI/MISO but have separate chip-select pins. Verify that both devices tri-state MISO when CS is inactive.

### ADS1293 electrode inputs
- RA -> IN1
- LA -> IN2
- LL -> IN3
- RL -> IN4 (right-leg drive)
- Lead I = LA − RA; Lead II = LL − RA; Lead III = Lead II − Lead I.

### GPS
- GPS TX -> ESP32 GPIO18 (RX)
- GPS RX -> ESP32 GPIO17 (TX)
- 9600 baud.

## Libraries
Install:
1. Arduino_GFX_Library
2. SparkFun MAX3010x Pulse and Proximity Sensor Library (MAX30105.h)
3. TinyGPSPlus
4. WiFiManager by tzapu
5. ESP32 Arduino core (WiFi, HTTPClient, WiFiClientSecure, SPI, Wire)

The Maxim `spo2_algorithm.h` is included with the SparkFun MAX3010x library package in the usual examples; if your installation does not provide it, add the matching Maxim SpO2 algorithm source/header files.

## Setup
1. Open the `PHMS_Updated` folder in Arduino IDE.
2. Select the ESP32-S3 board and correct USB/flash settings.
3. Replace `FIREBASE_API_KEY` and `FIREBASE_DATABASE_URL` in config.h with your own project values. Firebase is optional; use placeholders if not using it.
4. Compile and flash.
5. On first boot, connect to the `PHMS-Setup` WiFi access point and configure Wi-Fi.

## Display
The dashboard draws three stacked ECG traces and displays MAX30105 heart rate and SpO2. The trace scaling in `display_module.cpp` is a starting point and may need adjustment for your electrode gain/signal amplitude.

## Serial stream
At 921600 baud:
`D,sequence,timestamp_us,leadI_mV,leadII_mV,leadIII_mV,alarm,unused`
Lines beginning with `#` or `[... ]` are diagnostics.

## Removed
- MLX90614 temperature sensor
- Temperature firmware module and all temperature Firebase fields.

## Safety and validation
This is a development prototype, not a diagnostic or certified medical device. Validate ECG amplitude, polarity, frequency response, lead-off behavior and SpO2 against appropriate reference equipment before research or clinical claims. Use patient isolation. Never attach electrodes to a person while the ESP32 is connected to a non-isolated mains-powered computer or supply.
