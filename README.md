# maduino-bilge-monitor

Bilge pump and battery monitor for boats using the Maduino Zero 4G (SIM7600E).
Sends SMS alerts when the bilge pump runs too frequently, battery voltage is low,
temperature is too high, or humidity is too high. Responds to SMS commands for
remote status checks and control.

## Hardware

- [Maduino Zero 4G](https://www.makerfabs.com/maduino-zero-4g-lte.html) (SIM7600E)
- DS3231 RTC module (I2C)
- DHT11 temperature and humidity sensor
- DFRobot 25V voltage sensor
- SIM card with SMS capability (tested with Aldi Mobile on Telstra network)

## Wiring

| Component | Pin |
|-----------|-----|
| Bilge pump signal | D10 (active LOW) |
| Voltage sensor | A0 |
| DHT11 data | D2 |
| RTC | I2C (SDA/SCL) |

## Setup

### 1. Install Arduino libraries

- RTClib by Adafruit
- DHT sensor library by Adafruit

### 2. Configure credentials

Copy `config.h.example` to `config.h` and fill in your details:

```cpp
#define ALERT_PHONE "+61400000000"   // Your phone number
#define SMSC_NUMBER "+61418706275"   // Your carrier SMS service centre
```

`config.h` is excluded from Git so your phone number will never be committed.

### 3. Adjust thresholds

Edit these defines at the top of `BilgePumpMonitor.ino` to suit your vessel:

```cpp
#define PUMP_THRESHOLD            5       // Alert if pump runs more than this per hour
#define LOW_VOLTAGE_THRESHOLD     11.5    // Alert below this voltage (12V system)
#define HIGH_TEMP_THRESHOLD       45.0    // Alert above this temperature (°C)
#define HIGH_HUMIDITY_THRESHOLD   85.0    // Alert above this humidity (%)
```

### 4. Upload

Open `BilgePumpMonitor.ino` in the Arduino IDE, select the Maduino Zero 4G board
and upload.

## SMS Commands

Send any of these commands via SMS to the device:

| Command | Description |
|---------|-------------|
| `STATUS` | Current readings and pump count |
| `SENSORS` | Detailed temperature, humidity and voltage |
| `TIME` | Current RTC time |
| `SET TIME YYYY-MM-DD HH:MM:SS` | Set the RTC clock |
| `RESET ALL` | Reset all alert flags and pump counter |
| `RESET VOLTAGE` | Reset battery alert |
| `RESET PUMP` | Reset pump alert and counter |
| `RESET TEMP` | Reset temperature alert |
| `RESET HUMIDITY` | Reset humidity alert |
| `TEST VOLTAGE` | Trigger a test voltage alert |
| `TEST PUMP` | Trigger a test pump alert |
| `TEST TEMP` | Trigger a test temperature alert |
| `TEST HUMIDITY` | Trigger a test humidity alert |
| `HELP` | List all available commands |

Prefix commands with `#` when sending from the Arduino Serial Monitor for testing.

## Automatic Reports

Morning (08:00) and evening (16:00) status reports are implemented but currently
disabled. To re-enable, uncomment the `if()` blocks in `checkAutomaticReports()`.

## Alerts

The device sends an SMS alert when:

- Bilge pump runs more than `PUMP_THRESHOLD` times in an hour
- Battery voltage drops below `LOW_VOLTAGE_THRESHOLD`
- Temperature exceeds `HIGH_TEMP_THRESHOLD`
- Humidity exceeds `HIGH_HUMIDITY_THRESHOLD`

Alerts will not repeat until the condition clears and returns, or until manually
reset via SMS command.