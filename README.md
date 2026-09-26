# maduino-bilge-monitor

Bilge pump and battery monitor for boats using the Maduino Zero 4G (SIM7600E).
Sends SMS alerts when the bilge pump runs too frequently, battery voltage is low,
temperature is too high, or humidity is too high. Responds to SMS commands for
remote status checks and control. A companion ESP32 gateway relays the same
sensor data into Signal K for display and logging.

## Repository layout

| Folder | What it is |
|--------|------------|
| `BilgePumpMonitor/` | The Maduino Zero 4G sketch — sensors, SMS alerting, OLED display |
| `esp32-gateway/` | PlatformIO/SensESP project — relays sensor data into Signal K over Wi-Fi |

These are two separate microcontrollers connected by a single one-way wire (see
Architecture below), each with its own toolchain, kept in one repo because they
share the `$BILGE` data format documented here.

## Architecture

```
Maduino Zero 4G  --(one-way serial, D3 -> GPIO16)-->  ESP32 (SensESP)  --(Wi-Fi)-->  Signal K
```

Data flows **one way only**, Maduino -> ESP32 -> Signal K, never the reverse. This
is deliberate: the Maduino's SMS alerting is the safety-critical function of this
whole system, and it must keep working even if the ESP32, Wi-Fi, or Signal K
server are down, misconfigured, or being worked on. The Maduino has no dependency
on the ESP32 at all — it just transmits its status sentence every check interval
whether or not anything is listening.

## Hardware

- [Maduino Zero 4G](https://www.makerfabs.com/maduino-zero-4g-lte.html) (SIM7600E)
- DS3231 RTC module (I2C)
- DHT11 temperature and humidity sensor
- DFRobot 25V voltage sensor
- SSD1306 OLED display (I2C)
- SIM card with SMS capability (tested with Aldi Mobile on Telstra network)
- ESP32-WROOM-32 dev board (any board running SensESP) for the Signal K gateway

## Wiring — Maduino sensors

| Component | Pin |
|-----------|-----|
| Bilge pump signal | D10 (active LOW) |
| Voltage sensor | A0 |
| DHT11 data | D2 |
| RTC + OLED | I2C (SDA/SCL) |
| ESP32 gateway link (TX, one-way) | D3 |

D3 was chosen because it's confirmed free on the Maduino Zero 4G board —
D7/D8/D9 are internally wired to the SIM7600E modem (flight mode, RI, DTR) even
though they're never referenced in this sketch, so they're not safe to reuse.

## Wiring — Maduino to ESP32 gateway

| Maduino | ESP32 |
|---------|-------|
| D3 | GPIO16 (RX2) |
| GND | GND |

The ESP32's TX pin (GPIO17) is left unconnected — this link is transmit-only,
by design. The Maduino's serial logic is 3.3V (SAMD21 native I/O), matching the
ESP32's GPIO directly — no level shifter needed.

The link is implemented as a bit-banged software UART on the Maduino side
(`bilgeSerialBegin()` / `bilgeSendLine()` in the sketch), running at 9600 baud,
rather than a hardware SERCOM UART. This was a deliberate choice: it needs no
board-specific pin-mux knowledge to get right, and critically, it never disables
interrupts, so it can't ever delay or drop an incoming AT-command byte from the
SIM7600E modem.

## Data format — the `$BILGE` sentence

Once per `CHECK_INTERVAL`, the Maduino transmits one line on D3:

```
$BILGE,<voltage>,<pumpCount>,<temperatureC>,<humidityPct>\n
```

Example: `$BILGE,13.79,2,23.40,61.60`

The ESP32 gateway (`esp32-gateway/src/main.cpp`) parses this with `sscanf` and
publishes to Signal K as:

| Field | Signal K path | Units |
|-------|---------------|-------|
| Voltage | `electrical.batteries.bilge.voltage` | Volts |
| Pump count | `electrical.pumps.bilge.cycles` | count |
| Temperature | `environment.inside.bilge.temperature` | Kelvin (converted from °C) |
| Humidity | `environment.inside.bilge.relativeHumidity` | 0-1 ratio (converted from %) |

If you change this sentence format on the Maduino side, update the `sscanf`
pattern in `esp32-gateway/src/main.cpp` in the same commit — they're a matched
pair.

## Setup — Maduino sketch

### 1. Install Arduino libraries

- RTClib by Adafruit
- DHT sensor library by Adafruit (pulls in Adafruit Unified Sensor)
- Adafruit GFX Library
- Adafruit SSD1306

### 2. Configure credentials

Copy `config.example.h` to `config.h` (inside `BilgePumpMonitor/`) and fill in
your details:

```cpp
#define ALERT_PHONE "+61400000000"   // Your phone number
#define SMSC_NUMBER "+61418706275"   // Your carrier SMS service centre
```

`config.h` is excluded from Git so your phone number will never be committed.

### 3. Adjust thresholds

Edit these defines at the top of `BilgePumpMonitor.ino` to suit your vessel:

```cpp
#define PUMP_THRESHOLD            5       // Alert if pump runs more than this per 24hr period
#define LOW_VOLTAGE_THRESHOLD     11.5    // Alert below this voltage (12V system)
#define HIGH_TEMP_THRESHOLD       45.0    // Alert above this temperature (°C)
#define HIGH_HUMIDITY_THRESHOLD   85.0    // Alert above this humidity (%)
#define RESET_INTERVAL            86400000 // Reset pump counter every 24 hours
```

`PUMP_THRESHOLD` is intentionally left low to start — it's easier to lower the
sensitivity once you've established a baseline for what's normal on your boat
than to guess a good number up front.

### 4. Upload

Open `BilgePumpMonitor.ino` in the Arduino IDE, select the Maduino Zero 4G board
and upload.

## Setup — ESP32 gateway

The `esp32-gateway/` folder is a [PlatformIO](https://platformio.org/) project
using [SensESP](https://github.com/SignalK/SensESP). Build/upload with:

```bash
cd esp32-gateway
pio run --target upload
```

On first boot, the ESP32 starts a Wi-Fi access point (`bilge-esp32` by default)
for initial configuration — connect to it and set your boat's Wi-Fi network
credentials and Signal K server address via the web UI at `http://<esp32-ip>/`.
Automatic mDNS discovery of the Signal K server can be unreliable on a Pi with
multiple network interfaces — if the device doesn't show up under Signal K's
Devices page, turn off "Automatic server discovery" in the ESP32's config page
and enter the Signal K server's IP address and port directly.

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

- Bilge pump runs more than `PUMP_THRESHOLD` times within `RESET_INTERVAL`
  (currently 24 hours)
- Battery voltage drops below `LOW_VOLTAGE_THRESHOLD`
- Temperature exceeds `HIGH_TEMP_THRESHOLD`
- Humidity exceeds `HIGH_HUMIDITY_THRESHOLD`

Alerts will not repeat until the condition clears and returns, or until manually
reset via SMS command.

On boot, the Maduino now waits for cellular network registration (up to 30
seconds) before attempting the startup SMS, rather than a fixed delay — this
fixes a cold-boot failure mode where the startup SMS would silently fail to
send because the modem hadn't yet registered on the network.
