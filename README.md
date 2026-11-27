# maduino-bilge-monitor

Bilge pump monitor, run-time cutout and battery monitor for boats, using the
Maduino Zero 4G (SIM7600E). Sends SMS alerts when the bilge pump runs too
frequently, battery voltage is low, temperature is too high, or humidity is too
high. Responds to SMS commands for remote status checks and control.

**Pump guard (v1.3+):** the Maduino also times how long the Water Witch level
switch calls for the pump. If the pump runs too long it interrupts the pump's
power with a relay, waits an escalating lockout (10, 30, 60, then 120 minutes),
and retries. This stops a stuck float switch from flattening the battery on a
boat that is left unattended for 7 to 14 days.

A companion ESP32 gateway relays the same data, plus the guard state, into
Signal K for display and logging.

## Repository layout

| Folder / file | What it is |
|---------------|------------|
| `BilgePumpMonitor/BilgePumpMonitor.ino` | The Maduino Zero 4G sketch: sensors, SMS alerting, OLED display, pump guard wiring |
| `BilgePumpMonitor/PumpGuard.h` | The pump guard logic as a plain C++ class with no Arduino dependencies, so it can be unit tested on a PC |
| `esp32-gateway/` | PlatformIO/SensESP project: relays sensor data and guard state into Signal K over Wi-Fi |
| `config.example.h` | Template for `config.h` (phone numbers). Copy into `BilgePumpMonitor/` |

`BilgePumpMonitor.ino` and `PumpGuard.h` must sit in the same folder.

The Maduino and the ESP32 are two separate microcontrollers connected by a single
one-way wire (see Architecture below), each with its own toolchain, kept in one
repo because they share the `$BILGE` and `$PUMP` data formats documented here.

## Architecture

```
Maduino Zero 4G  --(one-way serial, D3 -> GPIO16)-->  ESP32 (SensESP)  --(Wi-Fi)-->  Signal K
```

Data flows **one way only**, Maduino -> ESP32 -> Signal K, never the reverse. This
is deliberate: the Maduino's SMS alerting and pump guard are the safety-critical
functions of this whole system, and they must keep working even if the ESP32,
Wi-Fi, or Signal K server are down, misconfigured, or being worked on. The Maduino
has no dependency on the ESP32 at all. It just transmits its status lines whether
or not anything is listening.

## Hardware

- [Maduino Zero 4G](https://www.makerfabs.com/maduino-zero-4g-lte.html) (SIM7600E)
- DS3231 RTC module (I2C)
- DHT11 temperature and humidity sensor
- DFRobot 25V voltage sensor
- SSD1306 OLED display (I2C)
- SIM card with SMS capability (tested with Aldi Mobile on Telstra network)
- ESP32-WROOM-32 dev board (any board running SensESP) for the Signal K gateway

Pump guard hardware:

- Water Witch electronic bilge level relay (the existing pump controller)
- Rule 1000 12 V bilge pump (about 2.9 A running)
- 2-channel 12 V opto-isolated relay board with per-channel High/Low trigger jumpers
  (K1 = interrupt relay, K2 = demand sense)
- 1-channel PC817 opto-isolator module, 12 V input (pump feed sense)
- R1: 30 A automotive 12 V SPDT relay (switches the pump current)
- Q1: NPN transistor, BC337 or 2N3904 (drives the K1 input from a 3.3 V pin)
- Resistors: 2.2k, 10k and 1k
- Two 1N5408 diodes (flyback protection across the R1 coil and across the pump)
- Pump power wiring about 1.5 mm2 (16 AWG) stranded, tinned. 24 or 25 AWG is fine
  for the signal wiring only
- Existing manual override switch with lamp, in parallel with R1's contact

## Pump guard: how it works

### Power path

```
+12V (fused) --> Water Witch --> N0 --+--> K1 (NC contact) --> R1 coil --> 0 V
                                      +--> K2 demand sense (opto board IN2)

+12V (fused) --> R1 contact (30 to 87) --+
                                          +--> pump node --+--> pump --> 0 V
+12V (fused) --> manual override switch --+                 +--> PC817 input (feed sense)
```

- K1 is wired through its **normally closed** contact. Energising K1 opens the
  contact, R1 drops out and the pump is held off.
- K2 looks at the Water Witch output upstream of K1, so the guard still sees the
  Water Witch calling while the pump is held off.
- The PC817 sees whether voltage actually reaches the pump, whichever way it was
  switched (R1 or the override).
- 1N5408 across the R1 coil and across the pump, stripe (cathode) towards the
  positive side. Wrongly fitted, a diode is a dead short.

### Fail-safe

K1 only interrupts the pump when the Maduino drives it. A Maduino that is dead,
reset or hung means K1 is off and the pump works normally. A hardware watchdog
(Adafruit SleepyDog, 16 s) resets a hung Maduino, which drops every pin to an
input; Q1's 10k pull-down then turns K1 off.

### Behaviour

| State | Meaning |
|-------|---------|
| `IDLE` | No demand from the Water Witch |
| `RUNNING` | Demand present and the pump is allowed to run. Longer than the run limit trips the guard |
| `LOCKOUT` | K1 energised, pump held off for an escalating time, then released for a retry |

- Default run limit is 180 s. Lockouts escalate 10, 30, 60, then 120 minutes (the
  longest repeats).
- Battery below 11.8 V on a trip jumps straight to the longest lockout.
- Demand clearing during a lockout releases the pump at once and resets the count.
- A run that ends normally after a retry resets the count.
- Inputs are debounced (250 ms), so brief blips are ignored.
- Over 14 days of a stuck float switch the pump runs about 2.5% of the time.

Independent fault checks, reported in both modes (confirmed for 10 s, reminder
every 6 h):

| Fault | Mask bit | Meaning |
|-------|----------|---------|
| Feed dead | 1 | Water Witch calling, not cut, but no voltage at the pump |
| Live while cut | 2 | Cut, but voltage still at the pump (K1/R1 stuck, or the override is on) |
| Live with no demand | 4 | No demand, not cut, but voltage at the pump (override on, or R1 stuck) |

### Log-only and enforcing

The guard **starts in log-only mode after every reboot**. It reports "would have
cut" by SMS but never energises K1. `GUARD ENFORCE` switches it on until the next
reboot. To start enforcing at boot, set `GUARD_START_ENFORCING` to `true` and reflash.

### Settings in the sketch

```cpp
#define GUARD_START_ENFORCING   false   // false = log-only after every reboot
#define GUARD_RUN_LIMIT_S       180     // pump may run this long nonstop
#define GUARD_LOW_BATT_V        11.8    // below this, a trip uses the longest lockout
#define WDT_TIMEOUT_MS          16000   // hardware watchdog
#define PUMP_LINE_INTERVAL      10000   // $PUMP line to the ESP32 at least this often (ms)
```

The lockout schedule is in `PumpGuard.h` (`defaultConfig()`).

## Wiring: Maduino

| Maduino pin | Direction | Connects to |
|-------------|-----------|-------------|
| D10 | Input (pull-up) | K2 demand contact through 1k. LOW = Water Witch calling |
| A1 | Input (pull-up) | PC817 module output. LOW = voltage at the pump |
| D4 | Output | Q1 base through 2.2k. HIGH = K1 on, pump held off |
| D3 | Output | ESP32 GPIO16 (one-way serial) |
| D2 | Input | DHT11 data |
| A0 | Analog input | DFRobot voltage sensor |
| SDA / SCL | I2C | DS3231 RTC and SSD1306 OLED |
| 3.3 V | Power out | PC817 module VCC |
| GND | | Common ground (see below) |

D5, D6 and D7 are the SIM7600E power key, reset and flight mode and are driven by
the sketch. D8 and D9 are also wired to the modem (RI, DTR). Do not use D5 to D9.

All grounds must be common: Maduino GND, opto board DC-, PC817 module, Q1 emitter
and the ESP32.

### Q1 driver and the opto relay board

- **K1 (CH1):** jumper on **Low**. Q1 collector to IN1, emitter to ground, base from
  D4 through 2.2k, 10k from base to ground. HIGH on D4 turns Q1 on, pulls IN1 low and
  energises K1.
- **K2 (CH2):** jumper on **High**. IN2 on the Water Witch output. K2 COM to ground,
  K2 NO through 1k to D10.
- **Q1 pinout differs by type.** Flat face towards you, legs down: BC337 is
  Collector, Base, Emitter. 2N3904 is Emitter, Base, Collector.

### PC817 module

12 V input from the pump node, VCC from the Maduino 3.3 V, output to A1. The module
output is active LOW: 3.3 V with no input, about 0.1 V with 12 V applied. It
tolerates 13.8 to 14 V.

## Wiring: Maduino to ESP32 gateway

| Maduino | ESP32 |
|---------|-------|
| D3 | GPIO16 (RX2) |
| GND | GND |

The ESP32's TX pin (GPIO17) is left unconnected, because this link is transmit-only
by design. The Maduino's serial logic is 3.3V (SAMD21 native I/O), matching the
ESP32's GPIO directly, so no level shifter is needed.

The link is implemented as a bit-banged software UART on the Maduino side
(`bilgeSerialBegin()` / `bilgeSendLine()` in the sketch), running at 9600 baud,
rather than a hardware SERCOM UART. This was a deliberate choice: it needs no
board-specific pin-mux knowledge to get right, and critically, it never disables
interrupts, so it can't ever delay or drop an incoming AT-command byte from the
SIM7600E modem.

## Data formats

### The `$BILGE` sentence

Once per `CHECK_INTERVAL` (60 s), the Maduino transmits one line on D3:

```
$BILGE,<voltage>,<pumpCount>,<temperatureC>,<humidityPct>\n
```

Example: `$BILGE,13.79,2,23.40,61.60`

| Field | Signal K path | Units |
|-------|---------------|-------|
| Voltage | `electrical.batteries.bilge.voltage` | Volts |
| Pump count | `electrical.pumps.bilge.cycles` | count |
| Temperature | `environment.inside.bilge.temperature` | Kelvin (converted from °C) |
| Humidity | `environment.inside.bilge.relativeHumidity` | 0-1 ratio (converted from %) |

The pump count increments each time the PC817 sees the pump feed go live, so it
counts times the pump actually ran.

### The `$PUMP` sentence

Sent on any change of guard state or inputs, and at least every 10 s:

```
$PUMP,<state>,<cut>,<demand>,<feed>,<enforce>,<trips>,<lastRunS>,<maxRunS>,<faultMask>,<lockoutLeftS>\n
```

Example: `$PUMP,2,1,1,0,1,1,30,30,0,574`

| Field | Signal K path (under `electrical.pumps.bilge.guard.`) | Meaning |
|-------|-------|---------|
| state | `state` | 0 idle, 1 running, 2 lockout (published as `idle` / `running` / `lockout`) |
| cut | `cut` | K1 energised, pump held off |
| demand | `demand` | Water Witch calling |
| feed | `feed` | Voltage at the pump |
| enforce | `enforcing` | false = log-only |
| trips | `trips` | Consecutive trips since the pump last ran normally |
| lastRunS | `lastRun` | Last completed run, seconds |
| maxRunS | `maxRun` | Longest run since the Maduino last rebooted, seconds |
| faultMask | `faultMask` | 1 feed dead, 2 live while cut, 4 live with no demand |
| lockoutLeftS | `lockoutRemaining` | Seconds until the retry |
| (derived) | `linkOk` | False if no `$PUMP` line for 35 s (Maduino down or link broken) |

If you change a sentence format on the Maduino side, update the `sscanf` pattern in
`esp32-gateway/src/main.cpp` in the same commit. They're a matched pair.

## Setup: Maduino sketch

### 1. Install Arduino libraries

- RTClib by Adafruit
- DHT sensor library by Adafruit (pulls in Adafruit Unified Sensor)
- Adafruit GFX Library
- Adafruit SSD1306
- Adafruit SleepyDog Library (hardware watchdog, new in v1.3)

### 2. Configure credentials

Copy `config.example.h` to `config.h` (inside `BilgePumpMonitor/`) and fill in
your details:

```cpp
#define ALERT_PHONE "+61400000000"   // Your phone number
#define SMSC_NUMBER "+61418706275"   // Your carrier SMS service centre
```

`ALERT_PHONE` receives the alerts and is also the **only number the device accepts
SMS commands from**. Commands from any other number are ignored and deleted.

`config.h` is excluded from Git so your phone number will never be committed.

### 3. Adjust thresholds

Edit these defines at the top of `BilgePumpMonitor.ino` to suit your vessel:

```cpp
#define PUMP_THRESHOLD            5       // Alert if the pump runs more than this per hour
#define LOW_VOLTAGE_THRESHOLD     11.5    // Alert below this voltage (12V system)
#define HIGH_TEMP_THRESHOLD       45.0    // Alert above this temperature (°C)
#define HIGH_HUMIDITY_THRESHOLD   85.0    // Alert above this humidity (%)
#define RESET_INTERVAL            3600000 // Reset pump counter every hour
```

`PUMP_THRESHOLD` is intentionally left low to start. It's easier to lower the
sensitivity once you've established a baseline for what's normal on your boat
than to guess a good number up front.

### 4. Upload

Open `BilgePumpMonitor/BilgePumpMonitor.ino` in the Arduino IDE, select the board
**Arduino Zero (Native USB Port)** and the Maduino's `usbmodem` port, and upload.
If the port is missing, check the cable carries data, or double-tap reset to enter
the bootloader. The serial monitor runs at 115200 baud. The startup banner shows the
firmware version, the reset cause and the guard mode.

## Setup: ESP32 gateway

The `esp32-gateway/` folder is a [PlatformIO](https://platformio.org/) project
using [SensESP](https://github.com/SignalK/SensESP) 3.x. Build and upload with:

```bash
cd esp32-gateway
pio run                  # build only
pio run -t upload        # build and flash (-t = target)
```

`platformio.ini` pins `upload_port` to one USB device. If the ESP32 shows up under
a different port (check `ls /dev/cu.*`), edit that line, or override it for one run
with `pio run -t upload --upload-port /dev/cu.usbserial-XXXX`. To watch the ESP32's
own log, use `pio device monitor --port /dev/cu.usbserial-XXXX`. Without `--port`
it may open the Maduino instead.

On first boot, the ESP32 starts a Wi-Fi access point (`bilge-esp32` by default)
for initial configuration. Connect to it and set your boat's Wi-Fi network
credentials and Signal K server address via the web UI at `http://<esp32-ip>/`.
Automatic mDNS discovery of the Signal K server can be unreliable on a Pi with
multiple network interfaces. If the device doesn't show up under Signal K's
Devices page, turn off "Automatic server discovery" in the ESP32's config page
and enter the Signal K server's IP address and port directly.

## SMS Commands

Send any of these commands via SMS from the `ALERT_PHONE` number. Case doesn't
matter. Use single spaces.

| Command | Description |
|---------|-------------|
| `STATUS` | Current readings, pump count and guard state |
| `SENSORS` | Detailed temperature, humidity and voltage |
| `TIME` | Current RTC time |
| `SET TIME YYYY-MM-DD HH:MM:SS` | Set the RTC clock (24 h, colons in the time) |
| `RESET ALL` | Reset all alert flags and pump counter |
| `RESET VOLTAGE` | Reset battery alert |
| `RESET PUMP` | Reset pump alert and counter |
| `RESET TEMP` | Reset temperature alert |
| `RESET HUMIDITY` | Reset humidity alert |
| `TEST VOLTAGE` | Trigger a test voltage alert |
| `TEST PUMP` | Trigger a test pump alert |
| `TEST TEMP` | Trigger a test temperature alert |
| `TEST HUMIDITY` | Trigger a test humidity alert |
| `GUARD` | Guard mode, state, lockout time left, last and max run |
| `GUARD LOG` | Log-only: the pump is never cut |
| `GUARD ENFORCE` | Enforce: cut the pump if it runs past the limit |
| `GUARD CLEAR` | Release the pump and reset the trip count |
| `GUARD LIMIT n` | Set the run limit to n seconds (30 to 3600) |
| `HELP` | List all available commands |

`GUARD LOG`, `GUARD ENFORCE` and `GUARD LIMIT` last until the next reboot, then the
defaults in the sketch apply.

Prefix commands with `#` **only** when typing them into the Arduino Serial Monitor
for testing (for example `#SET TIME 2026-11-27 21:00:00`). Do not use `#` in an SMS.

## Automatic Reports

Morning (08:00) and evening (16:00) status reports are implemented but currently
disabled. To re-enable, uncomment the `if()` blocks in `checkAutomaticReports()`.

## Alerts

The device sends an SMS alert when:

- Bilge pump runs more than `PUMP_THRESHOLD` times within `RESET_INTERVAL`
  (currently 1 hour)
- Battery voltage drops below `LOW_VOLTAGE_THRESHOLD`
- Temperature exceeds `HIGH_TEMP_THRESHOLD`
- Humidity exceeds `HIGH_HUMIDITY_THRESHOLD`
- The pump guard trips (enforcing), or would have tripped (log-only)
- The pump runs again after a lockout and finishes normally, or the Water Witch
  clears during a lockout
- A pump fault is detected or clears (see the fault table above)

Alerts will not repeat until the condition clears and returns, or until manually
reset via SMS command. Repeated guard trips send an SMS for trips 1 to 4, then
every third trip. Faults repeat as a reminder every 6 hours.

On boot, the Maduino waits for cellular network registration (up to 30
seconds) before attempting the startup SMS, rather than a fixed delay. This
fixes a cold-boot failure mode where the startup SMS would silently fail to
send because the modem hadn't yet registered on the network. The startup SMS
states the guard mode and the reset cause (power-on, watchdog, and so on).

## Testing the pump guard on the bench

Run in log-only mode first. For the cutout test use a 12 V lamp as the pump load,
or put the pump in water. A submersible pump should not run dry for more than a few
seconds.

1. Flash and check the startup banner (`Guard LOG-ONLY`, run limit 180 s).
2. Inputs: ground D10 and the OLED title changes from `OK-L` to `RUN-L`. Ground A1 and
   the pump counter increments.
3. Timing: hold demand past the run limit and a single "would trip" SMS arrives.
4. Cutout: `GUARD LIMIT 30`, `GUARD ENFORCE`, then hold demand with the real chain wired
   (Water Witch signal through K1 NC to the R1 coil, R1 contact to the lamp, PC817 on
   the lamp). K1 and R1 click, the lamp goes out, the OLED shows `CUT` and a trip SMS
   arrives. Dropping the demand releases the pump immediately.
5. Faults: with no demand, apply voltage to the feed input for over 10 s and a "pump
   live with no demand" SMS arrives. Switch the manual override on while cut and a
   "live while cut" SMS arrives.
6. ESP32: the `electrical.pumps.bilge.guard.*` paths appear in the Signal K Data
   Browser. Unplug D3 and `linkOk` goes false after about 35 s.

`PumpGuard.h` has no Arduino dependencies and can be tested on a PC with a normal C++
compiler.

## Roadmap

- Second float switch (high-high level): emergency SMS, and release the lockout when
  water is genuinely rising. Distinguishes a stuck float from a real flood.
- Pump current sensing (INA219 with an external low-value shunt on the supply side of
  both R1 and the override): detect a pump running dry or jammed.
- Optional second SMS recipient number.

## Version history

| Version | Changes |
|---------|---------|
| v1.2 | Monitoring and SMS alerts only |
| v1.3 | Pump guard, hardware watchdog, `$PUMP` line and Signal K guard paths |
| v1.3.1 | SMS commands accepted only from `ALERT_PHONE` (sender number was previously misread) |
