// BilgePumpMonitor
// Maduino Zero 4G (SIM7600E) - Bilge Pump and Battery Monitor
// Monitors bilge pump cycles, battery voltage, temperature and humidity.
// Sends SMS alerts when thresholds are exceeded.
// Responds to SMS commands for remote status and control.
// Pump guard: times how long the Water Witch calls for the pump and, if the
// pump runs too long, holds it off through the K1 interrupt relay with an
// escalating auto-retry lockout (see PumpGuard.h).
//
// Hardware:
//   - Maduino Zero 4G (SIM7600E LTE module)
//   - DS3231 RTC module (I2C)
//   - DHT11 temperature/humidity sensor
//   - DFRobot 25V voltage sensor on A0
//   - K2 demand contact on pin 10 (active LOW: Water Witch is calling)
//   - PC817 pump-feed module on A1 (active LOW: voltage at the pump)
//   - K1 interrupt relay driven from pin 4 through NPN Q1 (HIGH = pump held off)
//
// Libraries: Adafruit SleepyDog (new, for the watchdog), RTClib, DHT,
//            Adafruit GFX, Adafruit SSD1306. PumpGuard.h must sit in the
//            same folder as this sketch.
//
// Configuration:
//   Copy config.h.example to config.h and fill in your details.
//   See README.md for full setup instructions.

#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>
#include "config.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SleepyDog.h>
#include "PumpGuard.h"

// Pin definitions for Maduino Zero 4G (LTE)
#define LTE_RESET_PIN    6
#define LTE_PWRKEY_PIN   5
#define LTE_FLIGHT_PIN   7

// Monitoring pins
#define DEMAND_PIN       10   // K2 contact to ground: LOW = Water Witch is calling for the pump
#define FEED_PIN         A1   // PC817 module output: LOW = voltage present at the pump
#define K1_PIN           4    // Drives Q1 -> K1 interrupt relay: HIGH = pump held off
#define BATTERY_PIN      A0   // DFRobot 25V voltage sensor
#define DHT_PIN          2
#define DHT_TYPE         DHT11

// Alert thresholds
#define PUMP_THRESHOLD            5       // Alert if pump runs more than this per hour
#define LOW_VOLTAGE_THRESHOLD     11.5    // Alert below this voltage (12V system)
#define HIGH_TEMP_THRESHOLD       45.0    // Alert above this temperature (°C)
#define HIGH_HUMIDITY_THRESHOLD   85.0    // Alert above this humidity (%)
#define CHECK_INTERVAL            60000   // Check every 60 seconds
#define RESET_INTERVAL            3600000 // Reset pump counter every hour

// --- Pump guard (run-time cutout) ---
// The lockout schedule (10, 30, 60, then 120 minutes) lives in PumpGuard.h.
#define GUARD_START_ENFORCING   false   // false = log-only after every reboot, true = cut the pump
#define GUARD_RUN_LIMIT_S       180     // pump may run this long nonstop before the guard trips
#define GUARD_LOW_BATT_V        11.8    // below this, a trip jumps straight to the longest lockout
#define WDT_TIMEOUT_MS          16000   // hardware watchdog (SAMD21 maximum is about 16 s)
#define PUMP_LINE_INTERVAL      10000   // send a $PUMP status line to the ESP32 at least this often (ms)

// --- Firmware version banner ---
#define FW_VERSION "v1.3"

// --- One-way link to the ESP32 gateway (bit-banged UART TX, 9600 baud) ---
#define BILGE_TX_PIN 3          // wire to ESP32 GPIO16 (RX2); leave ESP32 TX unconnected
#define BILGE_BAUD   9600
#define BILGE_BIT_US (1000000UL / BILGE_BAUD)

// OLED Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  // Reset pin (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // Common I2C address (try 0x3D if 0x3C doesn't work)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Explicit prototype: the IDE's automatic prototypes can trip over nested types.
String guardEventText(const PumpGuard::Event& ev);

// Global variables
int pumpCount = 0;
bool lastPumpState = false;
float batteryVoltage = 0.0;
float temperature = 0.0;
float humidity = 0.0;
unsigned long lastCheckTime = 0;
unsigned long lastResetTime = 0;
bool lowVoltageAlertSent = false;
bool highPumpAlertSent = false;
bool highTempAlertSent = false;
bool highHumidityAlertSent = false;

// Pump guard
PumpGuard guard;
bool wdtEnabled = false;
unsigned long lastPumpLine = 0;

// Automatic status report flags
// Note: morning and evening reports are implemented but currently disabled.
// To re-enable, uncomment the if() blocks in checkAutomaticReports().
bool morningReportSent = false;
bool eveningReportSent = false;

bool waitForNetworkRegistration(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    Serial1.println("AT+CREG?");
    delay(500);
    String resp = "";
    while (Serial1.available()) {
      char c = Serial1.read();
      resp += c;
      SerialUSB.write(c);
    }
    if (resp.indexOf("+CREG: 0,1") >= 0 || resp.indexOf("+CREG: 0,5") >= 0 ||
        resp.indexOf("+CREG: 1,1") >= 0 || resp.indexOf("+CREG: 1,5") >= 0) {
      SerialUSB.println("Network registered");
      return true;
    }
    delay(1000);
  }
  SerialUSB.println("WARNING: network registration timed out");
  return false;
}
// SMS handling
String modemBuffer = "";
String lastSender = "";
String pendingDeleteIndex = "";

// Hardware objects
RTC_DS3231 rtc;
DHT dht(DHT_PIN, DHT_TYPE);

void bilgeSerialBegin() {
  pinMode(BILGE_TX_PIN, OUTPUT);
  digitalWrite(BILGE_TX_PIN, HIGH);   // idle high, like a real UART line
}

void bilgeSendByte(uint8_t b) {
  digitalWrite(BILGE_TX_PIN, LOW);           // start bit
  delayMicroseconds(BILGE_BIT_US);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(BILGE_TX_PIN, (b & 0x01) ? HIGH : LOW);
    b >>= 1;
    delayMicroseconds(BILGE_BIT_US);
  }
  digitalWrite(BILGE_TX_PIN, HIGH);          // stop bit
  delayMicroseconds(BILGE_BIT_US);
}

void bilgeSendLine(const String& line) {
  for (size_t i = 0; i < line.length(); i++) bilgeSendByte((uint8_t)line[i]);
  bilgeSendByte('\n');
}


// Delay that keeps the watchdog fed (used for the long SMS waits).
void delayWD(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (wdtEnabled) Watchdog.reset();
    delay(10);
  }
}


// Why did the Maduino last reset? Best effort, read from the SAMD21 reset-cause register.
const char* resetCauseText() {
  uint8_t r = PM->RCAUSE.reg;
  if (r & PM_RCAUSE_WDT)  return "watchdog";
  if (r & PM_RCAUSE_SYST) return "software";
  if (r & PM_RCAUSE_EXT)  return "reset button";
  if (r & (PM_RCAUSE_BOD12 | PM_RCAUSE_BOD33)) return "brownout";
  if (r & PM_RCAUSE_POR)  return "power-on";
  return "unknown";
}


void setup() {
  // First thing: K1 off, so the pump is enabled whatever happens next.
  pinMode(K1_PIN, OUTPUT);
  digitalWrite(K1_PIN, LOW);

  SerialUSB.begin(115200);
  unsigned long usbStart = millis();
  while (!SerialUSB && (millis() - usbStart < 3000)) {
    // wait up to 3s for the USB host to finish reconnecting after reset
  }
  delay(100);
  SerialUSB.println("=== BILGE MONITOR " FW_VERSION " ===");
  SerialUSB.println("=== Compiled: " __DATE__ " " __TIME__ " ===");
  SerialUSB.print("Reset cause: "); SerialUSB.println(resetCauseText());

  bilgeSerialBegin();
  Serial1.begin(115200);

  pinMode(DEMAND_PIN, INPUT_PULLUP);
  pinMode(FEED_PIN, INPUT_PULLUP);
  pinMode(BATTERY_PIN, INPUT);

  // Pump guard configuration
  PumpGuard::Config gc = PumpGuard::defaultConfig();
  gc.runLimitMs = GUARD_RUN_LIMIT_S * 1000UL;
  gc.lowBattV   = GUARD_LOW_BATT_V;
  gc.enforce    = GUARD_START_ENFORCING;
  guard.configure(gc);

  dht.begin();

  // Power up SIM7600E
  pinMode(LTE_RESET_PIN, OUTPUT);
  digitalWrite(LTE_RESET_PIN, LOW);
  pinMode(LTE_PWRKEY_PIN, OUTPUT);
  digitalWrite(LTE_RESET_PIN, LOW);
  delay(100);
  digitalWrite(LTE_PWRKEY_PIN, HIGH);
  delay(2000);
  digitalWrite(LTE_PWRKEY_PIN, LOW);
  pinMode(LTE_FLIGHT_PIN, OUTPUT);
  digitalWrite(LTE_FLIGHT_PIN, LOW);

  SerialUSB.println("=== Bilge Pump Monitor Starting ===");

  if (!rtc.begin()) {
    SerialUSB.println("Couldn't find RTC");
  } else {
    SerialUSB.println("RTC initialised");
    if (rtc.lostPower()) {
      SerialUSB.println("RTC lost power — setting to compile time");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    DateTime now = rtc.now();
    SerialUSB.print("Current time: ");
    SerialUSB.println(formatDateTime(now));
  }

  delay(5000);

  // Initialise SIM7600E
  Serial1.print("AT+CSCA=\"");
  Serial1.print(SMSC_NUMBER);
  Serial1.println("\"");
  delay(1000);
  Serial1.println("AT+CMGF=1");   // SMS text mode
  delay(1000);
  Serial1.println("AT+CNMI=2,1,0,0,0");  // Enable new SMS notifications
  delay(1000);

  SerialUSB.println("SIM7600E initialised");
  SerialUSB.print("Alert phone: "); SerialUSB.println(ALERT_PHONE);
  SerialUSB.print("Pump threshold: "); SerialUSB.print(PUMP_THRESHOLD); SerialUSB.println(" cycles/hr");
  SerialUSB.print("Low voltage threshold: "); SerialUSB.print(LOW_VOLTAGE_THRESHOLD); SerialUSB.println("V");
  SerialUSB.print("High temp threshold: "); SerialUSB.print(HIGH_TEMP_THRESHOLD); SerialUSB.println("°C");
  SerialUSB.print("High humidity threshold: "); SerialUSB.print(HIGH_HUMIDITY_THRESHOLD); SerialUSB.println("%");
  SerialUSB.print("Pump guard: "); SerialUSB.print(guard.enforce() ? "ENFORCING" : "LOG-ONLY");
  SerialUSB.print(", run limit "); SerialUSB.print(GUARD_RUN_LIMIT_S); SerialUSB.println("s");

  SerialUSB.println("Waiting for network registration...");
  waitForNetworkRegistration(30000);   // up to 30s; proceeds either way after that


  DateTime startTime = rtc.now();
  String startupMsg = "Bilge monitor started at ";
  startupMsg += formatDateTime(startTime);
  startupMsg += ". Monitoring pump, battery, temp & humidity.";
  startupMsg += " Guard ";
  startupMsg += guard.enforce() ? "ENFORCING" : "LOG-ONLY";
  startupMsg += ". Reset: ";
  startupMsg += resetCauseText();
  startupMsg += ".";
  sendSMS(ALERT_PHONE, startupMsg);

  lastCheckTime = millis();
  lastResetTime = millis();

  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    SerialUSB.println(F("SSD1306 allocation failed"));
  } else {
    SerialUSB.println("OLED display initialized");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Bilge Monitor");
    display.println("Starting...");
    display.display();
    delay(2000);
  }

  // Start the guard from the real input states, then arm the watchdog.
  // A watchdog reset drops every pin to an input, Q1's pull-down turns K1 off
  // and the pump is enabled again.
  guard.begin(millis(), digitalRead(DEMAND_PIN) == LOW, digitalRead(FEED_PIN) == LOW);
  int wdtMs = Watchdog.enable(WDT_TIMEOUT_MS);
  wdtEnabled = true;
  SerialUSB.print("Watchdog enabled, timeout ms: "); SerialUSB.println(wdtMs);
}



void loop() {
  if (wdtEnabled) Watchdog.reset();

  guardTick();            // pump guard first: K1 must not wait behind modem work
  handleModemCommunication();
  checkPumpState();

  if (millis() - lastCheckTime >= CHECK_INTERVAL) {
    checkBatteryVoltage();
    checkTemperatureHumidity();
    checkPumpThreshold();
    checkAutomaticReports();
    updateOLEDDisplay();
    lastCheckTime = millis();

    SerialUSB.print("Status — Pump: "); SerialUSB.print(pumpCount);
    SerialUSB.print(", Battery: "); SerialUSB.print(batteryVoltage); SerialUSB.print("V");
    SerialUSB.print(", Temp: "); SerialUSB.print(temperature); SerialUSB.print("°C");
    SerialUSB.print(", Humidity: "); SerialUSB.print(humidity); SerialUSB.println("%");
        char bilgeMsg[64];
    snprintf(bilgeMsg, sizeof(bilgeMsg), "$BILGE,%.2f,%d,%.1f,%.1f",
             batteryVoltage, pumpCount, temperature, humidity);
    bilgeSendLine(String(bilgeMsg));
  }

  if (millis() - lastResetTime >= RESET_INTERVAL) {
    pumpCount = 0;
    lastResetTime = millis();
    highPumpAlertSent = false;
    highTempAlertSent = false;
    highHumidityAlertSent = false;
    SerialUSB.println("Hourly counters reset");
  }

  delay(100);
}


// ---------------------------------------------------------------------------
// Pump guard
// ---------------------------------------------------------------------------

// Short text for the OLED and STATUS reply: OK / RUN / CUT, with -L in log-only mode.
String guardShort() {
  const char* s = "OK";
  if (guard.state() == PumpGuard::LOCKOUT) s = "CUT";
  else if (guard.state() == PumpGuard::RUNNING) s = "RUN";
  String r = s;
  if (!guard.enforce()) r += "-L";
  return r;
}


// "45s", "12m", "2.5h"
String fmtDuration(uint32_t ms) {
  uint32_t s = ms / 1000UL;
  if (s < 120) return String(s) + "s";
  uint32_t m = s / 60UL;
  if (m < 120) return String(m) + "m";
  return String(m / 60.0, 1) + "h";
}


String guardStatusText() {
  uint32_t now = millis();
  String r = "Guard ";
  r += guard.enforce() ? "ENFORCING" : "LOG-ONLY";
  r += ": ";
  switch (guard.state()) {
    case PumpGuard::IDLE:
      r += "idle";
      break;
    case PumpGuard::RUNNING:
      r += "pump running ";
      r += fmtDuration(guard.runElapsedMs(now));
      break;
    case PumpGuard::LOCKOUT:
      r += "CUT, ";
      r += fmtDuration(guard.lockoutRemainingMs(now));
      r += " left (trip ";
      r += String(guard.tripCount());
      r += ")";
      break;
  }
  r += ". Last run ";
  r += fmtDuration(guard.lastRunMs());
  r += ", max ";
  r += fmtDuration(guard.maxRunMs());
  r += ", limit ";
  r += fmtDuration(guard.runLimitMs());
  r += ".";
  if (guard.faultMask()) {
    r += " FAULT code ";
    r += String(guard.faultMask());
  }
  return r;
}


const char* guardFaultName(uint8_t evType) {
  switch (evType) {
    case PumpGuard::EV_FAULT_FEED_DEAD:      return "no feed while Water Witch calling";
    case PumpGuard::EV_FAULT_LIVE_WHILE_CUT: return "feed live while cut";
    case PumpGuard::EV_FAULT_LIVE_NO_DEMAND: return "feed live with no demand";
  }
  return "fault";
}


// Turns a guard event into SMS text. Returns "" when no SMS should be sent.
String guardEventText(const PumpGuard::Event& ev) {
  String stamp = formatDateTime(rtc.now());
  String m;

  switch (ev.type) {
    case PumpGuard::EV_TRIP:
      // Limit SMS volume on a long fault: trips 1-4, then every third trip.
      if (ev.n > 4 && (ev.n % 3) != 0) return "";
      m = "ALERT " + stamp + ": Pump ran " + fmtDuration(ev.a) + " nonstop. CUT for " +
          fmtDuration(ev.b) + " (trip " + String(ev.n) + "). Batt " + String(batteryVoltage, 1) + "V.";
      break;

    case PumpGuard::EV_WOULD_TRIP:
      m = "NOTE " + stamp + ": Pump has run " + fmtDuration(ev.a) + " nonstop (limit " +
          fmtDuration(guard.runLimitMs()) + "). LOG-ONLY, not cut.";
      break;

    case PumpGuard::EV_RETRY:
      SerialUSB.println("Guard: lockout ended, pump released for retry " + String(ev.n));
      return "";

    case PumpGuard::EV_CLEARED:
      m = "OK " + stamp + ": Pump finished a " + fmtDuration(ev.a) + " run after " +
          String(ev.n) + " cut(s). Guard reset.";
      break;

    case PumpGuard::EV_CLEARED_EARLY:
      m = "OK " + stamp + ": Water Witch cleared while pump was held off (" + String(ev.n) +
          " cut(s)). Pump released, guard reset.";
      break;

    case PumpGuard::EV_FAULT_FEED_DEAD:
      m = String(ev.a ? "REMINDER " : "FAULT ") + stamp +
          ": Water Witch calling but no voltage at pump. Check R1, fuse, wiring.";
      break;

    case PumpGuard::EV_FAULT_LIVE_WHILE_CUT:
      m = String(ev.a ? "REMINDER " : "FAULT ") + stamp +
          ": Pump live while cut. K1/R1 stuck or manual override is ON.";
      break;

    case PumpGuard::EV_FAULT_LIVE_NO_DEMAND:
      m = String(ev.a ? "REMINDER " : "FAULT ") + stamp +
          ": Pump live with no Water Witch demand. Override ON or R1 stuck.";
      break;

    case PumpGuard::EV_FAULT_RECOVERED:
      m = "OK " + stamp + ": Pump fault cleared (" + String(guardFaultName(ev.n)) + ").";
      break;

    default:
      return "";
  }
  return m;
}


void handleGuardEvents() {
  PumpGuard::Event ev;
  while (guard.popEvent(ev)) {
    String msg = guardEventText(ev);
    if (msg.length() > 0) sendSMS(ALERT_PHONE, msg);
  }
}


// $PUMP,state,cut,demand,feed,enforce,trips,lastRunS,maxRunS,faultMask,lockoutLeftS
// state: 0 idle, 1 running, 2 lockout. faultMask: 1 feed dead, 2 live while cut, 4 live no demand.
void sendPumpLine() {
  uint32_t now = millis();
  char buf[80];
  snprintf(buf, sizeof(buf), "$PUMP,%d,%d,%d,%d,%d,%d,%lu,%lu,%d,%lu",
           (int)guard.state(), guard.cut() ? 1 : 0, guard.demand() ? 1 : 0,
           guard.feed() ? 1 : 0, guard.enforce() ? 1 : 0, (int)guard.tripCount(),
           (unsigned long)(guard.lastRunMs() / 1000UL),
           (unsigned long)(guard.maxRunMs() / 1000UL),
           (int)guard.faultMask(),
           (unsigned long)(guard.lockoutRemainingMs(now) / 1000UL));
  bilgeSendLine(String(buf));
  lastPumpLine = now;
}


// Runs every loop.
void guardTick() {
  bool demandRaw = (digitalRead(DEMAND_PIN) == LOW);
  bool feedRaw   = (digitalRead(FEED_PIN) == LOW);

  guard.update(millis(), demandRaw, feedRaw, batteryVoltage);

  // Apply the output before anything slow (SMS) so a cut is never delayed.
  digitalWrite(K1_PIN, guard.cut() ? HIGH : LOW);

  // OLED and ESP32 line on any change, and the ESP32 line at least every PUMP_LINE_INTERVAL.
  static uint16_t lastSig = 0xFFFF;
  uint16_t sig = (uint16_t)guard.state() |
                 (uint16_t)((guard.cut() ? 1 : 0) << 2) |
                 (uint16_t)((guard.demand() ? 1 : 0) << 3) |
                 (uint16_t)((guard.feed() ? 1 : 0) << 4) |
                 (uint16_t)((uint16_t)guard.faultMask() << 5);
  bool changed = (sig != lastSig);
  lastSig = sig;
  if (changed) updateOLEDDisplay();
  if (changed || (millis() - lastPumpLine) >= PUMP_LINE_INTERVAL) sendPumpLine();

  handleGuardEvents();
}


void checkPumpState() {
  // Count a cycle each time the pump feed goes live (the pump actually ran).
  bool currentPumpState = guard.feed();
  if (currentPumpState && !lastPumpState) {
    pumpCount++;
    SerialUSB.print("Pump cycle detected! Count: ");
    SerialUSB.println(pumpCount);
  }
  lastPumpState = currentPumpState;
}


void checkBatteryVoltage() {
  int rawReading = analogRead(BATTERY_PIN);
  batteryVoltage = (rawReading * 3.3 / 1024.0) * 5.0; // DFRobot 5:1 divider

  if (batteryVoltage < LOW_VOLTAGE_THRESHOLD && !lowVoltageAlertSent) {
    DateTime now = rtc.now();
    String msg = "ALERT ";
    msg += formatDateTime(now);
    msg += ": Battery low at ";
    msg += batteryVoltage;
    msg += "V. Check charging system.";
    sendSMS(ALERT_PHONE, msg);
    lowVoltageAlertSent = true;
    SerialUSB.println("Low voltage SMS sent");
  }

  if (batteryVoltage > (LOW_VOLTAGE_THRESHOLD + 0.5)) {
    lowVoltageAlertSent = false;
  }
}


void checkPumpThreshold() {
  if (pumpCount >= PUMP_THRESHOLD && !highPumpAlertSent) {
    DateTime now = rtc.now();
    String msg = "ALERT ";
    msg += formatDateTime(now);
    msg += ": Bilge pumped ";
    msg += pumpCount;
    msg += " times this hour. Check for leaks.";
    sendSMS(ALERT_PHONE, msg);
    highPumpAlertSent = true;
    SerialUSB.println("High pump count SMS sent");
  }
}


void checkTemperatureHumidity() {
  float newHumidity = dht.readHumidity();
  float newTemp = dht.readTemperature();

  if (!isnan(newHumidity) && !isnan(newTemp)) {
    humidity = newHumidity;
    temperature = newTemp;

    if (temperature > HIGH_TEMP_THRESHOLD && !highTempAlertSent) {
      DateTime now = rtc.now();
      String msg = "ALERT ";
      msg += formatDateTime(now);
      msg += ": High temp ";
      msg += temperature;
      msg += "°C. Check ventilation.";
      sendSMS(ALERT_PHONE, msg);
      highTempAlertSent = true;
      SerialUSB.println("High temp SMS sent");
    }

    if (humidity > HIGH_HUMIDITY_THRESHOLD && !highHumidityAlertSent) {
      DateTime now = rtc.now();
      String msg = "ALERT ";
      msg += formatDateTime(now);
      msg += ": High humidity ";
      msg += humidity;
      msg += "%. Check for water ingress.";
      sendSMS(ALERT_PHONE, msg);
      highHumidityAlertSent = true;
      SerialUSB.println("High humidity SMS sent");
    }

    if (temperature < (HIGH_TEMP_THRESHOLD - 2.0)) highTempAlertSent = false;
    if (humidity < (HIGH_HUMIDITY_THRESHOLD - 5.0)) highHumidityAlertSent = false;

  } else {
    SerialUSB.println("Failed to read DHT11 sensor");
  }
}


void sendSMS(String phoneNumber, String message) {
  SerialUSB.println("Sending SMS: " + message);
  Serial1.print("AT+CMGS=\"");
  Serial1.print(phoneNumber);
  Serial1.println("\"");
  delayWD(1000);
  Serial1.print(message);
  Serial1.write(26); // Ctrl+Z
  delayWD(5000);
  SerialUSB.println("SMS sent");
}


void handleModemCommunication() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    SerialUSB.write(c);
    modemBuffer += c;
    if (c == '\n') {
      processModemResponse(modemBuffer);
      modemBuffer = "";
    }
    if (modemBuffer.length() > 1000) modemBuffer = "";
  }

  static String serialBuffer = "";
  while (SerialUSB.available() > 0) {
    char c = SerialUSB.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        if (serialBuffer.startsWith("#")) {
          processSMSCommand(serialBuffer.substring(1));
        } else {
          Serial1.println(serialBuffer);
        }
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}


void processModemResponse(String response) {
  response.trim();

  if (response.indexOf("+CMTI:") >= 0) {
    SerialUSB.println("New SMS: " + response);
    int commaPos = response.lastIndexOf(',');
    if (commaPos > 0) {
      String msgIndex = response.substring(commaPos + 1);
      msgIndex.trim();
      pendingDeleteIndex = msgIndex;
      delay(100);
      Serial1.print("AT+CMGR=");
      Serial1.println(msgIndex);
    }
  } else if (response.indexOf("+CMGR:") >= 0) {
    int quoteStart = response.indexOf('"', response.indexOf('"') + 1) + 1;
    int quoteEnd = response.indexOf('"', quoteStart);
    if (quoteEnd > quoteStart) {
      lastSender = response.substring(quoteStart, quoteEnd);
      SerialUSB.println("SMS from: " + lastSender);
    }
  } else if (response.length() > 0 &&
             !response.startsWith("AT") &&
             !response.startsWith("OK") &&
             !response.startsWith("ERROR") &&
             !response.startsWith("+") &&
             !response.startsWith("RING") &&
             !response.startsWith("CONNECT")) {

    if (lastSender.length() > 0) {
      if (lastSender == ALERT_PHONE) {
        SerialUSB.println("Ignoring SMS from own device");
        lastSender = "";
        if (pendingDeleteIndex.length() > 0) {
          deleteSMS(pendingDeleteIndex);
          pendingDeleteIndex = "";
        }
        return;
      }
      SerialUSB.println("SMS command from " + lastSender + ": " + response);
      processSMSCommand(response);
      if (pendingDeleteIndex.length() > 0) {
        deleteSMS(pendingDeleteIndex);
        pendingDeleteIndex = "";
      }
      lastSender = "";
    }
  }
}


void deleteSMS(String messageIndex) {
  Serial1.print("AT+CMGD=");
  Serial1.println(messageIndex);
  delay(1000);
  SerialUSB.println("Deleted SMS index: " + messageIndex);
}


void processSMSCommand(String command) {
  command.toUpperCase();
  command.trim();
  SerialUSB.println("Processing command: " + command);

  String reply = "";

  if (command == "STATUS") {
    DateTime now = rtc.now();
    reply = "Status ";
    reply += formatDateTime(now);
    reply += ": Battery ";
    reply += batteryVoltage;
    reply += "V, Pump: ";
    reply += pumpCount;
    reply += ", Temp: ";
    reply += temperature;
    reply += "C, Humidity: ";
    reply += humidity;
    reply += "%, Guard ";
    reply += guardShort();

  } else if (command == "SENSORS") {
    DateTime now = rtc.now();
    reply = "Sensors ";
    reply += formatDateTime(now);
    reply += ": Temp ";
    reply += temperature;
    reply += "C, Humidity ";
    reply += humidity;
    reply += "%, Battery ";
    reply += batteryVoltage;
    reply += "V";

  } else if (command == "TIME") {
    reply = "Current time: ";
    reply += formatDateTime(rtc.now());

  } else if (command.startsWith("SET TIME ")) {
    String timeStr = command.substring(9);
    if (parseAndSetTime(timeStr)) {
      reply = "Time set to: ";
      reply += formatDateTime(rtc.now());
    } else {
      reply = "Invalid format. Use: SET TIME YYYY-MM-DD HH:MM:SS";
    }

  } else if (command == "RESET ALL") {
    lowVoltageAlertSent = false;
    highPumpAlertSent = false;
    highTempAlertSent = false;
    highHumidityAlertSent = false;
    pumpCount = 0;
    reply = "All alerts reset. Monitoring resumed.";

  } else if (command == "RESET VOLTAGE" || command == "RESET BATTERY") {
    lowVoltageAlertSent = false;
    reply = "Battery alert reset.";

  } else if (command == "RESET PUMP") {
    highPumpAlertSent = false;
    pumpCount = 0;
    reply = "Pump alert reset. Counter cleared.";

  } else if (command == "RESET TEMP") {
    highTempAlertSent = false;
    reply = "Temperature alert reset.";

  } else if (command == "RESET HUMIDITY") {
    highHumidityAlertSent = false;
    reply = "Humidity alert reset.";

  } else if (command == "TEST VOLTAGE") {
    lowVoltageAlertSent = false;
    batteryVoltage = LOW_VOLTAGE_THRESHOLD - 0.1;
    checkBatteryVoltage();
    reply = "Voltage test alert sent.";

  } else if (command == "TEST PUMP") {
    highPumpAlertSent = false;
    pumpCount = PUMP_THRESHOLD + 1;
    checkPumpThreshold();
    reply = "Pump test alert sent.";

  } else if (command == "TEST TEMP") {
    highTempAlertSent = false;
    temperature = HIGH_TEMP_THRESHOLD + 1.0;
    checkTemperatureHumidity();
    reply = "Temperature test alert sent.";

  } else if (command == "TEST HUMIDITY") {
    highHumidityAlertSent = false;
    humidity = HIGH_HUMIDITY_THRESHOLD + 1.0;
    checkTemperatureHumidity();
    reply = "Humidity test alert sent.";

  } else if (command == "GUARD") {
    reply = guardStatusText();

  } else if (command == "GUARD LOG") {
    guard.setEnforce(false);
    reply = "Guard LOG-ONLY: pump will not be cut. Returns to the programmed default after a reboot.";

  } else if (command == "GUARD ENFORCE") {
    guard.setEnforce(true);
    reply = "Guard ENFORCING: pump is cut if it runs " + fmtDuration(guard.runLimitMs()) + " nonstop.";

  } else if (command == "GUARD CLEAR") {
    guard.clear(millis());
    reply = "Guard cleared: pump released, trip count reset.";

  } else if (command.startsWith("GUARD LIMIT ")) {
    long secs = command.substring(12).toInt();
    if (secs >= 30 && secs <= 3600) {
      guard.setRunLimitMs((uint32_t)secs * 1000UL);
      reply = "Run limit set to " + String(secs) + "s until next reboot.";
    } else {
      reply = "Limit must be 30 to 3600 seconds.";
    }

  } else if (command == "HELP") {
    reply = "Commands: STATUS, SENSORS, TIME, SET TIME, RESET ALL/VOLTAGE/PUMP/TEMP/HUMIDITY, TEST VOLTAGE/PUMP/TEMP/HUMIDITY, GUARD [LOG/ENFORCE/CLEAR/LIMIT n], HELP";

  } else {
    reply = "Unknown command. Send HELP for list.";
  }

  if (reply.length() > 0) sendSMS(ALERT_PHONE, reply);
}


String formatDateTime(DateTime dt) {
  char buffer[20];
  sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d",
          dt.day(), dt.month(), dt.year(),
          dt.hour(), dt.minute(), dt.second());
  return String(buffer);
}


bool parseAndSetTime(String timeStr) {
  timeStr.trim();
  if (timeStr.length() != 19) return false;
  int year   = timeStr.substring(0, 4).toInt();
  int month  = timeStr.substring(5, 7).toInt();
  int day    = timeStr.substring(8, 10).toInt();
  int hour   = timeStr.substring(11, 13).toInt();
  int minute = timeStr.substring(14, 16).toInt();
  int second = timeStr.substring(17, 19).toInt();
  if (year < 2020 || year > 2050) return false;
  if (month < 1 || month > 12) return false;
  if (day < 1 || day > 31) return false;
  if (hour < 0 || hour > 23) return false;
  if (minute < 0 || minute > 59) return false;
  if (second < 0 || second > 59) return false;
  rtc.adjust(DateTime(year, month, day, hour, minute, second));
  SerialUSB.println("RTC set to: " + formatDateTime(rtc.now()));
  return true;
}


void checkAutomaticReports() {
  DateTime now = rtc.now();
  int currentHour = now.hour();

  // Morning report at 08:00
  // Currently disabled — uncomment the if() block to re-enable.
  // Was disabled due to reports firing unexpectedly during testing.
  // if (currentHour == 8 && !morningReportSent) {
    String message = "Morning Report ";
    message += formatDateTime(now);
    message += ": Battery ";
    message += batteryVoltage;
    message += "V, Temp ";
    message += temperature;
    message += "C, Humidity ";
    message += humidity;
    message += "%, Pump count: ";
    message += pumpCount;
  //  sendSMS(ALERT_PHONE, message);
  //  morningReportSent = true;
  //  SerialUSB.println("Morning report sent");
  // }

  // Evening report at 16:00
  // Currently disabled — uncomment the if() block to re-enable.
  // if (currentHour == 16 && !eveningReportSent) {
    String message2 = "Evening Report ";
    message2 += formatDateTime(now);
    message2 += ": Battery ";
    message2 += batteryVoltage;
    message2 += "V, Temp ";
    message2 += temperature;
    message2 += "C, Humidity ";
    message2 += humidity;
    message2 += "%, Pump count: ";
    message2 += pumpCount;
  //  sendSMS(ALERT_PHONE, message2);
  //  eveningReportSent = true;
  //  SerialUSB.println("Evening report sent");
  // }

  // Reset daily flags at midnight
  if (currentHour == 0) {
    morningReportSent = false;
    eveningReportSent = false;
  }
}

void updateOLEDDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Line 1: Title, with the pump guard state at the right (OK / RUN / CUT, -L = log-only)
  display.setCursor(0, 0);
  display.println("BILGE MONITOR");
  display.setCursor(84, 0);
  display.print(guardShort());

  // Line 2: Battery voltage (large text)
  display.setTextSize(2);
  display.setCursor(0, 12);
  display.print(batteryVoltage, 1);
  display.println(" V");

  // Line 3: Pump count (large text)
  display.setCursor(0, 30);
  display.print("Pump: ");
  display.println(pumpCount);

  // Line 4-5: Temperature and Humidity (small text)
  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print("T:");
  display.print(temperature, 0);
  display.print("C ");
  display.print("H:");
  display.print(humidity, 0);
  display.println("%");

  // Line 6: Time
  DateTime now = rtc.now();
  display.setCursor(0, 56);
  char timeStr[12];
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  display.println(timeStr);

  display.display();
}
