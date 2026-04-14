// BilgePumpMonitor
// Maduino Zero 4G (SIM7600E) - Bilge Pump and Battery Monitor
// Monitors bilge pump cycles, battery voltage, temperature and humidity.
// Sends SMS alerts when thresholds are exceeded.
// Responds to SMS commands for remote status and control.
//
// Hardware:
//   - Maduino Zero 4G (SIM7600E LTE module)
//   - DS3231 RTC module (I2C)
//   - DHT11 temperature/humidity sensor
//   - DFRobot 25V voltage sensor on A0
//   - Bilge pump signal on pin 10 (active LOW)
//
// Configuration:
//   Copy config.h.example to config.h and fill in your details.
//   See README.md for full setup instructions.

#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>
#include "config.h"

// Pin definitions for Maduino Zero 4G (LTE)
#define LTE_RESET_PIN    6
#define LTE_PWRKEY_PIN   5
#define LTE_FLIGHT_PIN   7

// Monitoring pins
#define PUMP_INPUT_PIN   10   // Active LOW pump signal (confirmed working)
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

// Automatic status report flags
// Note: morning and evening reports are implemented but currently disabled.
// To re-enable, uncomment the if() blocks in checkAutomaticReports().
bool morningReportSent = false;
bool eveningReportSent = false;

// SMS handling
String modemBuffer = "";
String lastSender = "";
String pendingDeleteIndex = "";

// Hardware objects
RTC_DS3231 rtc;
DHT dht(DHT_PIN, DHT_TYPE);


void setup() {
  SerialUSB.begin(115200);
  delay(100);
  Serial1.begin(115200);

  pinMode(PUMP_INPUT_PIN, INPUT_PULLUP);
  pinMode(BATTERY_PIN, INPUT);

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

  DateTime startTime = rtc.now();
  String startupMsg = "Bilge monitor started at ";
  startupMsg += formatDateTime(startTime);
  startupMsg += ". Monitoring pump, battery, temp & humidity.";
  sendSMS(ALERT_PHONE, startupMsg);

  lastCheckTime = millis();
  lastResetTime = millis();
}


void loop() {
  handleModemCommunication();
  checkPumpState();

  if (millis() - lastCheckTime >= CHECK_INTERVAL) {
    checkBatteryVoltage();
    checkTemperatureHumidity();
    checkPumpThreshold();
    checkAutomaticReports();
    lastCheckTime = millis();

    SerialUSB.print("Status — Pump: "); SerialUSB.print(pumpCount);
    SerialUSB.print(", Battery: "); SerialUSB.print(batteryVoltage); SerialUSB.print("V");
    SerialUSB.print(", Temp: "); SerialUSB.print(temperature); SerialUSB.print("°C");
    SerialUSB.print(", Humidity: "); SerialUSB.print(humidity); SerialUSB.println("%");
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


void checkPumpState() {
  bool currentPumpState = !digitalRead(PUMP_INPUT_PIN); // Active LOW
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
  delay(1000);
  Serial1.print(message);
  Serial1.write(26); // Ctrl+Z
  delay(5000);
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
    reply += "%";

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

  } else if (command == "HELP") {
    reply = "Commands: STATUS, SENSORS, TIME, SET TIME, RESET ALL/VOLTAGE/PUMP/TEMP/HUMIDITY, TEST VOLTAGE/PUMP/TEMP/HUMIDITY, HELP";

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