#include "sensesp_app_builder.h"
#include "sensesp/sensors/sensor.h"

using namespace sensesp;

// ---- $BILGE line (every 60 s from the Maduino) ----
float bilge_voltage = 0;
float bilge_cycles = 0;
float bilge_temp = 0;
float bilge_humidity = 0;

// ---- $PUMP line (on change and every 10 s from the Maduino) ----
// $PUMP,state,cut,demand,feed,enforce,trips,lastRunS,maxRunS,faultMask,lockoutLeftS
int  pump_state = 0;          // 0 idle, 1 running, 2 lockout
bool pump_cut = false;        // K1 energised, pump held off
bool pump_demand = false;     // Water Witch calling for the pump
bool pump_feed = false;       // voltage actually present at the pump
bool pump_enforce = false;    // false = log-only
int  pump_trips = 0;
float pump_last_run_s = 0;
float pump_max_run_s = 0;
int  pump_fault_mask = 0;     // 1 feed dead, 2 live while cut, 4 live with no demand
float pump_lockout_left_s = 0;
unsigned long last_pump_line_ms = 0;
bool pump_line_seen = false;

// The link is one-way, so silence is the only sign the Maduino has died.
// It sends a $PUMP line at least every 10 s, so 35 s of nothing = link lost.
const unsigned long PUMP_LINK_TIMEOUT_MS = 35000;

String serial_line = "";

void parse_line(const String& line) {
  float v, n, t, h;
  if (sscanf(line.c_str(), "$BILGE,%f,%f,%f,%f", &v, &n, &t, &h) == 4) {
    bilge_voltage = v;
    bilge_cycles = n;
    bilge_temp = t + 273.15f;
    bilge_humidity = h / 100.0f;
    return;
  }

  int st, cut, dem, feed, enf, trips, fault;
  float last_run, max_run, lock_left;
  if (sscanf(line.c_str(), "$PUMP,%d,%d,%d,%d,%d,%d,%f,%f,%d,%f",
             &st, &cut, &dem, &feed, &enf, &trips,
             &last_run, &max_run, &fault, &lock_left) == 10) {
    pump_state = st;
    pump_cut = cut != 0;
    pump_demand = dem != 0;
    pump_feed = feed != 0;
    pump_enforce = enf != 0;
    pump_trips = trips;
    pump_last_run_s = last_run;
    pump_max_run_s = max_run;
    pump_fault_mask = fault;
    pump_lockout_left_s = lock_left;
    last_pump_line_ms = millis();
    pump_line_seen = true;
  }
}

void setup() {
  SetupLogging();

  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  SensESPAppBuilder builder;
  builder.set_hostname("bilge-esp32")->set_wifi_access_point("bilge-esp32", "bilgesetup")->set_sk_server("10.48.10.1", 3000)->get_app();

  // ---------- existing $BILGE paths (unchanged) ----------
  auto* voltage_sensor = new RepeatSensor<float>(2000, []() { return bilge_voltage; });
  voltage_sensor->connect_to(new SKOutputFloat(
      "electrical.batteries.bilge.voltage",
      new SKMetadata("V", "Bilge Battery Voltage",
                     "Battery voltage at the bilge monitor")));

  auto* cycles_sensor = new RepeatSensor<float>(2000, []() { return bilge_cycles; });
  cycles_sensor->connect_to(new SKOutputFloat(
      "electrical.pumps.bilge.cycles",
      new SKMetadata("", "Bilge Pump Cycles",
                     "Number of bilge pump activations in the current reset period")));

  auto* temp_sensor = new RepeatSensor<float>(5000, []() { return bilge_temp; });
  temp_sensor->connect_to(new SKOutputFloat(
      "environment.inside.bilge.temperature",
      new SKMetadata("K", "Bilge Temperature",
                     "Temperature inside the bilge compartment")));

  auto* humidity_sensor = new RepeatSensor<float>(5000, []() { return bilge_humidity; });
  humidity_sensor->connect_to(new SKOutputFloat(
      "environment.inside.bilge.relativeHumidity",
      new SKMetadata("ratio", "Bilge Humidity",
                     "Relative humidity inside the bilge compartment")));

  // ---------- new pump guard paths ----------
  auto* state_sensor = new RepeatSensor<String>(2000, []() {
    switch (pump_state) {
      case 1: return String("running");
      case 2: return String("lockout");
      default: return String("idle");
    }
  });
  state_sensor->connect_to(new SKOutputString(
      "electrical.pumps.bilge.guard.state",
      new SKMetadata("", "Bilge Guard State",
                     "idle, running or lockout (pump held off)")));

  auto* cut_sensor = new RepeatSensor<bool>(2000, []() { return pump_cut; });
  cut_sensor->connect_to(new SKOutputBool(
      "electrical.pumps.bilge.guard.cut",
      new SKMetadata("", "Bilge Pump Cut",
                     "True while the guard is holding the pump off")));

  auto* demand_sensor = new RepeatSensor<bool>(2000, []() { return pump_demand; });
  demand_sensor->connect_to(new SKOutputBool(
      "electrical.pumps.bilge.guard.demand",
      new SKMetadata("", "Bilge Pump Demand",
                     "True while the Water Witch is calling for the pump")));

  auto* feed_sensor = new RepeatSensor<bool>(2000, []() { return pump_feed; });
  feed_sensor->connect_to(new SKOutputBool(
      "electrical.pumps.bilge.guard.feed",
      new SKMetadata("", "Bilge Pump Feed",
                     "True while voltage is present at the pump")));

  auto* enforce_sensor = new RepeatSensor<bool>(5000, []() { return pump_enforce; });
  enforce_sensor->connect_to(new SKOutputBool(
      "electrical.pumps.bilge.guard.enforcing",
      new SKMetadata("", "Bilge Guard Enforcing",
                     "True = guard will cut the pump, false = log-only")));

  auto* trips_sensor = new RepeatSensor<int>(5000, []() { return pump_trips; });
  trips_sensor->connect_to(new SKOutputInt(
      "electrical.pumps.bilge.guard.trips",
      new SKMetadata("", "Bilge Guard Trips",
                     "Consecutive trips since the pump last ran normally")));

  auto* last_run_sensor = new RepeatSensor<float>(5000, []() { return pump_last_run_s; });
  last_run_sensor->connect_to(new SKOutputFloat(
      "electrical.pumps.bilge.guard.lastRun",
      new SKMetadata("s", "Bilge Pump Last Run",
                     "Length of the last completed pump run")));

  auto* max_run_sensor = new RepeatSensor<float>(5000, []() { return pump_max_run_s; });
  max_run_sensor->connect_to(new SKOutputFloat(
      "electrical.pumps.bilge.guard.maxRun",
      new SKMetadata("s", "Bilge Pump Longest Run",
                     "Longest pump run since the Maduino last rebooted")));

  auto* lockout_sensor = new RepeatSensor<float>(2000, []() { return pump_lockout_left_s; });
  lockout_sensor->connect_to(new SKOutputFloat(
      "electrical.pumps.bilge.guard.lockoutRemaining",
      new SKMetadata("s", "Bilge Lockout Remaining",
                     "Time left before the pump is released for a retry")));

  auto* fault_sensor = new RepeatSensor<int>(2000, []() { return pump_fault_mask; });
  fault_sensor->connect_to(new SKOutputInt(
      "electrical.pumps.bilge.guard.faultMask",
      new SKMetadata("", "Bilge Guard Fault Mask",
                     "Bit 0 feed dead, bit 1 live while cut, bit 2 live with no demand")));

  auto* link_sensor = new RepeatSensor<bool>(5000, []() {
    return pump_line_seen && (millis() - last_pump_line_ms) < PUMP_LINK_TIMEOUT_MS;
  });
  link_sensor->connect_to(new SKOutputBool(
      "electrical.pumps.bilge.guard.linkOk",
      new SKMetadata("", "Bilge Monitor Link",
                     "False if no $PUMP line has arrived for 35 s (Maduino down or link broken)")));
}

void loop() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (serial_line.length() < 120) serial_line += c;   // cap: line noise must not grow forever
      continue;
    }
    parse_line(serial_line);
    serial_line = "";
  }
  event_loop()->tick();
}
