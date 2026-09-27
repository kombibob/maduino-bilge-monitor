#include "sensesp_app_builder.h"
#include "sensesp/sensors/sensor.h"

using namespace sensesp;

float bilge_voltage = 0;
float bilge_cycles = 0;
float bilge_temp = 0;
float bilge_humidity = 0;

String serial_line = "";

void setup() {
  SetupLogging();

  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  SensESPAppBuilder builder;
  builder.set_hostname("bilge-esp32")->set_wifi_access_point("bilge-esp32", "bilgesetup")->set_sk_server("10.48.10.1", 3000)->get_app();

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
}

void loop() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c != '\n') {
      serial_line += c;
      continue;
    }
    float v, n, t, h;
    if (sscanf(serial_line.c_str(), "$BILGE,%f,%f,%f,%f", &v, &n, &t, &h) == 4) {
      bilge_voltage = v;
      bilge_cycles = n;
      bilge_temp = t + 273.15f;
      bilge_humidity = h / 100.0f;
    }
    serial_line = "";
  }
  event_loop()->tick();
}