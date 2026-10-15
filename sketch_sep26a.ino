#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "config.h"
#include "modem.h"
#include "location_engine.h"
#include "guard_engine.h"
#include "ignition.h"
#include "rfid_auth.h"
#include "mqtt_client.h"
#include "report.h"
#include "console.h"

void processGuardEvents(unsigned long now) {
  guard::Event ev;
  if (guard::poll(ev)) {
    Serial.println("\n[SECURITY EVENT] Reason: " + String(ev.reason));
    report::dispatchAlert(ev.reason);
    if (strcmp(ev.reason, "rfid_disarmed") == 0 || strcmp(ev.reason, "rfid_armed") == 0) {
      report::dispatchTelemetry();
    }
  }

  static uint32_t lastGpsMs = 0;
  if ((guard::state == guard::ARMED || guard::state == guard::ALARM) && (now - lastGpsMs > GPS_TOW_CHECK_MS)) {
    lastGpsMs = now;
    LocationResult l;
    if (location::tryGetGpsFix(l)) {
      guard::onGpsFix(l.lat, l.lon, location::speedKmh());
    }
  }
}

void setup() {
  Serial.begin(115200);

  ignition::begin();
  report::init();
  guard::begin(ignition::ignitionNow);
  rfid::begin();

  modem::begin();
  location::begin();
  modem::waitRegistered(20);
  location::attachBearer();
  mqtt::recoverTcpConnection();
}

void loop() {
  console::process();

  unsigned long now = millis();

  rfid::process(now);

  processGuardEvents(now);

  report::processPeriodicTelemetry(now);

  delay(10);
}