#pragma once
#include <Arduino.h>
#include <stdio.h>
#include <Preferences.h>
#include "config.h"
#include "modem.h"
#include "location_engine.h"
#include "guard_engine.h"
#include "ignition.h"
#include "rfid_auth.h"
#include "mqtt_client.h"

namespace report {

inline uint32_t lastPublishMs   = 0;
inline uint32_t lastMotionMs    = 0;
inline uint32_t alertCount      = 0;
inline String   lastAlertReason = "none";

inline void init() {
  Preferences prefs;
  prefs.begin("motorep", true);
  alertCount = prefs.getUInt("alerts", 0);
  prefs.end();
}

inline String serializeAlert(const LocationResult &loc, const String &reason) {
  char buf[448];
  snprintf(buf, sizeof(buf),
    "{\"alert\":true,\"reason\":\"%s\",\"lat\":%.6f,\"lng\":%.6f,\"loc_type\":\"%s\","
    "\"csq\":%d,\"rssi_dbm\":%d,\"spd\":%.1f,\"hdg\":%.1f,\"ign\":%s,\"auth\":%s,\"tag\":\"%s\"}",
    reason.c_str(),
    loc.lat,
    loc.lon,
    loc.locType.c_str(),
    location::tower.csq,
    location::tower.rssiDbm,
    loc.speedKmh,
    loc.headingDeg,
    ignition::isIgnitionOn() ? "true" : "false",
    rfid::isSessionAuthenticated() ? "true" : "false",
    rfid::getActiveTag().c_str()
  );
  return String(buf);
}

inline String serializeTelemetry(const LocationResult &loc) {
  char buf[448];
  snprintf(buf, sizeof(buf),
    "{\"lat\":%.6f,\"lng\":%.6f,\"loc_type\":\"%s\",\"mcc\":%u,\"mnc\":%u,\"lac\":%lu,\"cid\":%lu,"
    "\"csq\":%d,\"rssi_dbm\":%d,\"speed\":%.1f,\"heading\":%.1f,\"ign\":%s,\"auth\":%s,\"tag\":\"%s\"}",
    loc.lat,
    loc.lon,
    loc.locType.c_str(),
    location::tower.mcc,
    location::tower.mnc,
    (unsigned long)location::tower.lac,
    (unsigned long)location::tower.cid,
    location::tower.csq,
    location::tower.rssiDbm,
    loc.speedKmh,
    loc.headingDeg,
    ignition::isIgnitionOn() ? "true" : "false",
    rfid::isSessionAuthenticated() ? "true" : "false",
    rfid::getActiveTag().c_str()
  );
  return String(buf);
}

inline bool dispatchAlert(const String &reason) {
  alertCount++;
  lastAlertReason = reason;

  Preferences prefs;
  prefs.begin("motorep", false);
  prefs.putUInt("alerts", alertCount);
  prefs.end();

  LocationResult loc = location::resolveBest();

  String payload = serializeAlert(loc, reason);
  Serial.println("\n[ALERT] " + payload);
  return mqtt::publishAlert(payload);
}

inline bool dispatchTelemetry() {
  LocationResult loc = location::resolveBest();

  String payload = serializeTelemetry(loc);
  Serial.println("\n[DATA] " + payload);
  return mqtt::publishTelemetry(payload);
}

inline void printGentleStatusReport() {
  String csqResp = modem::sendAT("AT+CSQ", 300);
  int csq = 0;
  int csqIdx = csqResp.indexOf("+CSQ: ");
  if (csqIdx != -1) csq = csqResp.substring(csqIdx + 6).toInt();
  int rssiDbm = (csq == 99 || csq == 0) ? -115 : (-113 + (csq * 2));

  String cregResp = modem::sendAT("AT+CREG?", 300);
  String cregStatus = "Idle (0,0)";
  if (cregResp.indexOf(",1") != -1) cregStatus = "Registered (Home)";
  if (cregResp.indexOf(",2") != -1) cregStatus = "Searching...";
  if (cregResp.indexOf(",3") != -1) cregStatus = "Registration Denied";
  if (cregResp.indexOf(",5") != -1) cregStatus = "Registered (Roaming)";

  String gpsResp = modem::sendAT("AT+CGNSINF", 300);
  String gpsStatus = (gpsResp.indexOf("+CGNSINF: 1,1,") != -1) ? "Locked (3D Fix)" : "Searching (No Fix)";

  Serial.println("[STATUS] Signal: CSQ " + String(csq) + " (" + String(rssiDbm) + " dBm) | Cellular: " + cregStatus + " | GPS: " + gpsStatus);
}

inline uint32_t getCurrentIntervalMs() {
  if (ignition::isOverridden()) {
    return TELEMETRY_INTERVAL_BENCH_MS;
  }
  if (guard::state == guard::ALARM) {
    return TELEMETRY_INTERVAL_ALARM_MS;
  }
  if (ignition::isIgnitionOn()) {
    return TELEMETRY_INTERVAL_DRIVING_MS;
  }
  return TELEMETRY_INTERVAL_ARMED_MS;
}

inline void processPeriodicTelemetry(unsigned long now) {
  uint32_t interval = getCurrentIntervalMs();
  if (lastPublishMs != 0 && (now - lastPublishMs < interval)) return;
  lastPublishMs = now;

  if (!modem::isNetworkRegistered()) {
    printGentleStatusReport();
    return;
  }

  dispatchTelemetry();
}

}

inline String serializeAlert(const LocationResult &loc, const String &reason) { return report::serializeAlert(loc, reason); }
inline String serializeTelemetry(const LocationResult &loc) { return report::serializeTelemetry(loc); }
inline bool dispatchAlert(const String &reason) { return report::dispatchAlert(reason); }
inline bool dispatchTelemetry() { return report::dispatchTelemetry(); }
inline void printGentleStatusReport() { report::printGentleStatusReport(); }
inline void processPeriodicTelemetry(unsigned long now) { report::processPeriodicTelemetry(now); }
