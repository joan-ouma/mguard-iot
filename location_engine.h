#pragma once
#include <Arduino.h>
#include <math.h>
#include <Preferences.h>
#include "config.h"
#include "modem.h"

struct LocationResult {
  double lat = 0.0;
  double lon = 0.0;
  float speedKmh = 0.0;
  float headingDeg = 0.0;
  String locType = "none";
};

struct NeighborCell {
  uint32_t lac = 0;
  uint32_t cid = 0;
  int16_t  dbm = -115;
};

struct CellTowerInfo {
  uint16_t mcc = 639;
  uint8_t  mnc = 2;
  uint32_t lac = 0;
  uint32_t cid = 0;
  int16_t  csq = 0;
  int16_t  rssiDbm = -115;
  uint8_t  neighborCount = 0;
  NeighborCell neighbors[7];
};

namespace location {

inline CellTowerInfo  tower;
inline LocationResult cached;
inline float headingDeg_ = 0.0f;
inline float speedMps_   = 0.0f;
inline uint32_t lastNvsSaveMs = 0;

inline float speedKmh() { return speedMps_ * 3.6f; }
inline float heading()  { return headingDeg_; }
inline void  setCachedPosition(double lat, double lon) { cached.lat = lat; cached.lon = lon; }

inline int readBatteryPercent() {
  String resp = modem::sendAT("AT+CBC", 1500);
  int idx = resp.indexOf("+CBC:");
  if (idx == -1) return 0;

  int firstComma = resp.indexOf(',', idx);
  int secondComma = resp.indexOf(',', firstComma + 1);
  if (firstComma == -1 || secondComma == -1) return 0;

  String val = resp.substring(secondComma + 1);
  int end = val.indexOf('\r');
  if (end != -1) val = val.substring(0, end);
  val.trim();
  return val.toInt();
}

inline void saveToNvs(double lat, double lon) {
  uint32_t now = millis();
  if (now - lastNvsSaveMs < 30000 && lastNvsSaveMs != 0) return;
  lastNvsSaveMs = now;

  Preferences prefs;
  prefs.begin("motoloc", false);
  prefs.putDouble("lat", lat);
  prefs.putDouble("lon", lon);
  prefs.end();
}

inline void begin() {
  Preferences prefs;
  prefs.begin("motoloc", true);
  cached.lat = prefs.getDouble("lat", 0.0);
  cached.lon = prefs.getDouble("lon", 0.0);
  if (cached.lat != 0.0 && cached.lon != 0.0) {
    cached.locType = "last_known";
  }
  prefs.end();

  modem::sendAT("AT+CGNSPWR=1", 2000);
  modem::sendAT("AT+CGNSMOD=1,1,0,0");
  modem::sendAT("AT+CGNSEPO=1", 1000);
  modem::sendAT("AT+CENG=3,1", 500);
}

inline void attachBearer() {
  modem::sendAT("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  modem::sendAT(String("AT+SAPBR=3,1,\"APN\",\"") + CELLULAR_APN + "\"");
  modem::sendAT("AT+SAPBR=1,1", 5000);
}

inline bool tryGetGpsFix(LocationResult &loc) {
  String resp = modem::sendAT("AT+CGNSINF", 1000);
  int tagIdx = resp.indexOf("+CGNSINF: ");
  if (tagIdx == -1) {
    if (resp.indexOf("+CGNSINF: 0,") != -1) {
      modem::sendAT("AT+CGNSPWR=1", 1000);
    }
    speedMps_ = 0.0f;
    return false;
  }

  String data = resp.substring(tagIdx + 10);
  int endLine = data.indexOf('\r');
  if (endLine != -1) data = data.substring(0, endLine);

  int commas[8];
  int searchFrom = 0;
  for (int i = 0; i < 8; i++) {
    int pos = data.indexOf(',', searchFrom);
    if (pos == -1) {
      speedMps_ = 0.0f;
      return false;
    }
    commas[i] = pos;
    searchFrom = pos + 1;
  }

  int fixStatus = data.substring(commas[0] + 1, commas[1]).toInt();
  if (fixStatus != 1) {
    speedMps_ = 0.0f;
    return false;
  }

  double lat = data.substring(commas[2] + 1, commas[3]).toDouble();
  double lon = data.substring(commas[3] + 1, commas[4]).toDouble();
  if (lat == 0.0 || lon == 0.0 || fabs(lat) > 90.0 || fabs(lon) > 180.0) {
    speedMps_ = 0.0f;
    return false;
  }

  loc.lat = lat;
  loc.lon = lon;
  loc.locType = "gps";

  float gpsSpeedKmh = data.substring(commas[5] + 1, commas[6]).toFloat();
  if (gpsSpeedKmh <= 2.0f || gpsSpeedKmh > 200.0f) {
    speedMps_ = 0.0f;
  } else {
    float gpsCourse = data.substring(commas[6] + 1, commas[7]).toFloat();
    speedMps_   = gpsSpeedKmh / 3.6f;
    headingDeg_ = gpsCourse;
  }

  loc.speedKmh   = speedKmh();
  loc.headingDeg = heading();
  cached = loc;
  saveToNvs(lat, lon);
  return true;
}

inline bool tryGetLbsFix(LocationResult &loc) {
  speedMps_ = 0.0f;

  if (!modem::isNetworkRegistered()) return false;
  String resp = modem::sendAT("AT+CIPGSMLOC=1,1", 3000);
  int startIdx = resp.indexOf("+CIPGSMLOC: 0,");
  if (startIdx == -1) return false;

  int c1 = resp.indexOf(',', startIdx);
  if (c1 == -1) return false;
  int c2 = resp.indexOf(',', c1 + 1);
  if (c2 == -1) return false;
  int c3 = resp.indexOf(',', c2 + 1);
  if (c3 == -1) return false;

  double lon = resp.substring(c1 + 1, c2).toDouble();
  double lat = resp.substring(c2 + 1, c3).toDouble();

  if (fabs(lat) > 90.0 && fabs(lon) <= 90.0) {
    double tmp = lat; lat = lon; lon = tmp;
  }
  if (lat == 0.0 || lon == 0.0 || fabs(lat) > 90.0 || fabs(lon) > 180.0) return false;

  loc.lat = lat;
  loc.lon = lon;
  loc.locType = "lbs";
  loc.speedKmh = 0.0f;
  loc.headingDeg = headingDeg_;
  cached = loc;
  saveToNvs(lat, lon);
  return true;
}

inline void parseCellTowerId(const String &resp, CellTowerInfo &t) {
  int q1 = resp.indexOf('"');
  if (q1 == -1) return;
  int q2 = resp.indexOf('"', q1 + 1);
  if (q2 == -1) return;
  int q3 = resp.indexOf('"', q2 + 1);
  if (q3 == -1) return;
  int q4 = resp.indexOf('"', q3 + 1);
  if (q4 == -1) return;

  t.lac = strtoul(resp.substring(q1 + 1, q2).c_str(), NULL, 16);
  t.cid = strtoul(resp.substring(q3 + 1, q4).c_str(), NULL, 16);
}

inline void parseSignalStrength(const String &resp, CellTowerInfo &t) {
  int idx = resp.indexOf("+CSQ: ");
  if (idx == -1) return;

  int val = resp.substring(idx + 6).toInt();
  t.csq = val;
  t.rssiDbm = (val == 99 || val == 0) ? -115 : (-113 + (val * 2));
}

inline void parseCengNeighbors(const String &resp, CellTowerInfo &t) {
  t.neighborCount = 0;
  if (resp.length() == 0) return;

  String s = resp;
  s.replace("\r", " ");
  s.replace("\n", " ");

  int searchPos = 0;
  while (searchPos < s.length() && t.neighborCount < 7) {
    int start = s.indexOf("+CENG:", searchPos);
    if (start == -1) break;

    int end = s.indexOf(' ', start + 6);
    if (end == -1) end = s.length();
    String line = s.substring(start, end);

    int q1 = line.indexOf('"');
    if (q1 != -1) {
      int q2 = line.indexOf('"', q1 + 1);
      int q3 = line.indexOf('"', q2 + 1);
      int q4 = line.indexOf('"', q3 + 1);
      if (q2 != -1 && q3 != -1 && q4 != -1) {
        uint32_t lac = strtoul(line.substring(q1 + 1, q2).c_str(), NULL, 16);
        uint32_t cid = strtoul(line.substring(q3 + 1, q4).c_str(), NULL, 16);
        if (lac != 0 && cid != 0) {
          bool duplicate = false;
          for (uint8_t i = 0; i < t.neighborCount; ++i) {
            if (t.neighbors[i].lac == lac && t.neighbors[i].cid == cid) {
              duplicate = true;
              break;
            }
          }
          if (!duplicate) {
            t.neighbors[t.neighborCount].lac = lac;
            t.neighbors[t.neighborCount].cid = cid;
            t.neighbors[t.neighborCount].dbm = t.rssiDbm;
            t.neighborCount++;
          }
        }
      }
    }

    int values[16];
    int numCount = 0;
    int pos = 0;
    while (pos < line.length() && numCount < 16) {
      while (pos < line.length() && !isdigit((unsigned char)line.charAt(pos)) && line.charAt(pos) != '-' && line.charAt(pos) != '+') {
        pos++;
      }
      if (pos >= line.length()) break;
      int startVal = pos;
      while (pos < line.length() && (isdigit((unsigned char)line.charAt(pos)) || line.charAt(pos) == '-' || line.charAt(pos) == '+')) {
        pos++;
      }
      String token = line.substring(startVal, pos);
      if (token.length() > 0) {
        values[numCount++] = token.toInt();
      }
    }

    if (numCount >= 8) {
      for (int i = 3; i + 1 < numCount && t.neighborCount < 7; ++i) {
        uint32_t lac = (uint32_t)labs(values[i]);
        uint32_t cid = (uint32_t)labs(values[i + 1]);
        if (lac > 0 && cid > 0 && lac < 65535 && cid < 65535) {
          bool duplicate = false;
          for (uint8_t j = 0; j < t.neighborCount; ++j) {
            if (t.neighbors[j].lac == lac && t.neighbors[j].cid == cid) {
              duplicate = true;
              break;
            }
          }
          if (!duplicate) {
            t.neighbors[t.neighborCount].lac = lac;
            t.neighbors[t.neighborCount].cid = cid;
            t.neighbors[t.neighborCount].dbm = (int16_t)values[i + 2];
            t.neighborCount++;
          }
        }
      }
    }

    searchPos = start + 6;
  }
}

inline void updateServingCellTower() {
  String cengResp = modem::sendAT("AT+CENG?", 500);
  parseCengNeighbors(cengResp, tower);
  parseCellTowerId(modem::sendAT("AT+CREG?", 500), tower);
  parseSignalStrength(modem::sendAT("AT+CSQ", 500), tower);

  if (tower.lac == 0 && tower.cid == 0 && tower.neighborCount > 0) {
    tower.lac = tower.neighbors[0].lac;
    tower.cid = tower.neighbors[0].cid;
  }
}

inline LocationResult resolveBest() {
  LocationResult loc;

  if (tryGetGpsFix(loc)) {
    return loc;
  }

  speedMps_ = 0.0f;

  // Note: AT+CIPGSMLOC conflicts with active TCP socket in CIPMUX=0 mode.
  // Cell tower info (MCC, MNC, LAC, CID, CSQ) is provided in telemetry payload for backend resolution.
  // if (tryGetLbsFix(loc)) {
  //   loc.speedKmh = 0.0f;
  //   return loc;
  // }

  updateServingCellTower();

  if (cached.lat != 0.0 && cached.lon != 0.0) {
    loc = cached;
    loc.locType = "last_known";
    loc.speedKmh = 0.0f;
    return loc;
  }

  loc.lat = 0.0;
  loc.lon = 0.0;
  loc.speedKmh = 0.0f;
  loc.locType = "none";
  return loc;
}

}
