#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Preferences.h>
#include "config.h"
#include "guard_engine.h"
#include "ignition.h"

namespace rfid {

inline MFRC522 mfrc522(PIN_RC522_SDA, PIN_RC522_RST);
inline Preferences prefs;

inline String masterUid = "A3 B2 C1 D0";

// Session authentication status
inline volatile bool sessionAuthenticated = false;
inline String authenticatedTag = "";
inline String lastScannedTag = "";
inline uint32_t lastScanMs = 0;
inline bool learnMode = false;
constexpr uint32_t SCAN_COOLDOWN_MS = 2500;

inline bool isSessionAuthenticated() {
  return sessionAuthenticated;
}

inline void clearSession() {
  sessionAuthenticated = false;
  authenticatedTag = "";
}

inline String getActiveTag() {
  if (authenticatedTag.length() > 0) return authenticatedTag;
  return lastScannedTag;
}

inline String getLastScannedTag() {
  return lastScannedTag;
}

// Convert raw byte array to uppercase hex string e.g. "A3 B2 C1 D0"
inline String uidToString(MFRC522::Uid *uid) {
  String s = "";
  for (byte i = 0; i < uid->size; i++) {
    if (i > 0) s += " ";
    if (uid->uidByte[i] < 0x10) s += "0";
    s += String(uid->uidByte[i], HEX);
  }
  s.toUpperCase();
  return s;
}

// Normalize tag by removing spaces, dashes, colons and uppercasing: "B7 99 36 06" -> "B7993606"
inline String normalizeUid(String s) {
  s.replace(" ", "");
  s.replace(":", "");
  s.replace("-", "");
  s.toUpperCase();
  s.trim();
  return s;
}

inline String getStoredTags() {
  prefs.begin("motopf", true);
  String tags = prefs.getString("tags", "");
  prefs.end();
  return tags;
}

inline void saveStoredTags(const String& tags) {
  prefs.begin("motopf", false);
  prefs.putString("tags", tags);
  prefs.end();
}

// Check if a UID is in master config or stored in NVS Preferences
inline bool isAuthorized(const String& uid) {
  String norm = normalizeUid(uid);
  if (norm.length() == 0) return false;

  if (norm.equalsIgnoreCase(normalizeUid(masterUid))) return true;

  String stored = getStoredTags();
  String needle = "," + norm + ",";
  return (stored.indexOf(needle) != -1);
}

// Add a tag permanently to NVS
inline void addAuthorizedTag(const String& uid) {
  String norm = normalizeUid(uid);
  if (norm.length() == 0) return;

  String stored = getStoredTags();
  if (stored.length() == 0) {
    stored = ",";
  }
  String needle = "," + norm + ",";
  if (stored.indexOf(needle) == -1) {
    stored += norm + ",";
    saveStoredTags(stored);
    Serial.printf("[RFID] Tag [%s] permanently saved to NVS storage!\n", norm.c_str());
  } else {
    Serial.printf("[RFID] Tag [%s] is already authorized in NVS.\n", norm.c_str());
  }
}

// Revoke a tag permanently from NVS
inline void revokeTag(const String& uid) {
  String norm = normalizeUid(uid);
  String stored = getStoredTags();
  String needle = "," + norm + ",";
  if (stored.indexOf(needle) != -1) {
    stored.replace(needle, ",");
    saveStoredTags(stored);
    Serial.printf("[RFID] Tag [%s] permanently removed from NVS.\n", norm.c_str());
  } else {
    Serial.printf("[RFID] Tag [%s] not found in NVS.\n", norm.c_str());
  }
}

// Clear all registered tags from NVS
inline void clearAllTags() {
  saveStoredTags(",");
  Serial.println("[RFID] All dynamic tags cleared from NVS.");
}

// List all registered tags from NVS
inline void listAuthorizedTags() {
  Serial.println("\n--- [PERSISTENT AUTHORIZED TAGS IN NVS] ---");
  Serial.printf("  Master Tag: [%s]\n", masterUid.c_str());
  String stored = getStoredTags();
  int count = 0;
  int start = 1;
  while (start < stored.length()) {
    int nextComma = stored.indexOf(',', start);
    if (nextComma == -1) break;
    String t = stored.substring(start, nextComma);
    count++;
    Serial.printf("  Tag #%d: [%s]\n", count, t.c_str());
    start = nextComma + 1;
  }
  if (count == 0) {
    Serial.println("  (No dynamic tags registered yet)");
  }
  Serial.println("-------------------------------------------");
}

// Set master tag permanently
inline void setMasterTag(const String& uid) {
  masterUid = uid;
  masterUid.toUpperCase();
  prefs.begin("motopf", false);
  prefs.putString("master", masterUid);
  prefs.end();
  Serial.println("[RFID] Master tag updated and saved: " + masterUid);
}

// Called upon successful card authentication
inline void onAuthSuccess(const String& uid) {
  sessionAuthenticated = true;
  authenticatedTag = uid;
  lastScannedTag = uid;
  Serial.println("\n[RFID] >>> AUTH GRANTED <<< Tag: [" + uid + "]");

  if (guard::state == guard::ARMED || guard::state == guard::ALARM) {
    guard::disarm();
    Serial.println("[GUARD] Disarmed by RFID authentication.");
    guard::emit("rfid_disarmed", 0.0f, 0.0f);
  } else if (guard::state == guard::DISARMED) {
    // If already disarmed, tapping the card manually arms the bike immediately
    sessionAuthenticated = false;
    authenticatedTag = "";
    guard::armManual();
    Serial.println("[GUARD] Manually armed by RFID tag.");
    guard::emit("rfid_armed", 0.0f, 0.0f);
  }
}

// Called when an unknown card is tapped
inline void onAuthDenied(const String& uid) {
  lastScannedTag = uid;
  Serial.println("\n[RFID] >>> AUTH DENIED <<< Unknown tag: [" + uid + "]");

  // Always emit an alert event on unauthorized card presentation
  guard::emit("rfid_unauthorized_scan", 4.0f, 0.0f);
}

inline void diagnostics() {
  byte v = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  Serial.println("\n--- [RFID HARDWARE DIAGNOSTICS] ---");
  Serial.printf("  SPI Pins: SCK=%d, MISO=%d, MOSI=%d, SS/SDA=%d, RST=%d\n",
                PIN_RC522_SCK, PIN_RC522_MISO, PIN_RC522_MOSI, PIN_RC522_SDA, PIN_RC522_RST);
  Serial.printf("  Firmware Version Register: 0x%02X\n", v);
  if (v == 0x91) {
    Serial.println("  Status: [OK] NXP RC522 v1.0 Detected.");
  } else if (v == 0x92) {
    Serial.println("  Status: [OK] NXP RC522 v2.0 Detected.");
  } else if (v == 0x12) {
    Serial.println("  Status: [OK] Counterfeit/Clone RC522 Detected (Functional).");
  } else if (v == 0x00 || v == 0xFF) {
    Serial.println("  Status: [FAILED] No SPI communication! Check wiring and 3.3V power.");
  } else {
    Serial.printf("  Status: Unknown chip version (0x%02X).\n", v);
  }
  Serial.printf("  Master Tag: [%s]\n", masterUid.c_str());
  Serial.printf("  Auth Session: %s\n", sessionAuthenticated ? "AUTHENTICATED" : "LOCKED");
  Serial.println("-----------------------------------");
}

inline void begin() {
  pinMode(PIN_RC522_RST, OUTPUT);
  digitalWrite(PIN_RC522_RST, HIGH);
  pinMode(PIN_RC522_SDA, OUTPUT);
  digitalWrite(PIN_RC522_SDA, HIGH);

  // Initialize ESP32 SPI with configured custom pins
  SPI.begin(PIN_RC522_SCK, PIN_RC522_MISO, PIN_RC522_MOSI, PIN_RC522_SDA);
  delay(10);
  mfrc522.PCD_Init();
  delay(10);

  // Re-assert SPI bus pins in case PCD_Init() called SPI.begin() with defaults
  SPI.begin(PIN_RC522_SCK, PIN_RC522_MISO, PIN_RC522_MOSI, PIN_RC522_SDA);

  // Set maximum antenna receiver gain for maximum sensitivity / range
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);

  // Load master tag if previously stored
  prefs.begin("motopf", true);
  String storedMaster = prefs.getString("master", "");
  prefs.end();
  if (storedMaster.length() > 0) {
    masterUid = storedMaster;
  }

  // Hook authentication check into guard engine
  guard::setAuthCheck(isSessionAuthenticated);

  diagnostics();
}

// Periodic non-blocking polling called in loop()
inline void process(unsigned long now) {
  if (guard::state == guard::ARMED || guard::state == guard::SETTLING) {
    sessionAuthenticated = false;
    authenticatedTag = "";
  }

  if (now - lastScanMs < SCAN_COOLDOWN_MS) return;

  // Try both RequestA (idle tags) and WakeupA (halted tags)
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  mfrc522.PCD_WriteRegister(mfrc522.TxModeReg, 0x00);
  mfrc522.PCD_WriteRegister(mfrc522.RxModeReg, 0x00);
  mfrc522.PCD_WriteRegister(mfrc522.ModWidthReg, 0x26);

  bool cardFound = (mfrc522.PICC_RequestA(bufferATQA, &bufferSize) == MFRC522::STATUS_OK) ||
                   (mfrc522.PICC_WakeupA(bufferATQA, &bufferSize) == MFRC522::STATUS_OK);

  if (!cardFound) {
    if (!mfrc522.PICC_IsNewCardPresent()) return;
  }

  if (!mfrc522.PICC_ReadCardSerial()) return;

  lastScanMs = now;
  String scannedUid = uidToString(&(mfrc522.uid));

  if (learnMode) {
    learnMode = false;
    Serial.println("\n[RFID LEARN] New card detected during learn mode!");
    addAuthorizedTag(scannedUid);
    onAuthSuccess(scannedUid);
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
    return;
  }

  if (isAuthorized(scannedUid)) {
    onAuthSuccess(scannedUid);
  } else {
    onAuthDenied(scannedUid);
  }

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

}
