#pragma once
#include <Arduino.h>
#include "modem.h"
#include "ignition.h"
#include "rfid_auth.h"

namespace console {

inline void handle() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.equalsIgnoreCase("I1")) {
    ignition::setOverride(1);
    Serial.println("\n[IGNITION] Key Turned ON (Simulated: Force ON)");
    return;
  }
  if (line.equalsIgnoreCase("I0")) {
    ignition::setOverride(0);
    Serial.println("\n[IGNITION] Key Turned OFF (Simulated: Force OFF)");
    return;
  }
  if (line.equalsIgnoreCase("IA")) {
    ignition::setOverride(-1);
    Serial.println("\n[IGNITION] Reverted to Hardware Sensing (GPIO 34)");
    return;
  }

  if (line.startsWith("RFID_ADD ") || line.startsWith("rfid_add ")) {
    String tag = line.substring(9);
    tag.trim();
    tag.toUpperCase();
    rfid::addAuthorizedTag(tag);
    return;
  }

  if (line.startsWith("RFID_DEL ") || line.startsWith("rfid_del ")) {
    String tag = line.substring(9);
    tag.trim();
    tag.toUpperCase();
    rfid::revokeTag(tag);
    return;
  }

  if (line.startsWith("RFID_MASTER ") || line.startsWith("rfid_master ")) {
    String tag = line.substring(12);
    tag.trim();
    tag.toUpperCase();
    rfid::setMasterTag(tag);
    return;
  }

  if (line.equalsIgnoreCase("RFID_LIST")) {
    rfid::listAuthorizedTags();
    return;
  }

  if (line.equalsIgnoreCase("RFID_LEARN")) {
    rfid::learnMode = true;
    Serial.println("\n[RFID LEARN MODE ACTIVE] Tap any card/fob to the reader now to permanently save it...");
    return;
  }

  if (line.equalsIgnoreCase("RFID_CLR")) {
    rfid::clearAllTags();
    return;
  }

  if (line.equalsIgnoreCase("RFID_STATUS")) {
    rfid::diagnostics();
    return;
  }

  if (line.startsWith("*") && line.endsWith("#")) {
    Serial.println("\n[USSD] Dialing: " + line + " ...");
    String resp = modem::runUSSD(line);
    Serial.println("[USSD] " + resp);
    return;
  }

  if (line.startsWith("U*") || line.startsWith("u*")) {
    String code = line.substring(1);
    Serial.println("\n[USSD] Dialing: " + code + " ...");
    String resp = modem::runUSSD(code);
    Serial.println("[USSD] " + resp);
    return;
  }

  if (line.startsWith("USSD ") || line.startsWith("ussd ")) {
    String code = line.substring(5);
    code.trim();
    Serial.println("\n[USSD] Dialing: " + code + " ...");
    String resp = modem::runUSSD(code);
    Serial.println("[USSD] " + resp);
    return;
  }

  if (line.length() > 0) {
    modem::sim868.println(line);
  }
}

inline void streamModemPassthrough() {
  while (modem::sim868.available()) {
    Serial.write(modem::sim868.read());
  }
}

inline void process() {
  streamModemPassthrough();
  handle();
}

}

inline void checkSerialCommands() { console::handle(); }
