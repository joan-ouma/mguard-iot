#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"

namespace modem {

inline HardwareSerial sim868(2);

inline uint8_t consecutiveFails = 0;
inline bool    sleepActive      = false;

inline void hardReset();
inline void begin();
inline void powerOn();

inline bool isAtCommandDone(const String &resp, const String &waitFor) {
  if (waitFor.length() > 0) {
    return (resp.indexOf(waitFor) != -1 || resp.indexOf("ERROR") != -1 || resp.indexOf("FAIL") != -1);
  }
  return (resp.indexOf("OK") != -1 || resp.indexOf("ERROR") != -1 ||
          resp.indexOf("CONNECT") != -1 || resp.indexOf("FAIL") != -1);
}

inline void wake() {
  if (sleepActive) {
    sim868.print("AT\r\n");
    delay(50);
  }
}

inline String sendAT(const String &cmd, uint32_t timeoutMs = 2000, const String &waitFor = "") {
  wake();

  while (sim868.available()) sim868.read();
  sim868.print(cmd + "\r\n");

  String resp = "";
  uint32_t start = millis();
  bool success = false;

  while (millis() - start < timeoutMs) {
    while (sim868.available()) {
      resp += (char)sim868.read();
    }

    if (isAtCommandDone(resp, waitFor)) {
      delay(waitFor.length() > 0 ? 80 : 50);
      while (sim868.available()) resp += (char)sim868.read();
      success = (resp.indexOf("ERROR") == -1 && resp.indexOf("FAIL") == -1);
      break;
    }
    delay(10);
  }

  if (!success && (resp.length() == 0 || resp.indexOf("ERROR") != -1)) {
    consecutiveFails++;
    if (consecutiveFails >= MODEM_MAX_CONSECUTIVE_FAILS) {
      Serial.printf("[MODEM WATCHDOG] %d consecutive failures detected! Performing hardware reset...\n", consecutiveFails);
      hardReset();
      consecutiveFails = 0;
    }
  } else {
    consecutiveFails = 0;
  }

  return resp;
}

inline bool waitPrompt(uint32_t timeoutMs = 2500) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (sim868.available()) {
      if ((char)sim868.read() == '>') return true;
    }
    delay(5);
  }
  return false;
}

inline bool isNetworkRegistered() {
  String resp = sendAT("AT+CREG?", 500);
  return (resp.indexOf(",1") != -1 || resp.indexOf(",5") != -1);
}

inline void powerOn() {
  Serial.println("[HW] Booting SIM868 via PWRKEY...");
  pinMode(PIN_SIM868_PWRKEY, OUTPUT);
  digitalWrite(PIN_SIM868_PWRKEY, LOW);
  delay(1200);
  pinMode(PIN_SIM868_PWRKEY, INPUT);
  delay(5000);
}

inline void hardReset() {
  Serial.println("[HW] Hard-resetting SIM868 via PWRKEY cycle...");
  pinMode(PIN_SIM868_PWRKEY, OUTPUT);
  digitalWrite(PIN_SIM868_PWRKEY, LOW);
  delay(2000);
  pinMode(PIN_SIM868_PWRKEY, INPUT);
  delay(3000);
  powerOn();
  begin();
}

inline void setSleepMode(bool enable) {
  if (enable) {
    sendAT("AT+CSCLK=2");
    sleepActive = true;
  } else {
    wake();
    sendAT("AT+CSCLK=0");
    sleepActive = false;
  }
}

inline void begin() {
  sim868.begin(9600, SERIAL_8N1, PIN_SIM868_RX, PIN_SIM868_TX);
  powerOn();

  sendAT("ATE0");
  sendAT("AT+CFUN=1");
  sendAT("AT+CSCLK=0");
  sendAT("AT+CBAND=\"ALL_BAND\"");
  sendAT("AT+COPS=0");
  sendAT("AT+CREG=2");
  sendAT("AT+CGATT=1", 3000);

  consecutiveFails = 0;
  sleepActive = false;
}

inline bool waitRegistered(int seconds) {
  Serial.println("[GSM] Waiting for network registration...");
  for (int i = 0; i < seconds; i++) {
    if (isNetworkRegistered()) {
      Serial.println("[GSM] Network Registered!");
      return true;
    }
    delay(1000);
  }
  return false;
}

inline String runUSSD(const String &code, uint32_t timeoutMs = 12000) {
  if (!isNetworkRegistered()) {
    return "Error: Network not registered";
  }

  String cmd = "AT+CUSD=1,\"" + code + "\",15";
  String resp = sendAT(cmd, timeoutMs, "+CUSD:");

  int q1 = resp.indexOf('\"');
  if (q1 != -1) {
    int q2 = resp.lastIndexOf('\"');
    if (q2 > q1) {
      return resp.substring(q1 + 1, q2);
    }
  }

  if (resp.length() == 0) {
    return "Error: USSD timeout";
  }

  return resp;
}

}

inline HardwareSerial& sim868 = modem::sim868;
inline String sendAT(const String &cmd, uint32_t timeoutMs = 2000, const String &waitFor = "") {
  return modem::sendAT(cmd, timeoutMs, waitFor);
}
inline bool waitPrompt(uint32_t timeoutMs = 2500) { return modem::waitPrompt(timeoutMs); }
inline bool isNetworkRegistered() { return modem::isNetworkRegistered(); }
inline void powerOnModem() { modem::powerOn(); }
inline void modemHardReset() { modem::hardReset(); }
inline void modemSetSleepMode(bool enable) { modem::setSleepMode(enable); }
inline void modemBegin() { modem::begin(); }
inline bool modemWaitRegistered(int seconds) { return modem::waitRegistered(seconds); }
inline String runUSSD(const String &code, uint32_t timeoutMs = 12000) { return modem::runUSSD(code, timeoutMs); }
