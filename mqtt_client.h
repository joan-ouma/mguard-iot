#pragma once
#include <Arduino.h>
#include "config.h"
#include "modem.h"

namespace mqtt {

inline bool connectTcpSocket() {
  if (!modem::isNetworkRegistered()) {
    Serial.println("[GSM] Not registered on cellular network yet.");
    return false;
  }

  Serial.println("[TCP] Reconnecting TCP/SSL Socket to Broker...");
  modem::sendAT("AT+CIPSHUT", 1000);
  modem::sendAT("AT+CIPMUX=0", 500);
  modem::sendAT("AT+CIPCCFG=10,2,256,1", 500);
  modem::sendAT(String("AT+CSTT=\"") + CELLULAR_APN + "\"", 1000);
  modem::sendAT("AT+CIICR", 3000);
  modem::sendAT("AT+CIFSR", 1000);

  if (USE_SSL) {
    modem::sendAT("AT+CIPSSL=1", 1000);
  } else {
    modem::sendAT("AT+CIPSSL=0", 1000);
  }

  String connCmd = String("AT+CIPSTART=\"TCP\",\"") + BROKER_HOST + "\"," + String(BROKER_PORT);
  String connResp = modem::sendAT(connCmd, 15000, "CONNECT");

  if (connResp.indexOf("CONNECT OK") == -1 && connResp.indexOf("ALREADY CONNECT") == -1) {
    Serial.println("[TCP] Socket Connection Failed: " + connResp);
    return false;
  }

  Serial.println("[TCP] Socket Connected Successfully!");
  return true;
}

inline void sendConnectPacket() {
  Serial.println("[MQTT] Transmitting Authenticated Connect Packet to Broker...");

  uint16_t clientIdLen = strlen(CLIENT_ID);
  uint16_t userLen     = strlen(MQTT_USER);
  uint16_t passLen     = strlen(MQTT_PASSWORD);

  int remLen = 10 + (2 + clientIdLen) + (2 + userLen) + (2 + passLen);

  uint8_t remBytes[4];
  int remCount = 0;
  int temp = remLen;
  do {
    uint8_t d = temp % 128;
    temp /= 128;
    if (temp > 0) d |= 128;
    remBytes[remCount++] = d;
  } while (temp > 0);

  int totalLen = 1 + remCount + remLen;

  while (modem::sim868.available()) modem::sim868.read();

  modem::sim868.print("AT+CIPSEND=");
  modem::sim868.println(totalLen);

  if (!modem::waitPrompt()) {
    Serial.println("[MQTT] Connect prompt failed.");
    return;
  }

  modem::sim868.write((uint8_t)0x10);

  for (int i = 0; i < remCount; i++) {
    modem::sim868.write(remBytes[i]);
  }

  uint8_t varHeader[] = {
    0x00, 0x04, 'M', 'Q', 'T', 'T',
    0x04, 0xC2, 0x00, 0xB4
  };
  modem::sim868.write(varHeader, sizeof(varHeader));

  modem::sim868.write((uint8_t)(clientIdLen >> 8));
  modem::sim868.write((uint8_t)(clientIdLen & 0xFF));
  modem::sim868.print(CLIENT_ID);

  modem::sim868.write((uint8_t)(userLen >> 8));
  modem::sim868.write((uint8_t)(userLen & 0xFF));
  modem::sim868.print(MQTT_USER);

  modem::sim868.write((uint8_t)(passLen >> 8));
  modem::sim868.write((uint8_t)(passLen & 0xFF));
  modem::sim868.print(MQTT_PASSWORD);

  delay(400);
}

inline void recoverTcpConnection() {
  if (!connectTcpSocket()) return;
  sendConnectPacket();
}

inline bool publish(const char* topic, const String &payload) {
  int topicLen = strlen(topic);
  int payloadLen = payload.length();
  int remLen = 2 + topicLen + payloadLen;

  uint8_t remBytes[4];
  int remCount = 0;
  int temp = remLen;
  do {
    uint8_t d = temp % 128;
    temp /= 128;
    if (temp > 0) d |= 128;
    remBytes[remCount++] = d;
  } while (temp > 0);

  int totalLen = 1 + remCount + 2 + topicLen + payloadLen;

  while (modem::sim868.available()) modem::sim868.read();

  modem::sim868.print("AT+CIPSEND=");
  modem::sim868.println(totalLen);

  if (!modem::waitPrompt()) {
    Serial.println("[MQTT] Publish failed (prompt not received). Re-establishing connection...");
    recoverTcpConnection();
    return false;
  }

  modem::sim868.write(0x30);
  for (int i = 0; i < remCount; i++) modem::sim868.write(remBytes[i]);
  modem::sim868.write(topicLen >> 8);
  modem::sim868.write(topicLen & 0xFF);
  modem::sim868.print(topic);
  modem::sim868.print(payload);
  return true;
}

inline bool publishTelemetry(const String &payload) {
  if (!modem::isNetworkRegistered()) {
    Serial.println("[GSM] Not registered on cellular network yet.");
    return false;
  }
  bool ok = publish(TELEMETRY_TOPIC, payload);
  if (ok) Serial.println("[MQTT] Telemetry Published Successfully!");
  return ok;
}

inline bool publishAlert(const String &payload) {
  if (!modem::isNetworkRegistered()) return false;
  bool ok = publish(ALERT_TOPIC, payload);
  if (ok) Serial.println("[ALERT] Burglary Alert Published via MQTT!");
  return ok;
}

}

inline bool connectTcpSocket() { return mqtt::connectTcpSocket(); }
inline void mqttSendConnectPacket() { mqtt::sendConnectPacket(); }
inline void recoverTcpConnection() { mqtt::recoverTcpConnection(); }
inline bool mqttPublish(const char* topic, const String &payload) { return mqtt::publish(topic, payload); }
inline bool mqttPublishPayload(const String &payload) { return mqtt::publishTelemetry(payload); }
inline bool mqttPublishAlert(const String &payload) { return mqtt::publishAlert(payload); }
