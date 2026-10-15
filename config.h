#pragma once
#include <Arduino.h>

constexpr uint8_t PIN_SIM868_RX     = 16;
constexpr uint8_t PIN_SIM868_TX     = 17;
constexpr uint8_t PIN_SIM868_PWRKEY = 4;

constexpr uint8_t PIN_IGNITION      = 34;

constexpr uint8_t PIN_I2C_SDA       = 21;
constexpr uint8_t PIN_I2C_SCL       = 22;

// RC522 RFID (SPI) Pinout (IRQ left disconnected)
constexpr uint8_t PIN_RC522_RST     = 27;
constexpr uint8_t PIN_RC522_MISO    = 32;
constexpr uint8_t PIN_RC522_MOSI    = 33;
constexpr uint8_t PIN_RC522_SCK     = 25;
constexpr uint8_t PIN_RC522_SDA     = 26; // SS / Chip Select

const char* const CELLULAR_APN      = "safaricom";

const char* const BROKER_HOST       = "suna.jullie.tech";
constexpr uint16_t BROKER_PORT      = 1883;
constexpr bool     USE_SSL          = false;
const char* const CLIENT_ID         = "860123456789001";
const char* const MQTT_USER         = "kardio";
const char* const MQTT_PASSWORD     = "MustaGND0R#";

const char* const TELEMETRY_TOPIC   = "motoguard/v1/telemetry/860123456789001";
const char* const ALERT_TOPIC       = "motoguard/v1/alert/860123456789001";

constexpr uint32_t TELEMETRY_INTERVAL_ARMED_MS   = 120000;
constexpr uint32_t TELEMETRY_INTERVAL_ALARM_MS   = 15000;
constexpr uint32_t TELEMETRY_INTERVAL_DRIVING_MS = 10000;
constexpr uint32_t TELEMETRY_INTERVAL_BENCH_MS   = 2000;
constexpr uint32_t TELEMETRY_INTERVAL_MS         = 120000;

constexpr uint32_t GPS_TOW_CHECK_MS              = 10000;

constexpr uint32_t MQTT_RECONNECT_MIN_MS         = 5000;
constexpr uint32_t MQTT_RECONNECT_MAX_MS         = 60000;
constexpr uint8_t  MODEM_MAX_CONSECUTIVE_FAILS   = 5;

constexpr double METERS_PER_DEG_LAT              = 111139.0;
constexpr double DEG_TO_RAD_FACTOR               = 0.017453292519943295;
