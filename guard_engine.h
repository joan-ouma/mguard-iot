#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include "config.h"

namespace guard {

constexpr uint8_t  IMU_I2C_ADDR       = 0x68;
constexpr uint16_t SAMPLE_HZ          = 100;
constexpr uint8_t  WINDOW_SIZE        = 25;
constexpr uint32_t ARM_DELAY_MS       = 45000;
constexpr uint32_t SETTLE_MS          = 4000;
constexpr uint32_t IGN_DEBOUNCE_MS    = 1500;
constexpr float    SCORE_ALARM        = 8.0f;
constexpr float    SCORE_DECAY        = 0.25f;
constexpr float    TILT_THRESHOLD_DEG = 12.0f;
constexpr uint32_t TILT_HOLD_MS       = 2000;
constexpr uint32_t REPEAT_ALERT_MS    = 30000;
constexpr uint32_t CALM_DURATION_MS   = 60000;
constexpr float    NOISE_FLOOR        = 0.010f;

enum State : uint8_t { DISARMED, SETTLING, ARMED, ALARM };
struct Raw   { int16_t ax, ay, az, gx, gy, gz; };
struct Event { const char* reason; float score; float tiltDeg; };
struct Features {
  float rms;
  float peak;
  float gyroEnergy;
  float gyroSpread;
  float tiltDeg;
  float accelMean[3];
  float gyroMean[3];
};

inline bool (*ignitionFn)() = nullptr;
inline bool (*authCheckFn)() = nullptr;
inline volatile State state = DISARMED;
inline volatile bool  logCsv = false;

inline void setAuthCheck(bool (*fn)()) {
  authCheckFn = fn;
}

inline Raw   latestRaw = {0, 0, 0, 0, 0, 0};
inline portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
inline volatile bool pending = false;
inline Event pendingEv = {"", 0, 0};

inline float gravityRef[3]  = {0, 0, 1};
inline float gyroBias[3]    = {0, 0, 0};
inline float noiseRms       = NOISE_FLOOR;
inline float threatScore    = 0.0f;
inline float sampleBuffer[WINDOW_SIZE][6];
inline uint8_t windowSampleIdx = 0;
inline float noiseAccumulator  = 0.0f;
inline uint16_t noiseSampleCount = 0;
inline uint32_t quietSinceMs   = 0;
inline uint32_t tiltSinceMs    = 0;
inline uint32_t lastEmitMs     = 0;
inline uint32_t lastActiveMs   = 0;
inline uint32_t ignOffSinceMs  = 0;
inline const char* primaryCause = "handling";
inline volatile double parkLat = 0.0;
inline volatile double parkLon = 0.0;
inline uint8_t farCount = 0;
inline Preferences prefs;

inline bool writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

inline int16_t readInt16() {
  uint8_t highByte = Wire.read();
  uint8_t lowByte  = Wire.read();
  return (int16_t)((highByte << 8) | lowByte);
}

inline bool readRaw(Raw& r) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(IMU_I2C_ADDR, (uint8_t)14, (uint8_t)true) != 14) return false;

  r.ax = readInt16(); r.ay = readInt16(); r.az = readInt16();
  readInt16();
  r.gx = readInt16(); r.gy = readInt16(); r.gz = readInt16();

  if (r.ax == 0 && r.ay == 0 && r.az == 0) return false;
  return true;
}

inline void normalizeVector(const float* v, float* out) {
  float norm = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (norm < 1e-6f) norm = 1.0f;
  out[0] = v[0] / norm; out[1] = v[1] / norm; out[2] = v[2] / norm;
}

inline bool isIgnitionDebounced(bool rawState, uint32_t now) {
  static bool stableState = false, lastRaw = false;
  static uint32_t changeTimestamp = 0;
  if (rawState != lastRaw) {
    lastRaw = rawState;
    changeTimestamp = now;
  }
  if (now - changeTimestamp >= IGN_DEBOUNCE_MS) {
    stableState = lastRaw;
  }
  return stableState;
}

inline void emit(const char* reason, float score, float tilt) {
  portENTER_CRITICAL(&mux);
  pendingEv = Event{reason, score, tilt};
  pending = true;
  portEXIT_CRITICAL(&mux);
}

inline bool poll(Event& out) {
  bool hasEvent = false;
  portENTER_CRITICAL(&mux);
  if (pending) {
    out = pendingEv;
    pending = false;
    hasEvent = true;
  }
  portEXIT_CRITICAL(&mux);
  return hasEvent;
}

inline void latest(Raw& out) {
  portENTER_CRITICAL(&mux);
  out = latestRaw;
  portEXIT_CRITICAL(&mux);
}

inline void saveNvsState() {
  prefs.begin("motoguard", false);
  prefs.putUChar("state", (uint8_t)state);
  prefs.putDouble("plat", (double)parkLat);
  prefs.putDouble("plon", (double)parkLon);
  prefs.end();
}

inline void transitionToState(State newState, uint32_t now) {
  (void)now;
  State previousState = state;
  state = newState;
  threatScore = 0.0f;
  tiltSinceMs = 0;
  quietSinceMs = 0;
  noiseAccumulator = 0.0f;
  noiseSampleCount = 0;

  if (newState == DISARMED) {
    parkLat = 0.0;
    parkLon = 0.0;
    farCount = 0;
  } else if (previousState == DISARMED && newState == SETTLING) {
    parkLat = 0.0;
    parkLon = 0.0;
    farCount = 0;
  }

  saveNvsState();
}

inline void disarm() {
  transitionToState(DISARMED, millis());
}

inline void armManual() {
  transitionToState(SETTLING, millis());
}

inline Features calculateFeatures() {
  Features feat = {};
  for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
    for (uint8_t j = 0; j < 3; j++) {
      feat.accelMean[j] += sampleBuffer[i][j] / WINDOW_SIZE;
      feat.gyroMean[j]  += sampleBuffer[i][3 + j] / WINDOW_SIZE;
    }
  }

  float accelVarSum = 0.0f, gyroSpreadSum = 0.0f, gyroEnergySum = 0.0f, peakSq = 0.0f;
  for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
    float dx = sampleBuffer[i][0] - feat.accelMean[0];
    float dy = sampleBuffer[i][1] - feat.accelMean[1];
    float dz = sampleBuffer[i][2] - feat.accelMean[2];
    float magSq = dx * dx + dy * dy + dz * dz;
    accelVarSum += magSq;
    if (magSq > peakSq) peakSq = magSq;

    float gxDiff = sampleBuffer[i][3] - feat.gyroMean[0];
    float gyDiff = sampleBuffer[i][4] - feat.gyroMean[1];
    float gzDiff = sampleBuffer[i][5] - feat.gyroMean[2];
    gyroSpreadSum += gxDiff * gxDiff + gyDiff * gyDiff + gzDiff * gzDiff;

    float bx = sampleBuffer[i][3] - gyroBias[0];
    float by = sampleBuffer[i][4] - gyroBias[1];
    float bz = sampleBuffer[i][5] - gyroBias[2];
    gyroEnergySum += bx * bx + by * by + bz * bz;
  }

  feat.rms        = sqrtf(accelVarSum / WINDOW_SIZE);
  feat.peak       = sqrtf(peakSq);
  feat.gyroSpread = sqrtf(gyroSpreadSum / WINDOW_SIZE);
  feat.gyroEnergy = sqrtf(gyroEnergySum / WINDOW_SIZE);

  float unitAccel[3];
  normalizeVector(feat.accelMean, unitAccel);
  float dotProduct = unitAccel[0] * gravityRef[0] + unitAccel[1] * gravityRef[1] + unitAccel[2] * gravityRef[2];
  feat.tiltDeg = acosf(fminf(1.0f, fmaxf(-1.0f, dotProduct))) * 57.29577951308232f;
  return feat;
}

inline void onWindowComplete(const Features& feat, uint32_t now, bool ignitionActive) {
  float vibrationSigma = feat.rms / noiseRms;

  if (logCsv) {
    Serial.printf("W,%lu,%d,%.4f,%.3f,%.2f,%.1f,%.2f\n",
                  (unsigned long)now, (int)state, feat.rms, feat.peak, feat.gyroEnergy, feat.tiltDeg, threatScore);
  }

  if (ignitionActive) {
    if (state != DISARMED) {
      bool isAuthed = authCheckFn ? authCheckFn() : true;
      if (!isAuthed) {
        emit("hotwire_unauthorized_ignition", 10.0f, feat.tiltDeg);
        state = ALARM;
        saveNvsState();
        return;
      }
      transitionToState(DISARMED, now);
    }
    ignOffSinceMs = now;
    return;
  }

  if (state == DISARMED) {
    if (now - ignOffSinceMs >= ARM_DELAY_MS) transitionToState(SETTLING, now);
    return;
  }

  if (state == SETTLING) {
    bool isQuiet = (feat.rms < 0.025f) && (feat.gyroSpread < 1.5f);
    if (!isQuiet) {
      quietSinceMs = 0;
      noiseAccumulator = 0.0f;
      noiseSampleCount = 0;
      return;
    }
    if (!quietSinceMs) quietSinceMs = now;
    noiseAccumulator += feat.rms;
    noiseSampleCount++;

    if (now - quietSinceMs >= SETTLE_MS) {
      float calculatedNoise = fmaxf(NOISE_FLOOR, noiseAccumulator / (float)noiseSampleCount);
      normalizeVector(feat.accelMean, gravityRef);
      for (int j = 0; j < 3; j++) gyroBias[j] = feat.gyroMean[j];
      transitionToState(ARMED, now);
      noiseRms = calculatedNoise;
      Serial.printf("[GUARD] ARMED. Resting noise=%.4f g\n", noiseRms);
    }
    return;
  }

  if (vibrationSigma < 2.0f && feat.tiltDeg < 3.0f) {
    noiseRms = fmaxf(NOISE_FLOOR, 0.98f * noiseRms + 0.02f * feat.rms);
    float unitAccel[3];
    normalizeVector(feat.accelMean, unitAccel);
    for (int j = 0; j < 3; j++) {
      gravityRef[j] = 0.99f * gravityRef[j] + 0.01f * unitAccel[j];
      gyroBias[j]   = 0.99f * gyroBias[j]   + 0.01f * feat.gyroMean[j];
    }
    normalizeVector(gravityRef, gravityRef);
  }

  threatScore = fmaxf(0.0f, threatScore - SCORE_DECAY);
  if (vibrationSigma > 5.0f)   threatScore += 0.2f;
  if (vibrationSigma > 10.0f)  { threatScore += 0.8f; primaryCause = "handling"; }
  if (feat.gyroEnergy > 15.0f) { threatScore += 0.8f; primaryCause = "rotation_or_push"; }
  if (feat.peak > 0.35f)       { threatScore += 1.5f; primaryCause = "impact"; }

  if (feat.tiltDeg > TILT_THRESHOLD_DEG) {
    if (!tiltSinceMs) tiltSinceMs = now;
  } else {
    tiltSinceMs = 0;
  }

  bool tiltHoldTriggered = tiltSinceMs && (now - tiltSinceMs >= TILT_HOLD_MS);
  if (vibrationSigma > 5.0f || tiltSinceMs) lastActiveMs = now;

  if (state == ARMED) {
    if (threatScore >= SCORE_ALARM || tiltHoldTriggered) {
      state = ALARM;
      lastEmitMs = now;
      saveNvsState();
      emit(tiltHoldTriggered ? "tilt_or_lift" : primaryCause, threatScore, feat.tiltDeg);
    }
  } else {
    if (now - lastActiveMs >= CALM_DURATION_MS) {
      transitionToState(SETTLING, now);
    } else if (now - lastEmitMs >= REPEAT_ALERT_MS && (threatScore >= SCORE_ALARM * 0.5f || tiltHoldTriggered)) {
      lastEmitMs = now;
      emit(tiltHoldTriggered ? "tilt_or_lift" : primaryCause, threatScore, feat.tiltDeg);
    }
  }
}

inline void step() {
  static Raw prev = {0, 0, 0, 0, 0, 0};
  static bool havePrev = false;
  static uint8_t failCount = 0, rejectCount = 0;
  uint32_t now = millis();

  Raw r;
  if (!readRaw(r)) {
    if (++failCount > 20) {
      Wire.end();
      delay(5);
      Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
      failCount = 0;
    }
    return;
  }
  failCount = 0;

  float ax = r.ax / 16384.0f, ay = r.ay / 16384.0f, az = r.az / 16384.0f;

  if (havePrev) {
    float dx = ax - prev.ax / 16384.0f, dy = ay - prev.ay / 16384.0f, dz = az - prev.az / 16384.0f;
    if (sqrtf(dx * dx + dy * dy + dz * dz) > 1.0f && rejectCount < 3) {
      rejectCount++;
      return;
    }
  }
  rejectCount = 0; prev = r; havePrev = true;

  portENTER_CRITICAL(&mux);
  latestRaw = r;
  portEXIT_CRITICAL(&mux);

  sampleBuffer[windowSampleIdx][0] = ax;
  sampleBuffer[windowSampleIdx][1] = ay;
  sampleBuffer[windowSampleIdx][2] = az;
  sampleBuffer[windowSampleIdx][3] = r.gx / 131.0f;
  sampleBuffer[windowSampleIdx][4] = r.gy / 131.0f;
  sampleBuffer[windowSampleIdx][5] = r.gz / 131.0f;

  if (++windowSampleIdx >= WINDOW_SIZE) {
    windowSampleIdx = 0;
    bool ign = isIgnitionDebounced(ignitionFn ? ignitionFn() : false, now);
    Features feat = calculateFeatures();
    onWindowComplete(feat, now, ign);
  }
}

inline void taskFn(void*) {
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&t, pdMS_TO_TICKS(1000 / SAMPLE_HZ));
    step();
  }
}

inline bool begin(bool (*ignition)()) {
  ignitionFn = ignition;
  ignOffSinceMs = millis();

  prefs.begin("motoguard", true);
  uint8_t savedState = prefs.getUChar("state", (uint8_t)DISARMED);
  parkLat = prefs.getDouble("plat", 0.0);
  parkLon = prefs.getDouble("plon", 0.0);
  prefs.end();

  if (savedState == (uint8_t)ARMED || savedState == (uint8_t)ALARM) {
    state = (State)savedState;
    Serial.printf("[GUARD] Restored state from NVS: %s, park=(%.6f, %.6f)\n",
                  (state == ARMED ? "ARMED" : "ALARM"), (double)parkLat, (double)parkLon);
  }

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_I2C_ADDR, (uint8_t)1, (uint8_t)true);
  uint8_t who = Wire.available() ? Wire.read() : 0;
  Serial.printf("[GUARD] WHO_AM_I=0x%02X (MPU-6500=0x70, MPU-9250=0x71, MPU-6050=0x68)\n", who);
  if (who == 0x00 || who == 0xFF) { Serial.println("[GUARD] IMU not found"); return false; }

  writeRegister(0x6B, 0x80); delay(100);
  writeRegister(0x6B, 0x01); delay(50);
  writeRegister(0x19, 9);
  writeRegister(0x1A, 0x04);
  writeRegister(0x1B, 0x00);
  writeRegister(0x1C, 0x00);
  writeRegister(0x1D, 0x04);

  if (logCsv) Serial.println("W,ms,state,rms_g,peak_g,gyro_dps,tilt_deg,score");
  xTaskCreatePinnedToCore(taskFn, "guard", 6144, nullptr, 3, nullptr, 0);
  return true;
}

inline void onGpsFix(double lat, double lon, float speedKmh) {
  if (state != ARMED && state != ALARM) return;
  if (parkLat == 0.0) {
    parkLat = lat;
    parkLon = lon;
    saveNvsState();
    return;
  }

  double dy = (lat - parkLat) * METERS_PER_DEG_LAT;
  double dx = (lon - parkLon) * METERS_PER_DEG_LAT * cos(parkLat * DEG_TO_RAD_FACTOR);
  float distance = (float)sqrt(dx * dx + dy * dy);

  if (distance > 40.0f || speedKmh > 6.0f) {
    if (++farCount >= 3) {
      emit("towed_or_stolen", threatScore, distance);
      farCount = 0;
    }
  } else {
    farCount = 0;
  }
}

}
