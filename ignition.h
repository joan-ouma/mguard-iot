#pragma once
#include <Arduino.h>
#include "config.h"

namespace ignition {

inline int8_t  overrideMode = -1;
inline bool    currentState = false;

inline void begin() {
  pinMode(PIN_IGNITION, INPUT);
}

inline bool read() {
  if (overrideMode == 1) {
    currentState = true;
    return currentState;
  }
  if (overrideMode == 0) {
    currentState = false;
    return currentState;
  }
  
  int adcVal = analogRead(PIN_IGNITION);
  currentState = (adcVal > 1500);
  return currentState;
}

inline bool isIgnitionOn() {
  return currentState;
}

inline bool ignitionNow() {
  return read();
}

inline void setOverride(int8_t mode) {
  overrideMode = mode;
  if (overrideMode == 1) {
    currentState = true;
  } else if (overrideMode == 0) {
    currentState = false;
  } else {
    read();
  }
}

inline int8_t getOverride() {
  return overrideMode;
}

inline bool isOverridden() {
  return overrideMode != -1;
}

}

inline bool ignitionNow() {
  return ignition::ignitionNow();
}

inline void readIgnitionState() {
  ignition::read();
}
