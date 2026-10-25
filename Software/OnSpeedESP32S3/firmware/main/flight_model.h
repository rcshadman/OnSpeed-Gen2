// Scripted flight used to drive the virtual sensors: taxi, takeoff, a slow-down through every
// AOA band to stall warning, steep turns with slip, and a flaps-20 approach and landing.
#pragma once
#include <stdint.h>

struct FlightState {
  float t;            // s
  float aoa;          // deg, true AOA the probe sees
  float gamma;        // deg, flight path angle
  float bank;         // deg
  float pitch;        // deg
  int flaps;          // deg (0, 20, 40)
  float ias;          // kt
  float latG, vertG;  // g
  float palt;         // ft
  float vsi;          // ft/min
  float heading;      // deg
  const char *phase;
};

class FlightModel {
 public:
  void step(float t, float dt);
  const FlightState &state() const { return s_; }
  float duration() const;
 private:
  FlightState s_ = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1000, 0, 270, ""};
};
