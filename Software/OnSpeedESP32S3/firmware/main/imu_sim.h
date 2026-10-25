// Simulated ISM330DHCX for the QEMU build. It turns the flight model's attitude and flight
// path into the body rates and specific forces a real IMU would measure, then adds the
// datasheet noise, offsets and quantisation, so the Gen2 AHRS runs on realistic input.
#pragma once
#include "ahrs.h"
#include "flight_model.h"

class ImuSim {
 public:
  // Call once per flight-model step; returns the truth for the step just taken.
  void step(const FlightState &fs, float dt);
  // One IMU sample in Gen2's aircraft axes (after the axis map): Ax fwd, Ay right, Az up
  // (g, +1 level), Gx=-p, Gy=-q, Gz=+r (dps).
  onspeed::AircraftImu sample();
 private:
  float gauss();
  bool have_ = false;
  FlightState prev_{};
  float p_ = 0, q_ = 0, r_ = 0, fx_ = 0, fy_ = 0, fz_ = -1;  // body rates (dps), specific force (g, NED body)
  uint32_t seed_ = 0x2545F491u;
};
