#include "flight_model.h"
#include <math.h>

struct Key {
  float t, aoa, gamma, bank, latG;
  int flaps;
  float iasOverride;  // <0: IAS from the lift equation
  const char *phase;
};

static const Key kKeys[] = {
    {0, 0, 0, 0, 0, 0, 0, "Taxi (audio muted under 25 kt)"},
    {4, 0, 0, 0, 0, 0, 20, "Takeoff roll"},
    {8, 1, 0, 0, 0, 0, 70, "Takeoff roll"},
    {10, 9, 7, 0, 0, 0, -1, "Rotate and climb"},
    {20, 6, 5, 0, 0, 0, -1, "Climb"},
    {26, 1.5, 0, 0, 0, 0, -1, "Level off, cruise (tones off)"},
    {34, 1.5, 0, 0, 0, 0, -1, "Cruise (tones off)"},
    {42, 5, 0, 0, 0, 0, -1, "Slowing down"},
    {50, 9.5, 0, 0, 0, 0, -1, "L/Dmax: slow low-tone pulses"},
    {56, 12.5, 0, 0, 0, 0, -1, "OnSpeed: solid low tone"},
    {62, 12.5, 0, 0, 0, 0, -1, "OnSpeed: solid low tone"},
    {68, 15, 0, 0, 0, 0, -1, "Slow: high-tone pulses speed up"},
    {73, 17.5, 0, 0, 0, 0, -1, "Stall warning: 20 pulses/s"},
    {76, 17.5, 0, 0, 0, 0, -1, "Stall warning: 20 pulses/s"},
    {79, 5, -4, 0, 0, 0, -1, "Recovery: nose down"},
    {85, 3, 0, 0, 0, 0, -1, "Recovered"},
    {90, 5, 0, 45, 0, 0, -1, "45 degree steep turn"},
    {95, 11, 0, 45, 0, 0, -1, "Steep turn at OnSpeed"},
    {98, 12, 0, 45, 0.05, 0, -1, "Steep turn, lateral G +0.05: tone pans right"},
    {101, 12, 0, 45, 0.05, 0, -1, "Steep turn, lateral G +0.05: tone pans right"},
    {103, 12, 0, 45, -0.05, 0, -1, "Steep turn, lateral G -0.05: tone pans left"},
    {106, 12, 0, 45, -0.05, 0, -1, "Steep turn, lateral G -0.05: tone pans left"},
    {108, 4, 0, 0, 0, 0, -1, "Roll out"},
    {114, 5, -3, 0, 0, 20, -1, "Flaps 20, descending"},
    {120, 10.5, -3, 0, 0, 20, -1, "Approach at OnSpeed (flaps 20)"},
    {128, 11, -3, 0, 0, 20, -1, "Approach at OnSpeed (flaps 20)"},
    {133, 14, -3, 0, 0, 20, -1, "Too slow on approach"},
    {137, 11, -3, 0, 0, 20, -1, "Back to OnSpeed"},
    {143, 11, -3, 0, 0, 20, -1, "Short final"},
    {147, 13, 0, 0, 0, 20, -1, "Flare"},
    {150, 2, 0, 0, 0, 20, 45, "Touchdown"},
    {155, 0, 0, 0, 0, 20, 15, "Rollout (audio muted)"},
    {160, 0, 0, 0, 0, 20, 5, "Stopped"},
};
static const int kN = sizeof(kKeys) / sizeof(kKeys[0]);

float FlightModel::duration() const { return kKeys[kN - 1].t; }

static float mix(float a, float b, float f) { return a + (b - a) * f; }

void FlightModel::step(float t, float dt) {
  int i = 0;
  while (i < kN - 2 && t >= kKeys[i + 1].t) i++;
  const Key &a = kKeys[i], &b = kKeys[i + 1];
  float f = (t - a.t) / (b.t - a.t);
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  // ease the transitions so pitch changes look like a pilot flying it
  f = f * f * (3 - 2 * f);
  s_.t = t;
  s_.aoa = mix(a.aoa, b.aoa, f);
  s_.gamma = mix(a.gamma, b.gamma, f);
  s_.bank = mix(a.bank, b.bank, f);
  s_.latG = mix(a.latG, b.latG, f);
  s_.flaps = f < 0.5f ? a.flaps : b.flaps;
  s_.phase = f < 0.5f || i + 1 == kN - 1 ? a.phase : b.phase;
  float br = s_.bank * (float)M_PI / 180;
  s_.vertG = 1.0f / cosf(br);
  float ias;
  if (a.iasOverride >= 0 && b.iasOverride >= 0) {
    ias = mix(a.iasOverride, b.iasOverride, f);
  } else {
    // lift equation, CL linear in AOA; stall-warning AOA gives ~58 kt (flaps 0) / ~54 kt (flaps 20).
    // Blend the two lift curves while the flaps travel so airspeed changes smoothly.
    auto liftIas = [&](int flaps) {
      float aoa0 = flaps >= 20 ? -4.0f : -2.0f;
      float k = flaps >= 20 ? 243.0f : 249.5f;
      float cl = s_.aoa - aoa0;
      if (cl < 0.5f) cl = 0.5f;
      return k * sqrtf(s_.vertG / cl);
    };
    ias = mix(liftIas(a.flaps), liftIas(b.flaps), f);
    if (a.iasOverride >= 0) ias = mix(a.iasOverride, ias, f);  // blend out of the takeoff roll
    else if (b.iasOverride >= 0) ias = mix(ias, b.iasOverride, f);  // blend into touchdown
  }
  s_.ias = ias;
  s_.pitch = s_.aoa * cosf(br) + s_.gamma;
  if (s_.ias < 40 && s_.gamma == 0) s_.pitch = s_.aoa;  // on the wheels
  float tas = s_.ias * (1 + s_.palt / 1000 * 0.02f);
  s_.vsi = tas * 101.27f * sinf(s_.gamma * (float)M_PI / 180);
  s_.palt += s_.vsi / 60 * dt;
  if (s_.palt < 1000) s_.palt = 1000;
  if (tas > 30) s_.heading += 1091 * tanf(br) / tas * dt;
  while (s_.heading >= 360) s_.heading -= 360;
  while (s_.heading < 0) s_.heading += 360;
}
