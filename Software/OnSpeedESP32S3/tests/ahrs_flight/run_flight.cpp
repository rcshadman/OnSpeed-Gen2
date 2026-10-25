// Host run of the QEMU flight profile through ImuSim + Ahrs (same code as the firmware).
#include <stdio.h>
#include <math.h>
#include "flight_model.h"
#include "imu_sim.h"
int main(int argc, char **argv) {
  bool fixes = argc > 1;
  FlightModel fm; ImuSim imu;
  onspeed::AhrsConfig c; c.imuSampleRate = 200; c.fixKnownBugs = fixes;
  onspeed::Ahrs ahrs(c);
  const float dt = 0.02f; int ticks = (int)(fm.duration() / dt);
  float maxP = 0, maxR = 0, sumP = 0, sumR = 0; int n = 0; float worstT = 0;
  for (int k = 0; k <= ticks; k++) {
    fm.step(k * dt, dt); const FlightState &fs = fm.state(); imu.step(fs, dt);
    for (int j = 0; j < 4; j++) {
      onspeed::AircraftImu m = imu.sample();
      if (k == 0 && j == 0) ahrs.begin(m, fs.palt);
      ahrs.processAHRS(m, fs.ias, fs.palt, 15, fs.aoa);
    }
    const auto &s = ahrs.state();
    float ep = s.smoothedPitch - fs.pitch, er = s.smoothedRoll - fs.bank;
    if (k % 50 == 0) printf("%6.1f %-45s pitch %6.2f/%6.2f roll %6.2f/%6.2f vsi %6.0f/%6.0f\n", k * dt, fs.phase, s.smoothedPitch, fs.pitch, s.smoothedRoll, fs.bank, s.kalmanVSI * 196.85f, fs.vsi);
    if (k * dt > 5) { if (fabsf(ep) > maxP) { maxP = fabsf(ep); worstT = k*dt; } if (fabsf(er) > maxR) maxR = fabsf(er); sumP += ep*ep; sumR += er*er; n++; }
  }
  printf("max pitch err %.2f (t=%.1f), max roll err %.2f, rms %.2f / %.2f\n", maxP, worstT, maxR, sqrtf(sumP/n), sqrtf(sumR/n));
}
