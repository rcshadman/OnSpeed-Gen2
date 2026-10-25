// Host timing of the AHRS port: ns per processAHRS() (and its parts) on recorded sim data.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

#include "ahrs.h"
#include "sim.h"

using namespace onspeed;

int main() {
  // 60 s of a 45 deg turn, realistic sensor errors
  AxisMap map = configureAxes("FORWARD", "UP");
  sim::Mount m{};
  const AxisSpec rows[3] = {map.forward, map.lateral, map.vertical};
  for (int r = 0; r < 3; r++) { m.Ma[r][rows[r].axis] = rows[r].sign; m.Mg[r][rows[r].axis] = -rows[r].sign; }
  sim::SensorCfg sc;
  sim::FlightState fs; fs.V = 100 * sim::KT; fs.D = -900;
  sim::Sim S(sc, fs, [](double t, const sim::FlightState &) { sim::Cmd c; c.mu = sim::smooth(t, 5, 8, 0, 45 * sim::D2R); return c; }, m);
  const int N = 208 * 60;
  std::vector<AircraftImu> imu(N);
  std::vector<float> ias(N), palt(N);
  for (int k = 0; k < N; k++) {
    int16_t a[3], g[3]; sim::Truth tr;
    S.sample(a, g, &palt[size_t(k)], &ias[size_t(k)], &tr);
    imu[size_t(k)] = mapAxes(map, scaleImu(a, g));
  }
  auto timeit = [&](const char *name, auto fn) {
    std::vector<double> reps;
    for (int r = 0; r < 15; r++) {
      auto t0 = std::chrono::steady_clock::now();
      fn();
      auto t1 = std::chrono::steady_clock::now();
      reps.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count() / N);
    }
    std::sort(reps.begin(), reps.end());
    printf("%-44s median %7.1f ns/update  (min %.1f)\n", name, reps[reps.size() / 2], reps[0]);
  };
  volatile float sink = 0;
  for (int mode = 0; mode < 2; mode++) {
    AhrsConfig ac; ac.fixKnownBugs = mode;
    timeit(mode ? "Ahrs::processAHRS (fixKnownBugs)" : "Ahrs::processAHRS (original)", [&] {
      Ahrs a(ac); a.begin(imu[0], palt[0]);
      for (int k = 0; k < N; k++) a.processAHRS(imu[size_t(k)], ias[size_t(k)], palt[size_t(k)]);
      sink = sink + a.state().smoothedPitch;
    });
  }
  timeit("  Madgwick::updateIMU + getPitch/getRoll", [&] {
    Madgwick f; f.begin(208, 0, 0);
    for (int k = 0; k < N; k++) {
      const AircraftImu &i = imu[size_t(k)];
      f.updateIMU(i.Gx, i.Gy, i.Gz, i.Ax, i.Ay, i.Az);
      sink = sink + f.getPitch() + f.getRoll();
    }
  });
  timeit("  KalmanFilter::Update", [&] {
    KalmanFilter kf; kf.Configure(0.79078f, 26.0638f, 1e-11f, palt[0] * 0.3048f, 0, 0);
    float z, v;
    for (int k = 0; k < N; k++) { kf.Update(palt[size_t(k)] * 0.3048f, 0.01f, 1.0f / 208, &z, &v); }
    sink = sink + v;
  });
  timeit("  scaleImu + mapAxes (IMUread.ino)", [&] {
    int16_t a[3] = {100, -2000, 4000}, g[3] = {10, -20, 30};
    for (int k = 0; k < N; k++) { a[0] = int16_t(k); AircraftImu x = mapAxes(map, scaleImu(a, g)); sink = sink + x.Ax; }
  });
  return 0;
}
