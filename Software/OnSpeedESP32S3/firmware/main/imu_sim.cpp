#include "imu_sim.h"
#include <math.h>

// ISM330DHCX datasheet (DS13012), ODR 208 Hz, +/-8 g, +/-250 dps:
//   accel noise density 60 ug/rtHz, gyro 5 mdps/rtHz -> RMS over the 104 Hz Nyquist band
//   accel zero-g offset +/-10 mg typical (not removed: Gen2 only levels pitch/roll in software)
//   gyro zero-rate offset +/-1 dps typical, removed by Gen2's ground calibration; 0.05 dps left
//   sensitivity 0.244 mg/LSB and 8.75 mdps/LSB
static const float kAccNoise = 60e-6f * 10.2f, kGyroNoise = 0.005f * 10.2f;
static const float kAccOff[3] = {0.010f, -0.008f, 0.006f};
static const float kGyroOff[3] = {0.05f, -0.04f, 0.03f};
static const float kAccLsb = 0.000244f, kGyroLsb = 0.00875f;
static const float D2R = 0.01745329f, G = 9.80665f;

float ImuSim::gauss() {  // Box-Muller from an xorshift generator
  auto u = [&]() { seed_ ^= seed_ << 13; seed_ ^= seed_ >> 17; seed_ ^= seed_ << 5; return (seed_ + 0.5f) / 4294967296.0f; };
  float a = u(), b = u();
  return sqrtf(-2 * logf(a)) * cosf(6.2831853f * b);
}

static void velNED(const FlightState &s, float v[3]) {
  float tas = s.ias * (1 + s.palt / 1000 * 0.02f) * 0.514444f;
  float g = s.gamma * D2R, h = s.heading * D2R;
  v[0] = tas * cosf(g) * cosf(h); v[1] = tas * cosf(g) * sinf(h); v[2] = -tas * sinf(g);
}

void ImuSim::step(const FlightState &fs, float dt) {
  if (!have_) { prev_ = fs; have_ = true; }
  // Euler-angle rates by backward difference, heading unwrapped
  float dphi = (fs.bank - prev_.bank) / dt, dth = (fs.pitch - prev_.pitch) / dt;
  float dh = fs.heading - prev_.heading;
  if (dh > 180) dh -= 360;
  if (dh < -180) dh += 360;
  float dpsi = dh / dt;
  float phi = fs.bank * D2R, th = fs.pitch * D2R, psi = fs.heading * D2R;
  p_ = dphi - dpsi * sinf(th);
  q_ = dth * cosf(phi) + dpsi * cosf(th) * sinf(phi);
  r_ = -dth * sinf(phi) + dpsi * cosf(th) * cosf(phi);
  // specific force f = a - g in NED, rotated into the body frame
  float v0[3], v1[3];
  velNED(prev_, v0); velNED(fs, v1);
  float fn = (v1[0] - v0[0]) / dt, fe = (v1[1] - v0[1]) / dt, fd = (v1[2] - v0[2]) / dt - G;
  float cph = cosf(phi), sph = sinf(phi), cth = cosf(th), sth = sinf(th), cps = cosf(psi), sps = sinf(psi);
  // body = R^T * NED, R = Rz(psi) Ry(th) Rx(phi)
  float bx = cth * cps * fn + cth * sps * fe - sth * fd;
  float by = (sph * sth * cps - cph * sps) * fn + (sph * sth * sps + cph * cps) * fe + sph * cth * fd;
  float bz = (cph * sth * cps + sph * sps) * fn + (cph * sth * sps - sph * cps) * fe + cph * cth * fd;
  fx_ = bx / G; fy_ = by / G; fz_ = bz / G;
  prev_ = fs;
}

onspeed::AircraftImu ImuSim::sample() {
  auto qa = [](float v) { return roundf(v / kAccLsb) * kAccLsb; };
  auto qg = [](float v) { return roundf(v / kGyroLsb) * kGyroLsb; };
  onspeed::AircraftImu m;
  m.Ax = qa(fx_ + kAccOff[0] + kAccNoise * gauss());
  m.Ay = qa(fy_ + kAccOff[1] + kAccNoise * gauss());
  m.Az = qa(-fz_ + kAccOff[2] + kAccNoise * gauss());
  m.Gx = qg(-p_ + kGyroOff[0] + kGyroNoise * gauss());
  m.Gy = qg(-q_ + kGyroOff[1] + kGyroNoise * gauss());
  m.Gz = qg(r_ + kGyroOff[2] + kGyroNoise * gauss());
  return m;
}
