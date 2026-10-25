// 6-DOF kinematic flight + ISM330DHCX/baro/IAS sensor simulator for the OnSpeed Gen2 AHRS host test.
// All truth math is double precision (host only); the outputs are what the Gen2 firmware sees:
// int16 chip-frame IMU counts (before scaleAccel/scaleGyro and the axis map), static pressure as
// sensor counts converted to mbar, IAS in knots updated at 50 Hz.
//
// Flight model: point-mass wind-axis kinematics (V, gamma, chi) driven by commanded
// Vdot, bank mu and load factor n (or a flight-path-hold law), so the trajectory is always
// physically consistent. The body frame = wind frame rotated by sideslip (yaw), AOA (pitch) and
// a roll offset (cross-controlled slip). Specific force f_w = (Vdot + g sin gamma, 0, -n g).
#pragma once
#include <cmath>
#include <cstdint>
#include <functional>
#include <random>
#include <vector>

namespace sim {

constexpr double G = 9.80665, D2R = M_PI / 180, R2D = 180 / M_PI, KT = 0.514444;

struct M3 {
  double m[3][3];
  static M3 I() { M3 r{}; r.m[0][0] = r.m[1][1] = r.m[2][2] = 1; return r; }
  M3 operator*(const M3 &b) const { M3 r{}; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) for (int k = 0; k < 3; k++) r.m[i][j] += m[i][k] * b.m[k][j]; return r; }
  M3 T() const { M3 r{}; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) r.m[i][j] = m[j][i]; return r; }
};
struct V3 { double x = 0, y = 0, z = 0; };
inline V3 mul(const M3 &a, const V3 &v) { return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z, a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z, a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z}; }
inline M3 Rx(double a) { M3 r = M3::I(); r.m[1][1] = cos(a); r.m[1][2] = -sin(a); r.m[2][1] = sin(a); r.m[2][2] = cos(a); return r; }
inline M3 Ry(double a) { M3 r = M3::I(); r.m[0][0] = cos(a); r.m[0][2] = sin(a); r.m[2][0] = -sin(a); r.m[2][2] = cos(a); return r; }
inline M3 Rz(double a) { M3 r = M3::I(); r.m[0][0] = cos(a); r.m[0][1] = -sin(a); r.m[1][0] = sin(a); r.m[1][1] = cos(a); return r; }
// rotation vector of R (log map)
inline V3 rotvec(const M3 &R) {
  V3 v{(R.m[2][1] - R.m[1][2]) / 2, (R.m[0][2] - R.m[2][0]) / 2, (R.m[1][0] - R.m[0][1]) / 2};
  double s = sqrt(v.x * v.x + v.y * v.y + v.z * v.z), c = (R.m[0][0] + R.m[1][1] + R.m[2][2] - 1) / 2;
  double th = atan2(s, c), k = s > 1e-12 ? th / s : 1.0;
  return {v.x * k, v.y * k, v.z * k};
}

inline double smooth(double t, double t0, double t1, double a, double b) {  // smoothstep a->b
  if (t <= t0) return a;
  if (t >= t1) return b;
  double u = (t - t0) / (t1 - t0);
  return a + (b - a) * u * u * (3 - 2 * u);
}

struct Cmd {
  double Vdot = 0;            // m/s^2
  double mu = 0;              // wind-axis bank, rad
  double n = NAN;             // load factor; NAN -> flight path hold on gammaCmd
  double gammaCmd = 0;        // rad
  double dphi = 0, betay = 0; // body roll offset and yaw offset vs wind axes (slip), rad
  double alphaOv = NAN;       // AOA override (rad), e.g. on the ground; NAN -> lift curve
};

struct FlightState { double t = 0, V = 60, gam = 0, chi = 0, N = 0, E = 0, D = -300; };

// Gauss-Markov turbulence channel: two cascaded first-order lags on white noise, unit std.
struct GM2 {
  double a = 0, y1 = 0, y2 = 0, scale = 1;
  void init(double tau, double dt, std::mt19937_64 &rng) {
    a = dt / tau;
    std::normal_distribution<double> nd;
    double s2 = 0; int N = 400000;
    for (int i = 0; i < N; i++) { step1(nd(rng)); if (i > 20000) s2 += y2 * y2; }
    scale = 1 / sqrt(s2 / (N - 20001)); y1 = y2 = 0;
  }
  void step1(double w) { y1 += a * (w - y1); y2 += a * (y1 - y2); }
  double step(double w) { step1(w); return y2 * scale; }
};

struct Truth {
  double t, pitch, roll, yaw;        // deg
  double p, q, r;                    // NED body rates, deg/s
  double fx, fy, fz;                 // NED body specific force, g
  double vsi, alt, tas, gamma, alpha, earthVertG;  // m/s, m, m/s, deg, deg, g
  M3 Rnb;
};

struct SensorCfg {
  double fs = 208;
  int sub = 8;                       // fine integration steps per IMU sample
  // ISM330DHCX: datasheet accel noise 60-80 ug/rtHz, gyro 5 mdps/rtHz; sampled noise over fs/2
  double accNoiseDensity = 80e-6;    // g/rtHz
  double gyrNoiseDensity = 5e-3;     // dps/rtHz
  double accBias = 0.010;            // g, per chip axis, random sign
  double gyrBias = 0.3;              // dps, per chip axis, random sign
  double accRW = 0.04e-3;            // g/rts   (~1 mg over 10 min)
  double gyrRW = 0.002;              // dps/rts (~0.05 dps over 10 min)
  double baroNoise = 0.3;            // m, white per sample, before 0.122 mbar quantisation
  bool baroQuantize = true;          // SSCSRNN1.6BA7A3 14-bit counts (~1 m/count at SL)
  double iasNoise = 0.3;             // kt
  double iasDelay = 0.23;            // s, median(15)+SMA(10) at 50 Hz group delay in SensorRead()
  double turbN = 0, turbMu = 0;      // turbulence std: load factor (g), bank (deg)
  double turbAlpha = 0;              // AOA std (deg), extra
  uint64_t seed = 1;
};

// chip <-> aircraft axis mapping matrices (aircraft "A" vector = (f_x, f_y, -f_z)/g)
struct Mount { double Ma[3][3]; double Mg[3][3]; };  // aircraft = M * chip

class Sim {
 public:
  using CmdFn = std::function<Cmd(double t, const FlightState &s)>;
  Sim(const SensorCfg &c, const FlightState &init, CmdFn cmd, const Mount &mount)
      : c_(c), s_(init), cmd_(std::move(cmd)), mount_(mount), rng_(c.seed) {
    dtf_ = 1.0 / (c_.fs * c_.sub);
    std::uniform_int_distribution<int> sgn(0, 1);
    for (int i = 0; i < 3; i++) {
      ab_[i] = c_.accBias * (sgn(rng_) ? 1 : -1);
      gb_[i] = c_.gyrBias * (sgn(rng_) ? 1 : -1);
    }
    tN_.init(0.6, dtf_, rng_); tMu_.init(1.2, dtf_, rng_); tA_.init(0.4, dtf_, rng_);
    prev_ = evalFine(); stepFine(); cur_ = evalFine(); stepFine(); next_ = evalFine();
    ias_ = init.V / ((1 + -init.D / 0.3048 / 1000 * 0.02) * KT);
    iasBuf_.assign(size_t(c_.iasDelay * 50) + 1, ias_);
  }
  double gyroBias(int i) const { return gb_[i]; }
  void setGyroBias(double b) { for (int i = 0; i < 3; i++) gb_[i] = b * (gb_[i] >= 0 ? 1 : -1); }
  void setGyroBiasVec(double x, double y, double z) { gb_[0] = x; gb_[1] = y; gb_[2] = z; }
  void setAccBiasVec(double x, double y, double z) { ab_[0] = x; ab_[1] = y; ab_[2] = z; }

  // Advance one IMU sample. Outputs int16 counts, Palt (ft) and IAS (kt) as the firmware sees them.
  void sample(int16_t acc[3], int16_t gyr[3], float *Palt, float *IAS, Truth *tr) {
    *tr = cur_;
    std::normal_distribution<double> nd;
    double sa = c_.accNoiseDensity * sqrt(c_.fs / 2), sg = c_.gyrNoiseDensity * sqrt(c_.fs / 2);
    double dt = 1 / c_.fs;
    for (int i = 0; i < 3; i++) { ab_[i] += c_.accRW * sqrt(dt) * nd(rng_); gb_[i] += c_.gyrRW * sqrt(dt) * nd(rng_); }
    // gyro: mean body rate across the sample (central difference of the attitude history)
    V3 w = rotvec(prev_.Rnb.T() * next_.Rnb);
    double span = 2 * dtf_ * 1;  // prev_/next_ are +-1 fine step around cur_
    double p = w.x / span * R2D, q = w.y / span * R2D, r = w.z / span * R2D;
    tr->p = p; tr->q = q; tr->r = r;
    double A[3] = {cur_.fx, cur_.fy, -cur_.fz};   // Gen2 aircraft accel (Ax, Ay, Az)
    double Gv[3] = {-p, -q, r};                   // Gen2 aircraft gyro (Gx, Gy, Gz)
    for (int i = 0; i < 3; i++) {                 // chip = M^T * aircraft
      double a = 0, g = 0;
      for (int k = 0; k < 3; k++) { a += mount_.Ma[k][i] * A[k]; g += mount_.Mg[k][i] * Gv[k]; }
      a += ab_[i] + sa * nd(rng_);
      g += gb_[i] + sg * nd(rng_);
      acc[i] = quant(a / (8.0 / 32768.0));
      gyr[i] = quant(g / (250.0 / 32768.0));
    }
    // static pressure: ISA, noise in altitude, quantised as the SSC sensor counts
    double hft = (cur_.alt + c_.baroNoise * nd(rng_)) / 0.3048;
    double mbar = 1013.25 * pow(1 - hft / 145366.45, 1 / 0.190284);
    if (c_.baroQuantize) { double cnt = std::round(mbar / 1000 * 8192 + 1638.4); mbar = (cnt - 1638.4) / 8192 * 1000; }
    *Palt = float(145366.45 * (1 - pow(mbar / 1013.25, 0.190284)));
    // IAS at 50 Hz with the SensorRead() filter delay; inverse of Gen2's TAS rule of thumb
    if (cur_.t >= nextIas_) {
      nextIas_ += 0.02;
      double ias = cur_.tas / ((1 + cur_.alt / 0.3048 / 1000 * 0.02) * KT) + c_.iasNoise * nd(rng_);
      iasBuf_.push_back(ias); iasBuf_.erase(iasBuf_.begin());
      ias_ = iasBuf_.front();
    }
    *IAS = float(ias_);
    for (int k = 0; k < c_.sub; k++) { prev_ = cur_; cur_ = next_; stepFine(); next_ = evalFine(); }
  }
  const FlightState &state() const { return s_; }

 private:
  static int16_t quant(double v) { double r = std::round(v); return int16_t(r > 32767 ? 32767 : r < -32768 ? -32768 : r); }
  Cmd turbCmd(double t, const FlightState &s) {
    Cmd c = cmd_(t, s);
    if (std::isnan(c.n)) {  // flight-path hold
      double nh = (cos(s.gam) + 0.8 * (c.gammaCmd - s.gam) * s.V / G) / cos(c.mu);
      c.n = std::max(-1.0, std::min(4.0, nh));
    }
    c.n += turbN_; c.mu += turbMu_;
    return c;
  }
  struct D6 { double V, gam, chi, N, E, D; };
  D6 deriv(const FlightState &s, const Cmd &c) {
    D6 d;
    d.V = c.Vdot;
    d.gam = G * (c.n * cos(c.mu) - cos(s.gam)) / s.V;
    d.chi = G * c.n * sin(c.mu) / (s.V * cos(s.gam));
    d.N = s.V * cos(s.gam) * cos(s.chi);
    d.E = s.V * cos(s.gam) * sin(s.chi);
    d.D = -s.V * sin(s.gam);
    return d;
  }
  void stepFine() {
    std::normal_distribution<double> nd;
    turbN_ = c_.turbN * tN_.step(nd(rng_));
    turbMu_ = c_.turbMu * D2R * tMu_.step(nd(rng_));
    turbA_ = c_.turbAlpha * D2R * tA_.step(nd(rng_));
    double h = dtf_;
    auto add = [](FlightState s, const D6 &d, double k) { s.V += d.V * k; s.gam += d.gam * k; s.chi += d.chi * k; s.N += d.N * k; s.E += d.E * k; s.D += d.D * k; s.t += k; return s; };
    FlightState s0 = s_;
    D6 k1 = deriv(s0, turbCmd(s0.t, s0));
    FlightState s1 = add(s0, k1, h / 2); D6 k2 = deriv(s1, turbCmd(s1.t, s1));
    FlightState s2 = add(s0, k2, h / 2); D6 k3 = deriv(s2, turbCmd(s2.t, s2));
    FlightState s3 = add(s0, k3, h);     D6 k4 = deriv(s3, turbCmd(s3.t, s3));
    D6 d{(k1.V + 2 * k2.V + 2 * k3.V + k4.V) / 6, (k1.gam + 2 * k2.gam + 2 * k3.gam + k4.gam) / 6, (k1.chi + 2 * k2.chi + 2 * k3.chi + k4.chi) / 6,
         (k1.N + 2 * k2.N + 2 * k3.N + k4.N) / 6, (k1.E + 2 * k2.E + 2 * k3.E + k4.E) / 6, (k1.D + 2 * k2.D + 2 * k3.D + k4.D) / 6};
    s_ = add(s0, d, h);
  }
  Truth evalFine() {
    Cmd c = turbCmd(s_.t, s_);
    Truth t{};
    t.t = s_.t;
    double Vkt = s_.V / KT;
    double alpha = std::isnan(c.alphaOv) ? std::min(25.0, 43277.0 * c.n / (Vkt * Vkt)) * D2R + turbA_  // RV-4-like lift curve
                                         : c.alphaOv;
    M3 Rnw = Rz(s_.chi) * Ry(s_.gam) * Rx(c.mu);
    M3 Rwb = Rz(c.betay) * Ry(alpha) * Rx(c.dphi);
    t.Rnb = Rnw * Rwb;
    V3 fw{c.Vdot + G * sin(s_.gam), 0, -c.n * G};
    V3 fb = mul(Rwb.T(), fw);
    t.fx = fb.x / G; t.fy = fb.y / G; t.fz = fb.z / G;
    t.pitch = asin(-t.Rnb.m[2][0]) * R2D;
    t.roll = atan2(t.Rnb.m[2][1], t.Rnb.m[2][2]) * R2D;
    t.yaw = atan2(t.Rnb.m[1][0], t.Rnb.m[0][0]) * R2D;
    t.vsi = s_.V * sin(s_.gam);
    t.alt = -s_.D;
    t.tas = s_.V;
    t.gamma = s_.gam * R2D;
    t.alpha = alpha * R2D;
    // earth vertical acceleration (up, g): -(f_n.z + g)/g... f_n = R f_b ; a_n = f_n + g_n
    V3 fn = mul(Rnw, fw);
    t.earthVertG = -(fn.z + G) / G;
    return t;
  }

  SensorCfg c_;
  FlightState s_;
  CmdFn cmd_;
  Mount mount_;
  std::mt19937_64 rng_;
  double dtf_;
  double ab_[3], gb_[3];
  GM2 tN_, tMu_, tA_;
  double turbN_ = 0, turbMu_ = 0, turbA_ = 0;
  Truth prev_{}, cur_{}, next_{};
  std::vector<double> iasBuf_;
  double ias_ = 0, nextIas_ = 0;
};

}  // namespace sim
