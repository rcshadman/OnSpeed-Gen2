// Host test for the OnSpeed Gen2 AHRS port (components/onspeed_core/ahrs.cpp).
// Build/run: make run   (see Makefile). Prints a markdown results table and writes results_table.md.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "ahrs.h"
#include "sim.h"

using namespace onspeed;
using sim::D2R;
using sim::KT;
using sim::R2D;

static const double FS = 208;

// ---------------------------------------------------------------- result table
struct Row { std::string id, name, metric, thr, meas; int verdict; };  // 1 pass, 0 fail, 2 info
static std::vector<Row> g_rows;
static char buf_[256];
static const char *fmt(const char *f, double v) { snprintf(buf_, sizeof buf_, f, v); return buf_; }
static void row(const char *id, const char *name, const char *metric, const char *thr, double meas,
                const char *mfmt, int verdict) {
  g_rows.push_back({id, name, metric, thr, fmt(mfmt, meas), verdict});
}
static void check(const char *id, const char *name, const char *metric, double meas, double lim,
                  const char *mfmt = "%.2f", bool lessThan = true) {
  char t[64]; snprintf(t, sizeof t, lessThan ? "< %g" : "> %g", lim);
  bool ok = std::isfinite(meas) && (lessThan ? meas < lim : meas > lim);
  row(id, name, metric, t, meas, mfmt, ok ? 1 : 0);
}
static void info(const char *id, const char *name, const char *metric, double meas, const char *mfmt = "%.2f") {
  row(id, name, metric, "info", meas, mfmt, 2);
}

// ---------------------------------------------------------------- mounting
static sim::Mount mountFromMap(const AxisMap &m) {
  sim::Mount M{};
  const AxisSpec rows[3] = {m.forward, m.lateral, m.vertical};  // Ax, Ay, Az
  for (int r = 0; r < 3; r++) {
    M.Ma[r][rows[r].axis] = rows[r].sign;
    M.Mg[r][rows[r].axis] = -rows[r].sign;  // getGyroForAxis negates
  }
  return M;
}
// Box misaligned by pitch dth (nose up) and roll dph (right wing down) vs the airframe.
static sim::Mount misalign(const sim::Mount &m, double dth, double dph) {
  sim::M3 R = sim::Ry(dth) * sim::Rx(dph);  // box axes in aircraft (NED) coordinates
  sim::M3 S = sim::M3::I(); S.m[2][2] = -1;  // NED <-> Gen2 (fwd,right,up) components
  sim::M3 T = S * R.T() * S;                // aircraft Gen2 vector -> box Gen2 vector
  sim::Mount o{};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++) { o.Ma[i][j] += T.m[k][i] * m.Ma[k][j]; o.Mg[i][j] += T.m[k][i] * m.Mg[k][j]; }
  return o;
}

// ---------------------------------------------------------------- runner
struct Rec {
  double t; sim::Truth tr;
  float pitch, roll, vsi, fpa, alt, evg, daoa;
  bool finite;
};

struct Run {
  AhrsConfig ac;
  sim::SensorCfg sc;
  sim::FlightState init;
  sim::Sim::CmdFn cmd;
  double dur = 60;
  const char *ports = "FORWARD", *top = "UP";
  GyroBias cal;
  bool calibrateGyro = false;      // Gen2 sensor calibration: gxBias=-mean(gx) at t=0 (perfect)
  double misPitch = 0, misRoll = 0;  // box misalignment, deg
  double restartAt = -1;           // in-flight reboot time
  std::function<void(double t, AircraftImu &, float &IAS, float &Palt)> inject;
  double gyroBiasOverride = -1;    // dps, all axes, random signs kept
};

static std::vector<Rec> run(const Run &r, Ahrs *out = nullptr) {
  AxisMap map = configureAxes(r.ports, r.top);
  sim::Mount m = mountFromMap(map);
  if (r.misPitch != 0 || r.misRoll != 0) m = misalign(m, r.misPitch * D2R, r.misRoll * D2R);
  sim::Sim S(r.sc, r.init, r.cmd, m);
  if (r.gyroBiasOverride >= 0) S.setGyroBias(r.gyroBiasOverride);
  GyroBias cal = r.cal;
  if (r.calibrateGyro) { cal.gxBias = float(-S.gyroBias(0)); cal.gyBias = float(-S.gyroBias(1)); cal.gzBias = float(-S.gyroBias(2)); }
  Ahrs ahrs(r.ac);
  std::vector<Rec> rec;
  int N = int(r.dur * FS);
  rec.reserve(size_t(N));
  int restartK = r.restartAt >= 0 ? int(r.restartAt * FS) : -1;
  for (int k = 0; k < N; k++) {
    int16_t acc[3], gyr[3]; float Palt, IAS; sim::Truth tr;
    S.sample(acc, gyr, &Palt, &IAS, &tr);
    AircraftImu a = mapAxes(map, scaleImu(acc, gyr), cal);
    if (r.inject) r.inject(tr.t, a, IAS, Palt);
    if (k == 0 || k == restartK) { ahrs = Ahrs(r.ac); ahrs.begin(a, Palt); }
    // AOA for the experimental aoaCompensation option: truth + 0.5 deg calibration error
    ahrs.processAHRS(a, IAS, Palt, 15, float(tr.alpha + 0.5));
    const AhrsState &s = ahrs.state();
    Rec e{tr.t, tr, s.smoothedPitch, s.smoothedRoll, s.kalmanVSI, s.flightPath, s.kalmanAlt, s.earthVertG, s.derivedAOA, true};
    e.finite = std::isfinite(s.smoothedPitch) && std::isfinite(s.smoothedRoll) && std::isfinite(s.kalmanVSI) &&
               std::isfinite(s.flightPath) && std::isfinite(s.q[0]) && std::isfinite(s.kalmanAlt);
    rec.push_back(e);
  }
  if (out) *out = ahrs;
  return rec;
}

static double wrap(double a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
struct Stat { double rms = 0, maxabs = 0, mean = 0; int n = 0; };
static Stat stat(const std::vector<Rec> &v, double t0, double t1, const std::function<double(const Rec &)> &f) {
  Stat s; double sum = 0, sq = 0;
  for (auto &e : v) {
    if (e.t < t0 || e.t > t1) continue;
    double x = f(e);
    if (!std::isfinite(x)) { s.maxabs = NAN; s.rms = NAN; s.mean = NAN; return s; }
    sum += x; sq += x * x; s.n++;
    if (fabs(x) > s.maxabs) s.maxabs = fabs(x);
  }
  if (s.n) { s.mean = sum / s.n; s.rms = sqrt(sq / s.n); }
  return s;
}
static auto ePitch = [](const Rec &e) { return double(e.pitch) - e.tr.pitch; };
static auto eRoll = [](const Rec &e) { return wrap(double(e.roll) - e.tr.roll); };
static auto eAtt = [](const Rec &e) { return std::max(fabs(double(e.pitch) - e.tr.pitch), fabs(wrap(double(e.roll) - e.tr.roll))); };
static auto eVsiFpm = [](const Rec &e) { return (double(e.vsi) - e.tr.vsi) * 196.85; };
static auto eFpa = [](const Rec &e) { return double(e.fpa) - e.tr.gamma; };
static auto eAoa = [](const Rec &e) { return double(e.daoa) - e.tr.alpha; };
static auto eEvg = [](const Rec &e) { return double(e.evg) - e.tr.earthVertG; };

// time after t0 at which err drops below lim and stays below until t1
static double settle(const std::vector<Rec> &v, double t0, double t1, double lim, const std::function<double(const Rec &)> &f) {
  double last = t0;
  for (auto &e : v) {
    if (e.t < t0 || e.t > t1) continue;
    if (!(fabs(f(e)) < lim)) last = e.t;
  }
  return last - t0;
}
// delay (s) that best aligns estimate with truth: min_d RMS(est(t) - truth(t-d))
static double lag(const std::vector<Rec> &v, double t0, double t1, const std::function<double(const Rec &)> &est,
                  const std::function<double(const Rec &)> &truth) {
  double best = 1e30, bestd = 0;
  for (int d = 0; d < int(4 * FS); d += 2) {
    double sq = 0; int n = 0;
    for (size_t i = 0; i < v.size(); i++) {
      if (v[i].t < t0 || v[i].t > t1 || int(i) < d) continue;
      double x = est(v[i]) - truth(v[i - size_t(d)]);
      sq += x * x; n++;
    }
    if (n && sq / n < best) { best = sq / n; bestd = d / FS; }
  }
  return bestd;
}

// ---------------------------------------------------------------- scenarios
static sim::FlightState fs(double Vkt, double altFt = 3000) { sim::FlightState s; s.V = Vkt * KT; s.D = -altFt * 0.3048; return s; }
static sim::Cmd levelCmd(double, const sim::FlightState &) { return sim::Cmd(); }
static sim::Sim::CmdFn turnCmd(double bankDeg, double tIn, double tOut, double dphiDeg = 0) {
  return [=](double t, const sim::FlightState &) {
    sim::Cmd c;
    double up = sim::smooth(t, tIn, tIn + 3, 0, 1) - sim::smooth(t, tOut, tOut + 3, 0, 1);
    c.mu = bankDeg * D2R * up;
    c.dphi = dphiDeg * D2R * up;
    return c;
  };
}
static const double CLIMB_G = asin(1000 / 196.85 / (90 * KT));  // 1000 fpm at 90 kt
static sim::Cmd climbCmd(double t, const sim::FlightState &) {
  sim::Cmd c; c.gammaCmd = sim::smooth(t, 20, 23, 0, CLIMB_G) - sim::smooth(t, 100, 103, 0, CLIMB_G); return c;
}
static sim::Cmd pullupCmd(double t, const sim::FlightState &s) {
  sim::Cmd c;
  if (t >= 10 && t < 14.5) c.n = t < 11 ? sim::smooth(t, 10, 11, 1, 2) : t < 13.5 ? 2 : sim::smooth(t, 13.5, 14.5, 2, cos(s.gam));
  else c.gammaCmd = t < 10 ? 0 : sim::smooth(t, 20, 28, 20 * D2R, 0);
  c.Vdot = -sim::G * sin(s.gam) * 0.6;
  return c;
}
static sim::Sim::CmdFn stallCmd(double climbDeg) {
  return [=](double t, const sim::FlightState &) {
    sim::Cmd c;
    c.gammaCmd = t < 43 ? sim::smooth(t, 5, 8, 0, climbDeg * D2R) : sim::smooth(t, 43, 45, climbDeg * D2R, -3 * D2R);
    c.Vdot = (t > 10 && t < 38) ? -1 * KT : (t > 43 && t < 63) ? 1.0 : 0;
    return c;
  };
}
static sim::Cmd slipCmd(double t, const sim::FlightState &) {
  sim::Cmd c; double up = sim::smooth(t, 10, 13, 0, 1) - sim::smooth(t, 43, 46, 0, 1);
  c.gammaCmd = -5 * D2R; c.dphi = 12 * D2R * up; c.betay = 12 * D2R * up; return c;
}
static sim::Cmd patternCmd(double t, const sim::FlightState &) {  // alternating 30 deg turns, 10 min
  sim::Cmd c; int seg = int(t / 60); double ph = t - seg * 60;
  double dir = (seg % 2) ? -1 : 1;
  c.mu = dir * 30 * D2R * (sim::smooth(ph, 10, 13, 0, 1) - sim::smooth(ph, 45, 48, 0, 1));
  c.gammaCmd = (seg == 3) ? 5 * D2R * (sim::smooth(ph, 5, 8, 0, 1) - sim::smooth(ph, 50, 53, 0, 1)) : 0;
  return c;
}

static Run base(sim::Sim::CmdFn cmd, double Vkt, double dur) { Run r; r.init = fs(Vkt); r.cmd = cmd; r.dur = dur; return r; }

// ---------------------------------------------------------------- tests
static void testSimSelfCheck() {
  // specific force from the model vs numerical derivative of NED velocity along the trajectory
  Run r = base(turnCmd(60, 2, 20), 100, 25);
  r.sc.accNoiseDensity = 0; r.sc.gyrNoiseDensity = 0; r.sc.accBias = 0; r.sc.gyrBias = 0; r.sc.accRW = 0; r.sc.gyrRW = 0;
  AxisMap map = configureAxes("FORWARD", "UP");
  sim::Sim S(r.sc, r.init, r.cmd, mountFromMap(map));
  std::vector<sim::Truth> tr; std::vector<sim::FlightState> st;
  for (int k = 0; k < int(r.dur * FS); k++) { int16_t a[3], g[3]; float p, i; sim::Truth t; S.sample(a, g, &p, &i, &t); tr.push_back(t); }
  // velocity in NED from truth: v_n = Rnb * v_b; use gamma/tas/alpha consistency instead: check
  // f_n + g_n == d/dt(v_n) via VSI derivative (vertical channel) and earthVertG.
  double maxe = 0;
  for (size_t k = 1; k + 1 < tr.size(); k++) {
    double avs = (tr[k + 1].vsi - tr[k - 1].vsi) * FS / 2 / sim::G;  // vertical accel (up), g
    maxe = std::max(maxe, fabs(avs - tr[k].earthVertG));
  }
  check("S1", "Sim self-check (60 deg turn)", "max |d(VSI)/dt - earthVertG| (g)", maxe, 1e-3, "%.2e");
  // body-frame consistency: earthVertG == (Rnb f_b).up - 1
  double maxe2 = 0;
  for (auto &t : tr) {
    sim::V3 fn = sim::mul(t.Rnb, sim::V3{t.fx, t.fy, t.fz});
    maxe2 = std::max(maxe2, fabs(-(fn.z + 1) - t.earthVertG));
  }
  check("S2", "Sim self-check", "max |Rnb*f_b vs wind-axis f| (g)", maxe2, 1e-9, "%.2e");
}

static void testAxisTable() {
  const char *dirs[6] = {"FORWARD", "AFT", "LEFT", "RIGHT", "UP", "DOWN"};
  const double vec[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};  // NED
  for (int type = 0; type < 2; type++) {
    ImuType it = type ? ImuType::LSM9DS1 : ImuType::ISM330DHCX;
    struct E { int a, b, pa, ps, ta, ts, det; AxisMap m; };
    std::vector<E> es;
    for (int a = 0; a < 6; a++)
      for (int b = 0; b < 6; b++) {
        AxisMap m = configureAxes(dirs[a], dirs[b], it);
        if (!m.valid) continue;
        auto chipFor = [&](int d, int &ax, int &sg) {  // chip axis (and sign) pointing in direction d
          AxisSpec x = (d <= 1) ? m.forward : (d <= 3) ? m.lateral : m.vertical;
          bool neg = (d == 1 || d == 2 || d == 5);  // AFT, LEFT, DOWN
          ax = x.axis; sg = neg ? -x.sign : x.sign;
        };
        E e{a, b, 0, 0, 0, 0, 0, m};
        chipFor(a, e.pa, e.ps); chipFor(b, e.ta, e.ts);
        int M[3][3] = {};
        M[0][m.forward.axis] = m.forward.sign; M[1][m.lateral.axis] = m.lateral.sign; M[2][m.vertical.axis] = m.vertical.sign;
        e.det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
        es.push_back(e);
      }
    // consensus: the box-fixed chip axes of the ports face and the top face, and the handedness
    int best = 0, bi = 0;
    for (size_t i = 0; i < es.size(); i++) {
      int c = 0;
      for (auto &f : es) c += f.pa == es[i].pa && f.ps == es[i].ps && f.ta == es[i].ta && f.ts == es[i].ts && f.det == es[i].det;
      if (c > best) { best = c; bi = int(i); }
    }
    const E &K = es[size_t(bi)];
    std::string bad;
    int mapOk = 0;
    for (auto &e : es) {
      bool same = e.pa == K.pa && e.ps == K.ps && e.ta == K.ta && e.ts == K.ts && e.det == K.det;
      if (!same) { bad += bad.empty() ? "" : ", "; bad += std::string(dirs[e.a]) + "/" + dirs[e.b]; }
      // physical chip axes for this mounting from the consensus box geometry
      double dir[3][3] = {};
      for (int i = 0; i < 3; i++) { dir[K.pa][i] = K.ps * vec[e.a][i]; dir[K.ta][i] = K.ts * vec[e.b][i]; }
      int k = 3 - K.pa - K.ta;
      const double *u = dir[(k + 1) % 3], *w = dir[(k + 2) % 3];
      double hand = K.det < 0 ? 1 : -1;  // det(fwd,right,up)=-1 <=> right-handed chip
      dir[k][0] = hand * (u[1] * w[2] - u[2] * w[1]); dir[k][1] = hand * (u[2] * w[0] - u[0] * w[2]); dir[k][2] = hand * (u[0] * w[1] - u[1] * w[0]);
      double f[3] = {0.3, -0.2, -0.9}, om[3] = {11, -7, 5};  // NED body specific force (g), rates (dps)
      ImuSample smp;
      float ca[3], cg[3];
      for (int i = 0; i < 3; i++) { ca[i] = float(dir[i][0] * f[0] + dir[i][1] * f[1] + dir[i][2] * f[2]); cg[i] = float(dir[i][0] * om[0] + dir[i][1] * om[1] + dir[i][2] * om[2]); }
      smp.ax = ca[0]; smp.ay = ca[1]; smp.az = ca[2]; smp.gx = cg[0]; smp.gy = cg[1]; smp.gz = cg[2];
      AircraftImu A = mapAxes(e.m, smp);
      // chip handedness only matters for the gyro: a left-handed chip reports -omega
      double gs = K.det < 0 ? 1 : -1;
      double err = std::max({fabs(A.Ax - f[0]), fabs(A.Ay - f[1]), fabs(A.Az + f[2])});
      if (K.det < 0) err = std::max({err, fabs(A.Gx + gs * om[0]), fabs(A.Gy + gs * om[1]), fabs(A.Gz - gs * om[2])});
      if (err < 1e-5) mapOk++;
    }
    const char *nm = type ? "LSM9DS1" : "ISM330DHCX";
    char name[128], meas[160];
    snprintf(name, sizeof name, "Axis table %s: entries found", nm);
    check(type ? "A4" : "A1", name, "orientation pairs in table (of 24)", double(es.size()), 23.5, "%.0f", false);
    snprintf(name, sizeof name, "Axis table %s: entries agreeing on box geometry + handedness", nm);
    check(type ? "A5" : "A2", name, "consistent entries (of 24)", best, 23.5, "%.0f", false);
    snprintf(meas, sizeof meas, "%d (bad: %s; ports=%c%c top=%c%c, %s chip)", best, bad.empty() ? "none" : bad.c_str(), K.ps > 0 ? '+' : '-',
             "XYZ"[K.pa], K.ts > 0 ? '+' : '-', "XYZ"[K.ta], K.det < 0 ? "right-handed" : "left-handed");
    g_rows.back().meas = meas;
    snprintf(name, sizeof name, "Axis map %s, physically generated chip data", nm);
    check(type ? "A6" : "A3", name, type ? "entries giving Ax=fx,Ay=fy,Az=-fz (accel only)" : "entries giving Ax=fx,Ay=fy,Az=-fz,Gx=-p,Gy=-q,Gz=r", mapOk, 23.5, "%.0f", false);
  }
}

// ---------------------------------------------------------------- configuration matrix
// C0 Gen2 as-is: faithful port, gyro bias 0.3 dps uncalibrated, accel bias 10 mg (task spec)
// C1 Gen2 + its ground gyro calibration (gxBias = -mean), bias drift remains
// C2 C1 + fixKnownBugs (Kalman constrain, roll-init sign, NaN guards, install typo)
// C3 C2 + experimental AOA-aware centripetal compensation (AOA = truth + 0.5 deg)
constexpr int NCFG = 4;
static const char *kCfgName[NCFG] = {"C0 Gen2 as-is", "C1 +gyro cal", "C2 +fixes", "C3 +AOA comp"};
static Run cfg(Run r, int c) {
  r.calibrateGyro = c >= 1;
  r.ac.fixKnownBugs = c >= 2;
  r.ac.aoaCompensation = c >= 3;
  return r;
}
struct MRow { std::string id, name, metric, thr; std::string val[NCFG]; int verdict[NCFG]; };
static std::vector<MRow> g_mat;
// one metric over all configurations; lim<0 -> info only
static void mcheck(const char *id, const char *name, const char *metric, const double v[NCFG], double lim, const char *f = "%.2f") {
  MRow m{id, name, metric, lim < 0 ? "info" : std::string("< ") + fmt("%g", lim), {}, {}};
  for (int c = 0; c < NCFG; c++) {
    m.val[c] = fmt(f, v[c]);
    m.verdict[c] = lim < 0 ? 2 : (std::isfinite(v[c]) && v[c] < lim) ? 1 : 0;
  }
  g_mat.push_back(m);
}
using Metric = std::function<double(const std::vector<Rec> &)>;
struct MSpec { const char *id, *metric; double lim; Metric f; const char *fmt = "%.2f"; };
static void scenario(const char *name, const Run &r0, const std::vector<MSpec> &specs) {
  std::vector<std::vector<double>> vals(specs.size(), std::vector<double>(NCFG));
  for (int c = 0; c < NCFG; c++) {
    auto v = run(cfg(r0, c));
    for (size_t i = 0; i < specs.size(); i++) vals[i][size_t(c)] = specs[i].f(v);
  }
  for (size_t i = 0; i < specs.size(); i++) mcheck(specs[i].id, name, specs[i].metric, vals[i].data(), specs[i].lim, specs[i].fmt);
}
static Metric rmsOf(double t0, double t1, std::function<double(const Rec &)> e) { return [=](const std::vector<Rec> &v) { return stat(v, t0, t1, e).rms; }; }
static Metric maxOf(double t0, double t1, std::function<double(const Rec &)> e) { return [=](const std::vector<Rec> &v) { return stat(v, t0, t1, e).maxabs; }; }
static Metric meanAbsOf(double t0, double t1, std::function<double(const Rec &)> e) { return [=](const std::vector<Rec> &v) { return fabs(stat(v, t0, t1, e).mean); }; }
static Metric atT(double t, std::function<double(const Rec &)> e) { return [=](const std::vector<Rec> &v) { return fabs(e(v[size_t(t * FS)])); }; }

static void flightTests() {
  {
    Run r = base([](double, const sim::FlightState &) { sim::Cmd c; c.alphaOv = 11 * D2R; c.dphi = 1.5 * D2R; return c; }, 0, 60);
    r.init.V = 0.01;
    scenario("Power-up on ground, 11 deg nose up (taildragger), 60 s", r,
             {{"P1", "attitude err at first sample (deg)", 1.0, atT(1.0 / FS, eAtt)},
              {"P2", "max attitude err 0-60 s (deg)", 1.5, maxOf(0, 60, eAtt)},
              {"P3", "max |VSI| (IAS<25 -> 0) (fpm)", 1, maxOf(0, 60, eVsiFpm)}});
  }
  scenario("Level cruise 120 kt, 120 s", base(levelCmd, 120, 120),
           {{"L1", "pitch RMS err (deg)", 0.75, rmsOf(5, 120, ePitch)},
            {"L2", "roll RMS err (deg)", 0.75, rmsOf(5, 120, eRoll)},
            {"L3", "max attitude err (deg)", 1.5, maxOf(5, 120, eAtt)},
            {"L4", "VSI RMS err (fpm)", 50, rmsOf(20, 120, eVsiFpm), "%.1f"}});
  scenario("Climb 0->1000 fpm @ 90 kt (t=20-23 s), level at 100 s", base(climbCmd, 90, 140),
           {{"C1", "pitch max err, steady climb (deg)", 1.0, maxOf(40, 100, ePitch)},
            {"C2", "VSI mean err, steady climb (fpm)", 50, meanAbsOf(40, 100, eVsiFpm), "%.1f"},
            {"C3", "VSI RMS err, steady climb (fpm)", 75, rmsOf(40, 100, eVsiFpm), "%.1f"},
            {"C4", "VSI max err, climb entry 15-45 s (fpm)", 300, maxOf(15, 45, eVsiFpm), "%.0f"},
            {"C5", "VSI lag vs truth, best-fit delay (s)", 1.5,
             [](const std::vector<Rec> &v) { return lag(v, 15, 45, [](const Rec &e) { return double(e.vsi); }, [](const Rec &e) { return e.tr.vsi; }); }},
            {"C6", "flight path angle RMS err (deg)", 0.5, rmsOf(40, 100, eFpa)},
            {"C7", "derived AOA (pitch - FPA) RMS err (deg)", 1.0, rmsOf(40, 100, eAoa)},
            {"C8", "earthVertG RMS err (g)", 0.02, rmsOf(10, 140, eEvg), "%.4f"}});
  for (int bank : {45, 60}) {
    char nm[96]; snprintf(nm, sizeof nm, "%d deg coordinated level turn, 90 s (%.1f orbits), 100 kt", bank,
                          90 * sim::G * tan(bank * D2R) / (100 * KT) * R2D / 360);
    double lim = bank == 45 ? 2.0 : 3.0;
    bool b45 = bank == 45;
    scenario(nm, base(turnCmd(bank, 10, 100), 100, 130),
             {{b45 ? "T1" : "T6", "roll max err, steady turn 20-100 s (deg)", lim, maxOf(20, 100, eRoll)},
              {b45 ? "T2" : "T7", "pitch max err, steady turn (deg)", lim, maxOf(20, 100, ePitch)},
              {b45 ? "T3" : "T8", "max attitude err incl. roll in/out (deg)", lim + 1, maxOf(10, 130, eAtt)},
              {b45 ? "T4" : "T9", "VSI RMS err, turn + 30 s after (fpm)", 100, rmsOf(20, 130, eVsiFpm), "%.0f"},
              {b45 ? "T5" : "T10", "earthVertG RMS err in turn (g)", 0.03, rmsOf(20, 100, eEvg), "%.4f"}});
    Run nc = base(turnCmd(bank, 10, 100), 100, 130);
    nc.ac.compensation = false;
    scenario(b45 ? "45 deg turn, accel compensation DISABLED (classic Madgwick)" : "60 deg turn, accel compensation DISABLED (classic Madgwick)", nc,
             {{b45 ? "T11" : "T13", "roll max err, steady turn (deg)", -1, maxOf(20, 100, eRoll)},
              {b45 ? "T12" : "T14", "pitch max err, steady turn (deg)", -1, maxOf(20, 100, ePitch)}});
  }
  scenario("Pull-up to 2 g, 20 deg climb, push-over (120 kt)", base(pullupCmd, 120, 50),
           {{"U1", "pitch max err (deg)", 3.0, maxOf(5, 50, ePitch)},
            {"U2", "roll max err (deg)", 2.0, maxOf(5, 50, eRoll)},
            {"U3", "flight path angle max err (deg)", 3.0, maxOf(5, 50, eFpa)}});
  scenario("Power-off stall approach, level, 80->52 kt at 1 kt/s (pitch 16 deg)", base(stallCmd(0), 80, 70),
           {{"W1", "pitch max err (deg)", 2.0, maxOf(5, 70, ePitch)},
            {"W2", "flight path angle RMS err (deg)", 1.0, rmsOf(10, 70, eFpa)},
            {"W3", "derived AOA RMS err, approach (deg)", 1.5, rmsOf(10, 43, eAoa)}});
  scenario("Power-on stall approach, 8 deg climb, 80->52 kt (pitch 24 deg)", base(stallCmd(8), 80, 70),
           {{"W4", "pitch max err (deg)", 2.0, maxOf(5, 70, ePitch)},
            {"W5", "flight path angle RMS err (deg)", 1.0, rmsOf(10, 70, eFpa)},
            {"W6", "derived AOA RMS err, approach (deg)", 1.5, rmsOf(10, 43, eAoa)}});
  scenario("Forward slip 12 deg roll + 12 deg yaw, -5 deg path, 70 kt (0.19 g lateral)", base(slipCmd, 70, 60),
           {{"X1", "roll max err incl. entry/exit (deg)", 3.0, maxOf(5, 60, eRoll)},
            {"X2", "roll RMS err (deg)", 1.0, rmsOf(5, 60, eRoll)},
            {"X3", "pitch max err (deg)", 2.0, maxOf(5, 60, ePitch)}});
  scenario("Skidding turn: 30 deg path bank, body 22 deg (0.16 g lateral)", base(turnCmd(30, 10, 70, -8), 100, 90),
           {{"X4", "roll max err (deg)", 3.0, maxOf(5, 90, eRoll)}});
  {
    Run r = base(levelCmd, 120, 120);
    r.sc.turbN = 0.2; r.sc.turbMu = 3; r.sc.turbAlpha = 0.5;
    scenario("Moderate turbulence, 120 kt: n std 0.2 g, bank std 3 deg, AOA std 0.5 deg", r,
             {{"B1", "pitch RMS err (deg)", 1.0, rmsOf(10, 120, ePitch)},
              {"B2", "roll RMS err (deg)", 1.0, rmsOf(10, 120, eRoll)},
              {"B3", "max attitude err (deg)", 3.0, maxOf(10, 120, eAtt)},
              {"B4", "VSI RMS err (fpm)", 150, rmsOf(20, 120, eVsiFpm), "%.0f"},
              {"B5", "flight path RMS err (deg)", 1.0, rmsOf(20, 120, eFpa)}});
  }
  scenario("10 min pattern: alternating 30 deg turns, one climb, 110 kt", base(patternCmd, 110, 600),
           {{"M1", "pitch RMS err (deg)", 1.0, rmsOf(5, 600, ePitch)},
            {"M2", "roll RMS err (deg)", 1.0, rmsOf(5, 600, eRoll)},
            {"M3", "max attitude err (deg)", 3.0, maxOf(5, 600, eAtt)},
            {"M4", "VSI RMS err (fpm)", 100, rmsOf(20, 600, eVsiFpm), "%.0f"},
            {"M5", "altitude RMS err (ft)", 10, rmsOf(20, 600, [](const Rec &e) { return (e.alt - e.tr.alt) * 3.28084; })}});
  {
    Run r = base(turnCmd(45, 10, 200), 100, 160);
    r.restartAt = 50;
    scenario("In-flight reboot at t=50 s in a 45 deg turn (accel-only init)", r,
             {{"R1", "roll err right after reboot (deg)", -1, atT(50 + 2 / FS, eRoll)},
              {"R2", "time to |roll err| < 2 deg (s)", 60, [](const std::vector<Rec> &v) { return settle(v, 50, 160, 2, eRoll); }, "%.1f"}});
    Run l = base(levelCmd, 120, 60);
    l.restartAt = 30;
    scenario("In-flight reboot at t=30 s, level 120 kt (prevTAS = 0 -> TAS step)", l,
             {{"R3", "max pitch err after reboot (deg)", 2.0, maxOf(30, 60, ePitch)},
              {"R4", "time to |pitch err| < 1 deg (s)", 10, [](const std::vector<Rec> &v) { return settle(v, 30, 60, 1, ePitch); }, "%.1f"}});
  }
}

static void biasTests() {
  // FORWARD/UP mount: chip X = right (pitch axis), Y = forward (roll axis), Z = up (yaw axis)
  struct B { const char *id, *name; double x, y, z, lim; };
  const B bs[] = {
      {"G1", "Gyro bias 0.3 dps on yaw axis only, level 120 kt", 0, 0, 0.3, 1.0},
      {"G2", "Gyro bias 0.3 dps on roll+pitch axes only, level 120 kt", 0.3, -0.3, 0, 1.0},
      {"G3", "Gyro bias 0.1 dps on yaw axis only, level 120 kt", 0, 0, 0.1, 1.0},
      {"G4", "Gyro bias 1.0 dps all axes, level 120 kt", 1.0, -1.0, 1.0, -1},
      {"G5", "Gyro bias 0.9+0.9 dps roll+pitch (|b|=1.27 < slew limit 2*beta=1.33 dps), level", 0.9, -0.9, 0, -1},
      {"G6", "Gyro bias 1.0+1.0 dps roll+pitch (|b|=1.41 > slew limit), level", 1.0, -1.0, 0, -1},
  };
  for (auto &b : bs) {
    Run r = base(levelCmd, 120, 300);
    r.sc.accBias = 0; r.sc.accRW = 0; r.sc.gyrRW = 0;
    r.gyroBiasOverride = 0;
    sim::Mount m{}; (void)m;
    double bx = b.x, by = b.y, bz = b.z;
    // set biases through a wrapper run: emulate by injecting into the mapped aircraft gyro
    r.inject = [bx, by, bz](double, AircraftImu &a, float &, float &) {
      // chip X->lateral (Gy=-x), chip Y->forward (Gx=-y), chip Z->vertical (Gz=-z)
      a.Gx += float(-by); a.Gy += float(-bx); a.Gz += float(-bz);
    };
    auto v = run(r);
    double pred = atan(120 * KT * fabs(bz) * D2R / sim::G) * R2D;
    char mt[96]; snprintf(mt, sizeof mt, "max attitude err 20-300 s (deg)%s", bz != 0 ? "" : "");
    if (b.lim > 0) check(b.id, b.name, mt, stat(v, 20, 300, eAtt).maxabs, b.lim);
    else info(b.id, b.name, mt, stat(v, 20, 300, eAtt).maxabs);
    if (bz != 0 && b.id[1] != '4') {
      char id2[8]; snprintf(id2, sizeof id2, "%sp", b.id);
      info(id2, b.name, "predicted roll err atan(TAS*b_yaw/g) (deg)", pred);
    }
  }
  {
    Run r = base(levelCmd, 120, 600);
    r.calibrateGyro = true;
    auto v = run(r);
    check("G7", "Gyro calibrated at power-up (bias drift 0.002 dps/rt-s remains), level 10 min", "max attitude err (deg)", stat(v, 20, 600, eAtt).maxabs, 1.0);
  }
}

static void installTests() {
  // Box mounted 5 deg nose up, 3 deg right wing down; calibrate pitch/rollBias as WifiSerial.ino does.
  const double mp = 5, mr = 3;
  for (int fix = 0; fix < 2; fix++) {
    AxisMap map = configureAxes("FORWARD", "UP");
    sim::Mount m = misalign(mountFromMap(map), mp * D2R, mr * D2R);
    sim::SensorCfg sc; sc.accBias = 0; sc.accRW = 0;
    sim::FlightState g = fs(0); g.V = 0.01;
    sim::Sim S(sc, g, [](double, const sim::FlightState &) { sim::Cmd c; c.alphaOv = 0; return c; }, m);
    double aF = 0, aL = 0, aV = 0;
    for (int i = 0; i < 150; i++) {
      int16_t a[3], gy[3]; float p, ias; sim::Truth t; S.sample(a, gy, &p, &ias, &t);
      AircraftImu A = mapAxes(map, scaleImu(a, gy));
      aF += A.Ax; aL += A.Ay; aV += A.Az;
    }
    aF /= 150; aL /= 150; aV /= 150;
    double cp = atan2(aF, sqrt(aL * aL + aV * aV)) * 57.2957, cr = -atan2(aL, sqrt(aF * aF + aV * aV)) * 57.2957;
    Run r = base(turnCmd(45, 10, 100), 100, 130);
    r.misPitch = mp; r.misRoll = mr;
    r.ac.pitchBias = float(0 - cp); r.ac.rollBias = float(0 - cr);
    r.ac.fixKnownBugs = fix;
    r.sc.accBias = 0.0;  // isolate installation handling
    r.calibrateGyro = true;
    auto v = run(r);
    if (!fix) {
      info("I1", "Install 5 deg nose-up, 3 deg roll; calibrated", "pitchBias, rollBias (deg)", r.ac.pitchBias);
      char m2[64]; snprintf(m2, sizeof m2, "%.2f, %.2f", double(r.ac.pitchBias), double(r.ac.rollBias));
      g_rows.back().meas = m2;
      check("I2", "Install bias, level after a 45 deg turn", "max attitude err 110-130 s (deg)", stat(v, 110, 130, eAtt).maxabs, 1.0);
      check("I2b", "Install bias, 45 deg turn", "roll max err in turn (deg)", stat(v, 20, 100, eRoll).maxabs, 2.0);
      check("I3", "Install bias, power-up roll init (original setup())", "roll err at t=0.1 s (deg)", fabs(eRoll(v[20])), 1.0);
    } else {
      check("I4", "Install bias, power-up roll init with fixKnownBugs", "roll err at t=0.1 s (deg)", fabs(eRoll(v[20])), 1.0);
    }
  }
}

static void vsiTests() {
  // Kalman tuning: original (constrain() no-op) vs fixed, same climb profile
  for (int fix = 0; fix < 2; fix++) {
    Run r = base(climbCmd, 90, 140);
    r.ac.fixKnownBugs = fix;
    auto v = run(r);
    double L = lag(v, 15, 45, [](const Rec &e) { return double(e.vsi); }, [](const Rec &e) { return e.tr.vsi; });
    const char *nm = fix ? "VSI, Kalman with constrain() fix" : "VSI, Kalman as in original";
    info(fix ? "V4" : "V1", nm, "climb-entry lag (s)", L);
    info(fix ? "V5" : "V2", nm, "VSI RMS err steady climb (fpm)", stat(v, 40, 100, eVsiFpm).rms, "%.1f");
    Run t = base(levelCmd, 120, 120); t.sc.turbN = 0.2; t.sc.turbMu = 3; t.ac.fixKnownBugs = fix;
    auto w = run(t);
    info(fix ? "V6" : "V3", nm, "VSI RMS err in turbulence (fpm)", stat(w, 20, 120, eVsiFpm).rms, "%.1f");
  }
  {
    Run r = base(climbCmd, 90, 140);
    r.sc.baroQuantize = false;
    auto v = run(r);
    info("V7", "VSI, baro 0.3 m noise without 0.122 mbar quantisation", "VSI RMS err steady climb (fpm)", stat(v, 40, 100, eVsiFpm).rms, "%.1f");
  }
}

static void robustnessTests() {
  // zero accel (free fall / dead IMU) for 2 s in level flight
  {
    Run r = base(levelCmd, 120, 40);
    r.inject = [](double t, AircraftImu &a, float &, float &) { if (t > 20 && t < 22) a.Ax = a.Ay = a.Az = 0; };
    auto v = run(r);
    bool fin = true; for (auto &e : v) fin &= e.finite;
    check("N1", "Zero accel input 2 s (all axes 0 g)", "non-finite outputs (count)", fin ? 0 : 1, 0.5, "%.0f");
    info("N2", "Zero accel input 2 s", "max attitude err after (deg)", stat(v, 20, 40, eAtt).maxabs);
  }
  {
    Madgwick f; f.begin(208, 0, 0);
    bool fin = true;
    for (int i = 0; i < 2000; i++) { f.updateIMU(0, 0, 0, 0, 0, 0); f.updateIMU(0, 0, 0, 0, 0, 1); }
    float w, x, y, z; f.getQuaternion(&w, &x, &y, &z);
    fin = std::isfinite(w) && std::isfinite(f.getPitch()) && fabsf(w * w + x * x + y * y + z * z - 1) < 1e-2f;
    check("N3", "Madgwick: exact 0 accel and exact-gravity (zero gradient) inputs", "non-finite / bad |q|", fin ? 0 : 1, 0.5, "%.0f");
  }
  // large rates: injected 2000 dps on all axes for 2 s (beyond the 250 dps chip range)
  {
    Run r = base(levelCmd, 120, 80);
    r.inject = [](double t, AircraftImu &a, float &, float &) { if (t > 20 && t < 22) a.Gx = a.Gy = a.Gz = 2000; };
    auto v = run(r);
    bool fin = true; for (auto &e : v) fin &= e.finite;
    check("N4", "Gyro 2000 dps all axes for 2 s", "non-finite outputs (count)", fin ? 0 : 1, 0.5, "%.0f");
    info("N5", "Gyro 2000 dps for 2 s", "time to |att err|<2 deg after (s)", settle(v, 22, 80, 2, eAtt), "%.1f");
  }
  // saturated chip (real 250 dps limit): 300 deg/s roll for 2 s is clipped at 250
  {
    Run r = base(levelCmd, 120, 80);
    r.inject = [](double t, AircraftImu &a, float &, float &) { if (t > 20 && t < 22) a.Gx = std::max(-250.f, std::min(250.f, a.Gx - 300)); };
    auto v = run(r);
    info("N6", "Gyro saturated at 250 dps for 2 s (rate clipping)", "time to |att err|<2 deg after (s)", settle(v, 22, 80, 2, eAtt), "%.1f");
  }
  // static pressure I2C timeout: GetStaticPressure() returns -1 for one sample
  for (int fix = 0; fix < 2; fix++) {
    Run r = base(levelCmd, 120, 40);
    r.ac.fixKnownBugs = fix;
    r.inject = [](double t, AircraftImu &, float &, float &Palt) {
      if (t > 20 && t < 20 + 1.5 / FS) Palt = 145366.45f * (1 - powf((-1.0f + -2.1997f) / 1013.25f, 0.190284f));
    };
    auto v = run(r);
    bool fin = v.back().finite;
    check(fix ? "N8" : "N7", fix ? "Static sensor I2C timeout (-1 mbar), fixKnownBugs" : "Static sensor I2C timeout (-1 mbar) once, original",
          "outputs still NaN 20 s later (1=yes)", fin ? 0 : 1, 0.5, "%.0f");
    if (!fix) info("N7b", "Static sensor I2C timeout, original", "pitch/roll NaN 20 s later (1=yes)", std::isfinite(v.back().pitch) && std::isfinite(v.back().roll) ? 0 : 1, "%.0f");
  }
  // asin domain: pitot glitch IAS=26 kt in a 15 deg dive (VSI ~ -3200 fpm)
  for (int fix = 0; fix < 2; fix++) {
    Run r = base([](double t, const sim::FlightState &) { sim::Cmd c; c.gammaCmd = sim::smooth(t, 5, 8, 0, -15 * D2R); return c; }, 120, 30);
    r.ac.fixKnownBugs = fix;
    r.inject = [](double t, AircraftImu &, float &IAS, float &) { if (t > 20 && t < 21) IAS = 26; };
    auto v = run(r);
    int nan = 0; for (auto &e : v) nan += !std::isfinite(e.fpa);
    check(fix ? "N10" : "N9", fix ? "IAS glitch 26 kt in 15 deg dive, fixKnownBugs" : "IAS glitch 26 kt in 15 deg dive, original",
          "samples with NaN flightPath", nan, 0.5, "%.0f");
  }
  // 1 hour level flight: quaternion norm and SMA sum drift
  {
    Run r = base(levelCmd, 120, 3600);
    r.calibrateGyro = true;
    Ahrs a;
    auto v = run(r, &a);
    const float *q = a.state().q;
    double nq = sqrt(double(q[0]) * q[0] + double(q[1]) * q[1] + double(q[2]) * q[2] + double(q[3]) * q[3]);
    check("N11", "1 h level flight", "| |q|-1 | (fast invSqrt)", fabs(nq - 1), 2e-3, "%.2e");
    check("N12", "1 h level flight", "max attitude err last 10 min (deg)", stat(v, 3000, 3600, eAtt).maxabs, 1.5);
  }
}

int main() {
  testSimSelfCheck();
  testAxisTable();
  flightTests();
  biasTests();
  installTests();
  vsiTests();
  robustnessTests();
  int fails = 0, pass = 0, infos = 0;
  FILE *f = fopen("results_table.md", "w");
  const char *hdr = "| ID | Test | Metric | Threshold | Measured | Result |\n|---|---|---|---|---|---|\n";
  printf("%s", hdr); if (f) fputs(hdr, f);
  for (auto &r : g_rows) {
    const char *v = r.verdict == 1 ? "PASS" : r.verdict == 0 ? "**FAIL**" : "info";
    if (r.verdict == 1) pass++; else if (r.verdict == 0) fails++; else infos++;
    char line[512];
    snprintf(line, sizeof line, "| %s | %s | %s | %s | %s | %s |\n", r.id.c_str(), r.name.c_str(), r.metric.c_str(), r.thr.c_str(), r.meas.c_str(), v);
    printf("%s", line); if (f) fputs(line, f);
  }
  char mh[512];
  snprintf(mh, sizeof mh, "\n### Flight scenarios x configuration\n\n| ID | Scenario | Metric | Threshold | %s | %s | %s | %s |\n|---|---|---|---|---|---|---|---|\n",
           kCfgName[0], kCfgName[1], kCfgName[2], kCfgName[3]);
  printf("%s", mh); if (f) fputs(mh, f);
  int mf[NCFG] = {0}, mp[NCFG] = {0};
  for (auto &m : g_mat) {
    std::string line = "| " + m.id + " | " + m.name + " | " + m.metric + " | " + m.thr + " |";
    for (int c = 0; c < NCFG; c++) {
      line += " " + m.val[c] + (m.verdict[c] == 1 ? " PASS" : m.verdict[c] == 0 ? " **FAIL**" : "") + " |";
      if (m.verdict[c] == 1) mp[c]++; else if (m.verdict[c] == 0) mf[c]++;
    }
    line += "\n";
    printf("%s", line.c_str()); if (f) fputs(line.c_str(), f);
  }
  char tot[256];
  snprintf(tot, sizeof tot, "| | | **pass / fail** | | %d / %d | %d / %d | %d / %d | %d / %d |\n", mp[0], mf[0], mp[1], mf[1], mp[2], mf[2], mp[3], mf[3]);
  printf("%s", tot); if (f) fputs(tot, f);
  printf("\nsingle-config tests: %d pass, %d fail, %d info\n", pass, fails, infos);
  if (f) { fprintf(f, "\nsingle-config tests: %d pass, %d fail, %d info\n", pass, fails, infos); fclose(f); }
  return 0;  // failures are findings about the original algorithm, see RESULTS.md
}
