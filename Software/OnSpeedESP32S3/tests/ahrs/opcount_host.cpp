// Dynamic float-operation count for one processAHRS() call: compiles the unmodified port
// (ahrs.cpp) with 'float' replaced by a counting wrapper type. Host only.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <math.h>
#include <stdint.h>
#include <string.h>

struct Cnt { long add, mul, div, cmp, neg, cvt, fabs_, sqrt_, sin_, cos_, atan2_, asin_, pow_; };
static Cnt g{};

struct CF {
  float v;
  constexpr CF() : v(0) {}
  constexpr CF(float x) : v(x) {}
  constexpr CF(double x) : v(float(x)) {}
  CF(int x) : v(float(x)) { g.cvt++; }
  CF(long x) : v(float(x)) { g.cvt++; }
  CF(short x) : v(float(x)) { g.cvt++; }
  CF &operator+=(CF o) { g.add++; v += o.v; return *this; }
  CF &operator-=(CF o) { g.add++; v -= o.v; return *this; }
  CF &operator*=(CF o) { g.mul++; v *= o.v; return *this; }
  CF &operator/=(CF o) { g.div++; v /= o.v; return *this; }
  explicit operator int() const { g.cvt++; return int(v); }
};
inline CF operator+(CF a, CF b) { g.add++; return CF(a.v + b.v); }
inline CF operator-(CF a, CF b) { g.add++; return CF(a.v - b.v); }
inline CF operator*(CF a, CF b) { g.mul++; return CF(a.v * b.v); }
inline CF operator/(CF a, CF b) { g.div++; return CF(a.v / b.v); }
inline CF operator-(CF a) { g.neg++; return CF(-a.v); }
inline bool operator<(CF a, CF b) { g.cmp++; return a.v < b.v; }
inline bool operator>(CF a, CF b) { g.cmp++; return a.v > b.v; }
inline bool operator<=(CF a, CF b) { g.cmp++; return a.v <= b.v; }
inline bool operator>=(CF a, CF b) { g.cmp++; return a.v >= b.v; }
inline bool operator==(CF a, CF b) { g.cmp++; return a.v == b.v; }
inline bool operator!=(CF a, CF b) { g.cmp++; return a.v != b.v; }
inline CF fabsf(CF a) { g.fabs_++; return CF(std::fabs(a.v)); }
inline CF sqrtf(CF a) { g.sqrt_++; return CF(std::sqrt(a.v)); }
inline CF sinf(CF a) { g.sin_++; return CF(std::sin(a.v)); }
inline CF cosf(CF a) { g.cos_++; return CF(std::cos(a.v)); }
inline CF atan2f(CF a, CF b) { g.atan2_++; return CF(std::atan2(a.v, b.v)); }
inline CF asinf(CF a) { g.asin_++; return CF(std::asin(a.v)); }
inline CF powf(CF a, CF b) { g.pow_++; return CF(std::pow(a.v, b.v)); }
inline bool isfinite(CF a) { return std::isfinite(a.v); }

#define float CF
#include "ahrs.cpp"
#undef float

using namespace onspeed;

static void report(const char *name, const Cnt &c, int n) {
  double d = n;
  printf("%-34s add/sub %5.1f  mul %5.1f  div %4.1f  cmp %4.1f  neg %4.1f  int->float %3.1f  fabs %3.1f  atan2f %3.1f  asinf %3.1f  sin/cos %3.1f  powf %3.1f  sqrtf %3.1f\n",
         name, c.add / d, c.mul / d, c.div / d, c.cmp / d, c.neg / d, c.cvt / d, c.fabs_ / d, c.atan2_ / d, c.asin_ / d,
         (c.sin_ + c.cos_) / d, c.pow_ / d, c.sqrt_ / d);
}

int main() {
  const int N = 10000;
  for (int mode = 0; mode < 3; mode++) {
    AhrsConfig ac;
    ac.fixKnownBugs = mode >= 1;
    ac.aoaCompensation = mode >= 2;
    Ahrs a(ac);
    AircraftImu imu; imu.Ax = 0.05f; imu.Ay = -0.5f; imu.Az = 0.86f; imu.Gx = 1; imu.Gy = -5; imu.Gz = 9;
    a.begin(imu, 3000);
    g = Cnt{};
    for (int i = 0; i < N; i++) {
      imu.Ax = 0.05f + 0.001f * float(i % 7);
      a.processAHRS(imu, 100, 3000 + float(i % 3), 15, 5);
    }
    const char *nm[3] = {"processAHRS (original)", "processAHRS (fixKnownBugs)", "processAHRS (fixes + aoaComp)"};
    report(nm[mode], g, N);
  }
  {
    Madgwick f; f.begin(208, 3, 20);
    g = Cnt{};
    for (int i = 0; i < N; i++) f.updateIMU(1, -5, 9, 0.05f, -0.5f, 0.86f);
    report("  of which Madgwick::updateIMU", g, N);
    g = Cnt{};
    for (int i = 0; i < N; i++) { f.updateIMU(1, -5, 9, 0.05f, -0.5f, 0.86f); (void)f.getPitch(); }
    Cnt c = g; Cnt u{}; // subtract updateIMU
    (void)u;
    printf("  computeAngles = 2 atan2f + 1 asinf + ~10 mul/add per update (roll, pitch and the unused yaw)\n");
    (void)c;
  }
  {
    KalmanFilter k; k.Configure(0.79078f, 26.0638f, 1e-11f, 900, 0, 0);
    CF z, v;
    g = Cnt{};
    for (int i = 0; i < N; i++) k.Update(900, 0.1f, 1.0f / 208, &z, &v);
    report("  of which KalmanFilter::Update", g, N);
  }
  return 0;
}
