// Ported from OnSpeed-Gen2 v3.3.7g: AHRS.ino, IMUread.ino, MadgwickFusion.cpp, KalmanFilter.cpp,
// and the AHRS initialisation in OnSpeedTeensy_AHRS.ino setup(). See ahrs.h for frame notes.
// All math is float with f-suffixed literals and the f-variants of libm (no double on the S3).
#include "ahrs.h"
#include <math.h>
#include <string.h>

namespace onspeed {

// ---------------------------------------------------------------- IMUread.ino configureAxes()
namespace {
struct AxisRow { const char *ports, *boxtop, *vert, *lat, *fwd; };
// {portsOrientation, boxtopOrientation, verticalGloadAxis, lateralGloadAxis, forwardGloadAxis}
const AxisRow kAxisLSM9DS1[24] = {
    {"FORWARD", "LEFT", "Y", "Z", "X"},   {"FORWARD", "RIGHT", "-Y", "Z", "X"},
    {"FORWARD", "UP", "Z", "Y", "X"},     {"FORWARD", "DOWN", "-Z", "-Y", "X"},
    {"AFT", "LEFT", "-Y", "-Z", "-X"},    {"AFT", "RIGHT", "Y", "Z", "-X"},
    {"AFT", "UP", "Z", "-Y", "-X"},       {"AFT", "DOWN", "-Z", "Y", "-X"},
    {"LEFT", "FORWARD", "-Y", "-X", "Z"}, {"LEFT", "AFT", "Y", "-X", "-Z"},
    {"LEFT", "UP", "Z", "-X", "Y"},       {"LEFT", "DOWN", "-Z", "-X", "Y"},
    {"RIGHT", "FORWARD", "Y", "X", "Z"},  {"RIGHT", "AFT", "-Y", "X", "-Z"},
    {"RIGHT", "UP", "Z", "X", "-Y"},      {"RIGHT", "DOWN", "-Z", "X", "Y"},
    {"UP", "FORWARD", "X", "-Y", "Z"},    {"UP", "AFT", "X", "Y", "-Z"},
    {"UP", "LEFT", "X", "-Z", "-Y"},      {"UP", "RIGHT", "X", "Z", "Y"},
    {"DOWN", "FORWARD", "-X", "Y", "Z"},  {"DOWN", "AFT", "-X", "-Y", "-Z"},
    {"DOWN", "LEFT", "-X", "-Z", "Y"},    {"DOWN", "RIGHT", "-X", "Z", "-Y"}};
// "X and Y axes are swapped on this chip"
const AxisRow kAxisISM330[24] = {
    {"FORWARD", "LEFT", "X", "-Z", "Y"},  {"FORWARD", "RIGHT", "-X", "Z", "Y"},
    {"FORWARD", "UP", "Z", "X", "Y"},     {"FORWARD", "DOWN", "-Z", "-X", "Y"},
    {"AFT", "LEFT", "-X", "-Z", "-Y"},    {"AFT", "RIGHT", "X", "Z", "-Y"},
    {"AFT", "UP", "Z", "-X", "-Y"},       {"AFT", "DOWN", "-Z", "X", "-Y"},
    {"LEFT", "FORWARD", "-X", "-Y", "Z"}, {"LEFT", "AFT", "X", "-Y", "-Z"},
    {"LEFT", "UP", "Z", "-Y", "X"},       {"LEFT", "DOWN", "-Z", "-Y", "X"},
    {"RIGHT", "FORWARD", "X", "Y", "Z"},  {"RIGHT", "AFT", "-X", "Y", "-Z"},
    {"RIGHT", "UP", "Z", "Y", "-X"},      {"RIGHT", "DOWN", "-Z", "Y", "X"},
    {"UP", "FORWARD", "Y", "-X", "Z"},    {"UP", "AFT", "Y", "X", "-Z"},
    {"UP", "LEFT", "Y", "-Z", "-X"},      {"UP", "RIGHT", "Y", "Z", "X"},
    {"DOWN", "FORWARD", "-Y", "X", "Z"},  {"DOWN", "AFT", "-Y", "-X", "-Z"},
    {"DOWN", "LEFT", "-Y", "-Z", "X"},    {"DOWN", "RIGHT", "-Y", "Z", "-X"}};

AxisSpec parseAxis(const char *s) {
  AxisSpec a;
  size_t n = strlen(s);
  char c = s[n - 1];  // getAccelForAxis(): last char picks the axis, anything else is Z
  a.axis = c == 'X' ? 0 : c == 'Y' ? 1 : 2;
  a.sign = s[0] == '-' ? -1 : 1;
  return a;
}
}  // namespace

AxisMap configureAxes(const char *ports, const char *boxtop, ImuType type) {
  const AxisRow *t = type == ImuType::ISM330DHCX ? kAxisISM330 : kAxisLSM9DS1;
  AxisMap m;
  for (int i = 0; i < 24; i++) {
    if (strcmp(t[i].ports, ports) == 0 && strcmp(t[i].boxtop, boxtop) == 0) {
      m.vertical = parseAxis(t[i].vert);
      m.lateral = parseAxis(t[i].lat);
      m.forward = parseAxis(t[i].fwd);
      m.valid = true;
      break;
    }
  }
  // Original leaves the axis strings empty when nothing matches; getAccelForAxis("") then
  // reads s[-1] (undefined). We return valid=false and default every axis to +Z.
  return m;
}

ImuSample scaleImu(const int16_t acc[3], const int16_t gyr[3], ImuType type) {
  float aRes = type == ImuType::ISM330DHCX ? ISM330_ARES : LSM9DS1_ARES;
  float gRes = type == ImuType::ISM330DHCX ? ISM330_GRES : LSM9DS1_GRES;
  ImuSample s;
  s.ax = aRes * acc[0]; s.ay = aRes * acc[1]; s.az = aRes * acc[2];
  s.gx = gRes * gyr[0]; s.gy = gRes * gyr[1]; s.gz = gRes * gyr[2];
  return s;
}

AircraftImu mapAxes(const AxisMap &m, const ImuSample &s, const GyroBias &b) {
  const float a[3] = {s.ax, s.ay, s.az};
  const float g[3] = {s.gx + b.gxBias, s.gy + b.gyBias, s.gz + b.gzBias};
  auto acc = [&](AxisSpec x) { return x.sign < 0 ? -a[x.axis] : a[x.axis]; };
  // getGyroForAxis(): "if (gyroAxis[0]!='-') result*=-1;" i.e. every gyro is negated.
  auto gyr = [&](AxisSpec x) { return x.sign < 0 ? g[x.axis] : -g[x.axis]; };
  AircraftImu r;
  r.Az = acc(m.vertical); r.Ay = acc(m.lateral); r.Ax = acc(m.forward);
  r.Gx = gyr(m.forward);   // rollGyroAxis = forwardGloadAxis
  r.Gy = gyr(m.lateral);   // pitchGyroAxis = lateralGloadAxis
  r.Gz = gyr(m.vertical);  // yawGyroAxis = verticalGloadAxis
  return r;
}

// ---------------------------------------------------------------- AHRS.ino helpers
float calcPitch(float aFwd, float aLat, float aVert) {
  return atan2f(aFwd, sqrtf(aLat * aLat + aVert * aVert)) * AHRS_RAD2DEG;
}

float calcRoll(float aFwd, float aLat, float aVert) {
  return atan2f(aLat, sqrtf(aFwd * aFwd + aVert * aVert)) * AHRS_RAD2DEG;
}

// ---------------------------------------------------------------- MadgwickFusion.cpp
Madgwick::Madgwick() {
  beta_ = 0.011617f;  // betaDef, "2 * proportional gain"
  invSampleFreq_ = 1.0f / 208.0f;
  q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f;
}

void Madgwick::begin(float sampleFrequency, float Pitch, float Roll) {
  invSampleFreq_ = 1.0f / sampleFrequency;
  // Original: cos(Roll*DEG2RAD*0.5) etc. in double (DEG2RAD is a double macro).
  float cr2 = cosf(Roll * AHRS_DEG2RAD * 0.5f), sr2 = sinf(Roll * AHRS_DEG2RAD * 0.5f);
  float cp2 = cosf(Pitch * AHRS_DEG2RAD * 0.5f), sp2 = sinf(Pitch * AHRS_DEG2RAD * 0.5f);
  float cy2 = 1.0f, sy2 = 0.0f;  // Yaw = 0
  q0_ = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
  q1_ = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
  q2_ = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
  q3_ = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;
  anglesComputed_ = false;
}

void Madgwick::updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  gx *= 0.0174533f; gy *= 0.0174533f; gz *= 0.0174533f;

  qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
  qDot2 = 0.5f * (q0_ * gx + q2_ * gz - q3_ * gy);
  qDot3 = 0.5f * (q0_ * gy - q1_ * gz + q3_ * gx);
  qDot4 = 0.5f * (q0_ * gz + q1_ * gy - q2_ * gx);

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    _2q0 = 2.0f * q0_; _2q1 = 2.0f * q1_; _2q2 = 2.0f * q2_; _2q3 = 2.0f * q3_;
    _4q0 = 4.0f * q0_; _4q1 = 4.0f * q1_; _4q2 = 4.0f * q2_;
    _8q1 = 8.0f * q1_; _8q2 = 8.0f * q2_;
    q0q0 = q0_ * q0_; q1q1 = q1_ * q1_; q2q2 = q2_ * q2_; q3q3 = q3_ * q3_;

    // Gradient descent corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_ - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3_ - _2q1 * ax + 4.0f * q2q2 * q3_ - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // s==0 gives ~3e19*0 = 0, no NaN
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    qDot1 -= beta_ * s0; qDot2 -= beta_ * s1; qDot3 -= beta_ * s2; qDot4 -= beta_ * s3;
  }

  q0_ += qDot1 * invSampleFreq_;
  q1_ += qDot2 * invSampleFreq_;
  q2_ += qDot3 * invSampleFreq_;
  q3_ += qDot4 * invSampleFreq_;

  recipNorm = invSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  q0_ *= recipNorm; q1_ *= recipNorm; q2_ *= recipNorm; q3_ *= recipNorm;
  anglesComputed_ = false;
}

// Fast inverse square root, one Newton step, magic 0x5f375a86 (memcpy form, as in the original).
float Madgwick::invSqrt(float x) {
  float xhalf = 0.5f * x;
  uint32_t i;
  memcpy(&i, &x, sizeof(i));
  i = 0x5f375a86u - (i >> 1);
  memcpy(&x, &i, sizeof(i));
  x = x * (1.5f - xhalf * x * x);
  return x;
}

void Madgwick::computeAngles() {
  roll_ = atan2f(q0_ * q1_ + q2_ * q3_, 0.5f - q1_ * q1_ - q2_ * q2_);
  pitch_ = asinf(-2.0f * (q1_ * q3_ - q0_ * q2_));  // NaN if |arg|>1 (only at ~90 deg pitch)
  yaw_ = atan2f(q1_ * q2_ + q0_ * q3_, 0.5f - q2_ * q2_ - q3_ * q3_);
  anglesComputed_ = true;
}

// ---------------------------------------------------------------- KalmanFilter.cpp
void KalmanFilter::Configure(float zVariance, float zAccelVariance, float zAccelBiasVariance,
                             float zInitial, float vInitial, float aBiasInitial) {
  zAccelVariance_ = zAccelVariance;
  zAccelBiasVariance_ = zAccelBiasVariance;
  zVariance_ = zVariance;
  z_ = zInitial; v_ = vInitial; aBias_ = aBiasInitial;
  Pzz_ = 1.0f; Pzv_ = 0.0f; Pza_ = 0.0f;
  Pvz_ = 0.0f; Pvv_ = 1.0f; Pva_ = 0.0f;
  Paz_ = 0.0f; Pav_ = 0.0f; Paa_ = 100000.0f;
}

void KalmanFilter::Update(float z, float a, float dt, float *pZ, float *pV, bool fixConstrain) {
  // Predict state
  float accel = a - aBias_;
  v_ += accel * dt;
  z_ += v_ * dt;

  zAccelVariance_ = fabsf(accel) / 50.0f;
  // BUG (original): "constrain(zAccelVariance_, 1.0f, 50.0f);" discards its result, so the
  // process noise is |accel|/50 (~0 in smooth flight) and the configured 26.0638 is never used.
  if (fixConstrain) zAccelVariance_ = zAccelVariance_ < 1.0f ? 1.0f : zAccelVariance_ > 50.0f ? 50.0f : zAccelVariance_;

  // Predict state covariance
  float t00, t01, t02, t10, t11, t12, t20, t21, t22;
  float dt2div2 = dt * dt / 2.0f;
  float dt3div2 = dt2div2 * dt;
  float dt4div4 = dt2div2 * dt2div2;

  t00 = Pzz_ + dt * Pvz_ - dt2div2 * Paz_;
  t01 = Pzv_ + dt * Pvv_ - dt2div2 * Pav_;
  t02 = Pza_ + dt * Pva_ - dt2div2 * Paa_;
  t10 = Pvz_ - dt * Paz_;
  t11 = Pvv_ - dt * Pav_;
  t12 = Pva_ - dt * Paa_;
  t20 = Paz_; t21 = Pav_; t22 = Paa_;

  Pzz_ = t00 + dt * t01 - dt2div2 * t02;
  Pzv_ = t01 - dt * t02;
  Pza_ = t02;
  Pvz_ = t10 + dt * t11 - dt2div2 * t12;
  Pvv_ = t11 - dt * t12;
  Pva_ = t12;
  Paz_ = t20 + dt * t21 - dt2div2 * t22;
  Pav_ = t21 - dt * t22;
  Paa_ = t22;

  Pzz_ += dt4div4 * zAccelVariance_;
  Pzv_ += dt3div2 * zAccelVariance_;
  Pvz_ += dt3div2 * zAccelVariance_;
  Pvv_ += dt * dt * zAccelVariance_;
  Paa_ += zAccelBiasVariance_;

  // Error
  float innov = z - z_;
  float sInv = 1.0f / (Pzz_ + zVariance_);
  float kz = Pzz_ * sInv, kv = Pvz_ * sInv, ka = Paz_ * sInv;

  z_ += kz * innov;
  v_ += kv * innov;
  aBias_ += ka * innov;
  *pZ = z_;
  *pV = v_;

  Paz_ -= ka * Pzz_; Pav_ -= ka * Pzv_; Paa_ -= ka * Pza_;
  Pvz_ -= kv * Pzz_; Pvv_ -= kv * Pzv_; Pva_ -= kv * Pza_;
  Pzz_ -= kz * Pzz_; Pzv_ -= kz * Pzv_; Pza_ -= kz * Pza_;
}

// ---------------------------------------------------------------- AHRS.ino processAHRS()
float Ahrs::Sma30::add(float v) {
  sum -= ar[idx];
  ar[idx] = v;
  sum += ar[idx];
  if (++idx == 30) idx = 0;
  if (cnt < 30) cnt++;
  return sum / (float)cnt;
}

Ahrs::Ahrs(const AhrsConfig &cfg) : cfg_(cfg) {
  filter_.setBeta(cfg_.madgwickBeta);
  setInstallBias(cfg_.pitchBias, cfg_.rollBias);
}

void Ahrs::setInstallBias(float pitchBias, float rollBias) {
  cfg_.pitchBias = pitchBias;
  cfg_.rollBias = rollBias;
  float installPitchRad = pitchBias * AHRS_DEG2RAD;
  float installRollRad = rollBias * AHRS_DEG2RAD;
  cP_ = cosf(installPitchRad); sP_ = sinf(installPitchRad);
  cR_ = cosf(installRollRad);  sR_ = sinf(installRollRad);
}

void Ahrs::begin(const AircraftImu &imu, float Palt) {
  // setup(): smoothed accels seeded with the mapped (not install-corrected) accel read
  s_.aFwdSmoothed = imu.Ax;
  s_.aLatSmoothed = imu.Ay;
  s_.aVertSmoothed = imu.Az;
  s_.smoothedPitch = calcPitch(imu.Ax, imu.Ay, imu.Az) + cfg_.pitchBias;
  float roll = calcRoll(imu.Ax, imu.Ay, imu.Az);
  // BUG (original): setup() uses +calcRoll()+rollBias but the sensor calibration
  // (WifiSerial.ino) defines rollBias against -calcRoll(). Because Madgwick's roll is -roll,
  // filter.begin(..., smoothedRoll) is right only when rollBias == 0; with a nonzero
  // rollBias the filter starts 2*rollBias off and converges at the Madgwick slew rate.
  if (cfg_.fixKnownBugs) s_.smoothedRoll = -roll + cfg_.rollBias;
  else s_.smoothedRoll = roll + cfg_.rollBias;
  float madgwickRoll = cfg_.fixKnownBugs ? -s_.smoothedRoll : s_.smoothedRoll;
  filter_.begin(cfg_.imuSampleRate, -s_.smoothedPitch, madgwickRoll);
  kalman_.Configure(cfg_.kalmanZVariance, cfg_.kalmanZAccelVariance, cfg_.kalmanZAccelBiasVariance,
                    Palt * AHRS_FT2M, 0.00f, 0.00f);
  s_.kalmanAlt = Palt * AHRS_FT2M;
  lastPalt_ = Palt;
  s_.kalmanVSI = 0;
  // BUG (original): prevTAS starts at 0, so after an in-flight reboot the first TASdiff is the
  // whole TAS (~60 m/s): aFwdCp spikes to ~20 g and decays over ~0.3 s, kicking pitch ~3 deg.
  // The fix seeds prevTAS with the first TAS.
  s_.prevTAS = 0;
  s_.TASdiffSmoothed = 0;
  firstSample_ = true;
}

void Ahrs::processAHRS(const AircraftImu &imu, float IAS, float Palt, float OATc, float AOA) {
  // BUG (original): GetStaticPressure() returns -1 on an I2C timeout, so Palt =
  // 145366*(1-pow(negative,0.19)) = NaN. NaN TAS then sticks in TASdiffSmoothed (and the Kalman
  // state), making aFwdComp, the Madgwick quaternion, pitch/roll, VSI and flightPath NaN until
  // reboot. The fix holds the last finite Palt.
  if (cfg_.fixKnownBugs) {
    if (isfinite(Palt)) lastPalt_ = Palt;
    else Palt = lastPalt_;
  }
  const float Ax = imu.Ax, Ay = imu.Ay, Az = imu.Az, Gx = imu.Gx, Gy = imu.Gy, Gz = imu.Gz;
  if (cfg_.oatAvailable) {
    const float Kelvin = 273.15f, Temp_rate = 0.00198119993f;
    float ISA_temp_k = 15.0f - Temp_rate * Palt + Kelvin;
    float OAT_k = OATc + Kelvin;
    float DA = Palt + (ISA_temp_k / Temp_rate) * (1.0f - powf(ISA_temp_k / OAT_k, 0.2349690f));
    s_.TAS = AHRS_KTS2MPS * IAS / powf(1.0f - 6.8755856e-6f * DA, 2.12794f);  // m/s
  } else {
    s_.TAS = IAS * (1.0f + Palt / 1000.0f * 0.02f) * AHRS_KTS2MPS;  // m/s, +2%/1000 ft
  }
  const float TAS = s_.TAS;
  if (cfg_.fixKnownBugs && firstSample_) s_.prevTAS = TAS;
  firstSample_ = false;

  // diff TAS and smooth it; used for forward acceleration correction
  float TASdiff = TAS - s_.prevTAS;
  s_.prevTAS = TAS;
  s_.TASdiffSmoothed = cfg_.iasSmoothing * TASdiff + (1.0f - cfg_.iasSmoothing) * s_.TASdiffSmoothed;

  // installation correction, installYawRad = 0 (cos=1, sin=0)
  const float cP = cP_, sP = sP_, cR = cR_, sR = sR_, cY = 1.0f, sY = 0.0f;
  // BUG (original): "... * sin(installPitchRad) * - sin(installYawRad) ..." has '*' where '-'
  // was meant; with zero yaw the Gy*sin(roll)*sin(pitch) cross term is lost (2nd order).
  float gyTerm = cfg_.fixKnownBugs ? (cY * sR * sP - sY * cR) : (cY * sR * sP * -sY * cR);
  s_.rollRateCorr = Gx * cP * cY + Gy * gyTerm + Gz * (cY * cR * sP + sY * sR);
  s_.pitchRateCorr = Gx * cP * sY + Gy * (sY * sR * sP + cY * cR) + Gz * (sY * cR * sP - cY * sR);
  s_.yawRateCorr = Gx * -sP + Gy * sR * cP + Gz * cP * cR;

  s_.aVertCorr = -Ax * sP + Ay * sR * cP + Az * cR * cP;
  s_.aLatCorr = Ax * cP * sY + Ay * (sY * sP * sR + cY * cR) + Az * (sY * cR * sP - cY * sR);
  s_.aFwdCorr = Ax * cP * cY + Ay * (sR * sP * cY - sY * cR) + Az * (cY * cR * sP + sY * sR);

  // average gyro values, not used for AHRS (gYaw goes to the display)
  s_.gRoll = gxAvg_.add(s_.rollRateCorr);
  s_.gPitch = gyAvg_.add(s_.pitchRateCorr);
  s_.gYaw = gzAvg_.add(s_.yawRateCorr);

  // linear acceleration compensation
  s_.aFwdCp = s_.TASdiffSmoothed * cfg_.imuSampleRate * AHRS_MPS2G;  // /(1/imuSampleRate)
  // centripetal: m/s * rad/s -> m/s^2 -> g
  s_.aLatCp = TAS * s_.yawRateCorr * AHRS_DEG2RAD * AHRS_MPS2G;
  s_.aVertCp = TAS * s_.pitchRateCorr * AHRS_DEG2RAD * AHRS_MPS2G;
  if (cfg_.aoaCompensation) {  // not in the original: omega x v with v = TAS*(cos a, 0, sin a)
    // NED body rates: p = -rollRateCorr, q = -pitchRateCorr, r = yawRateCorr (see ahrs.h)
    float u = TAS * cosf(AOA * AHRS_DEG2RAD), w = TAS * sinf(AOA * AHRS_DEG2RAD);
    float k = AHRS_DEG2RAD * AHRS_MPS2G;
    s_.aFwdCp += -s_.pitchRateCorr * w * k;                          // q*w
    s_.aLatCp = (s_.yawRateCorr * u + s_.rollRateCorr * w) * k;      // r*u - p*w
    s_.aVertCp = u * s_.pitchRateCorr * k;                           // -(-q*u)
  }
  if (!cfg_.compensation) s_.aFwdCp = s_.aLatCp = s_.aVertCp = 0;

  const float a = cfg_.accSmoothing;
  s_.aFwdSmoothed = a * s_.aFwdCorr + (1.0f - a) * s_.aFwdSmoothed;
  s_.aFwdComp = s_.aFwdSmoothed - s_.aFwdCp;
  s_.aLatSmoothed = a * s_.aLatCorr + (1.0f - a) * s_.aLatSmoothed;
  s_.aLatComp = s_.aLatSmoothed - s_.aLatCp;
  s_.aVertSmoothed = a * s_.aVertCorr + (1.0f - a) * s_.aVertSmoothed;
  s_.aVertComp = s_.aVertSmoothed + s_.aVertCp;

  filter_.updateIMU(s_.rollRateCorr, s_.pitchRateCorr, s_.yawRateCorr, s_.aFwdComp, s_.aLatComp, s_.aVertComp);
  s_.smoothedPitch = -filter_.getPitch();
  s_.smoothedRoll = -filter_.getRoll();

  // VSI and flight path: earth-referenced vertical acceleration from the raw corrected accels
  float *q = s_.q;
  filter_.getQuaternion(&q[0], &q[1], &q[2], &q[3]);
  s_.earthVertG = 2.0f * (q[1] * q[3] - q[0] * q[2]) * s_.aFwdCorr + 2.0f * (q[0] * q[1] + q[2] * q[3]) * s_.aLatCorr +
                  (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * s_.aVertCorr - 1.0f;

  kalman_.Update(Palt * AHRS_FT2M, s_.earthVertG * AHRS_G2MPS, 1.0f / cfg_.imuSampleRate,
                 &s_.kalmanAlt, &s_.kalmanVSI, cfg_.fixKnownBugs);

  if (IAS < 25) s_.kalmanVSI = 0;  // zero VSI when airspeed is not yet alive (local copy only)

  if (IAS != 0) {
    float r = s_.kalmanVSI / TAS;
    // BUG (original): asin() of |VSI/TAS| > 1 is NaN (e.g. IAS just above 25 kt with a large
    // VSI); flightPath and derivedAOA then go NaN for that sample.
    if (cfg_.fixKnownBugs) r = r > 1.0f ? 1.0f : r < -1.0f ? -1.0f : r;
    s_.flightPath = asinf(r) * AHRS_RAD2DEG;
  } else {
    s_.flightPath = 0;
  }
  s_.derivedAOA = s_.smoothedPitch - s_.flightPath;
}

}  // namespace onspeed
