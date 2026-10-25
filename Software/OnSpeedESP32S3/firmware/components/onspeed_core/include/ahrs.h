// OnSpeed Gen2 AHRS, ported from OnSpeed-Gen2 (Software/OnSpeedTeensy_AHRS) v3.3.7g:
// AHRS.ino (processAHRS, calcPitch, calcRoll), IMUread.ino (configureAxes, getAccelForAxis,
// getGyroForAxis, scaleAccel, scaleGyro), MadgwickFusion.cpp/.h, KalmanFilter.cpp/.h and the
// AHRS part of setup() in OnSpeedTeensy_AHRS.ino. Single-precision float only (ESP32-S3 FPU).
//
// Frame conventions of the original, as derived from the code (not stated there):
//   Ax = forward specific force (g, + when accelerating forward or nose up at rest)
//   Ay = lateral specific force (g, + to the right; right wing low at rest gives Ay < 0)
//   Az = "vertical G" (g, +1 level at rest; i.e. minus the NED z specific force)
//   Gx = -p, Gy = -q, Gz = +r (deg/s, p/q/r = NED body rates: roll right, nose up, yaw right)
// getGyroForAxis() negates every gyro axis; the result is a right-handed (aft,left,down) body
// frame for the gyros and the gravity vector in that same frame for the accels, which is what
// Madgwick needs. smoothedPitch/smoothedRoll come out as normal (nose up +, right wing down +).
#pragma once
#include <stdint.h>

namespace onspeed {

// OnSpeedTeensy_AHRS.ino unit constants (as float)
constexpr float AHRS_DEG2RAD = 0.0174533f;
constexpr float AHRS_RAD2DEG = 57.2958f;
constexpr float AHRS_G2MPS = 9.80665f;
constexpr float AHRS_MPS2G = 0.101971621f;
constexpr float AHRS_FT2M = 0.3048f;
constexpr float AHRS_M2FT = 3.28084f;
constexpr float AHRS_MPS2FPM = 196.85f;
constexpr float AHRS_KTS2MPS = 0.514444f;

enum class ImuType : uint8_t { ISM330DHCX, LSM9DS1 };

// IMU full-scale settings (OnSpeedTeensy_AHRS.ino aRes/gRes, IMUread.ino initAccelGyro)
constexpr float ISM330_ARES = 8.0f / 32768.0f;    // g / LSB, +/-8 g
constexpr float ISM330_GRES = 250.0f / 32768.0f;  // dps / LSB, 250 dps
constexpr float LSM9DS1_ARES = 8.0f / 32768.0f;
constexpr float LSM9DS1_GRES = 245.0f / 32768.0f;

// One axis of the box-to-aircraft map: which chip axis (0=X,1=Y,2=Z) and its sign.
struct AxisSpec { uint8_t axis = 2; int8_t sign = 1; };
struct AxisMap {
  AxisSpec vertical, lateral, forward;  // verticalGloadAxis, lateralGloadAxis, forwardGloadAxis
  bool valid = false;                   // false: orientation pair not in the table
};

// IMUread.ino configureAxes(). portsOrientation/boxtopOrientation are "FORWARD", "AFT",
// "LEFT", "RIGHT", "UP", "DOWN". Gyro axes are the same as the accel axes (yaw=vertical,
// pitch=lateral, roll=forward), as in the original.
AxisMap configureAxes(const char *portsOrientation, const char *boxtopOrientation,
                      ImuType type = ImuType::ISM330DHCX);

// Chip-frame sample after scaleAccel()/scaleGyro(): g and deg/s.
struct ImuSample { float ax = 0, ay = 0, az = 1, gx = 0, gy = 0, gz = 0; };
ImuSample scaleImu(const int16_t acc[3], const int16_t gyr[3], ImuType type = ImuType::ISM330DHCX);

// Aircraft-frame values (globals Ax..Gz in ReadIMU()).
struct AircraftImu { float Ax = 0, Ay = 0, Az = 1, Gx = 0, Gy = 0, Gz = 0; };
struct GyroBias { float gxBias = 0, gyBias = 0, gzBias = 0; };  // chip axes, added to raw
// getAccelForAxis()/getGyroForAxis() applied to all six axes.
AircraftImu mapAxes(const AxisMap &map, const ImuSample &s, const GyroBias &bias = GyroBias());

// AHRS.ino calcPitch()/calcRoll(): accelerometer-only attitude, degrees.
float calcPitch(float aFwd, float aLat, float aVert);
float calcRoll(float aFwd, float aLat, float aVert);

// MadgwickFusion.cpp: Madgwick IMU (6-DOF) filter with OnSpeed's beta and fast inverse sqrt.
class Madgwick {
 public:
  Madgwick();
  void begin(float sampleFrequency, float pitch, float roll);  // degrees
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);  // dps, any accel unit
  float getRoll() { if (!anglesComputed_) computeAngles(); return roll_ * 57.29578f; }
  float getPitch() { if (!anglesComputed_) computeAngles(); return pitch_ * 57.29578f; }
  float getYaw() { if (!anglesComputed_) computeAngles(); return yaw_ * 57.29578f + 180.0f; }
  void getQuaternion(float *w, float *x, float *y, float *z) const { *w = q0_; *x = q1_; *y = q2_; *z = q3_; }
  void setBeta(float b) { beta_ = b; }
  static float invSqrt(float x);
 private:
  void computeAngles();
  float beta_, q0_, q1_, q2_, q3_, invSampleFreq_;
  float roll_ = 0, pitch_ = 0, yaw_ = 0;
  bool anglesComputed_ = false;
};

// KalmanFilter.cpp (har-in-air Kalmanfilter_altimeter_vario): z, v, accel-bias states.
class KalmanFilter {
 public:
  void Configure(float zVariance, float zAccelVariance, float zAccelBiasVariance,
                 float zInitial, float vInitial, float aBiasInitial);
  // fixConstrain=false reproduces the original, whose constrain() result is discarded.
  void Update(float z, float a, float dt, float *pZ, float *pV, bool fixConstrain = false);
  float aBias() const { return aBias_; }
 private:
  float z_ = 0, v_ = 0, aBias_ = 0;
  float Pzz_ = 1, Pzv_ = 0, Pza_ = 0, Pvz_ = 0, Pvv_ = 1, Pva_ = 0, Paz_ = 0, Pav_ = 0, Paa_ = 100000;
  float zAccelBiasVariance_ = 1e-11f, zAccelVariance_ = 26.0638f, zVariance_ = 0.79078f;
};

struct AhrsConfig {
  float imuSampleRate = 208;           // ISM330DHCX (238 for LSM9DS1)
  float pitchBias = 0, rollBias = 0;   // installation bias, degrees (sensor calibration)
  float accSmoothing = 0.060899f;      // accelerometer EMA alpha
  float iasSmoothing = 0.0179f;        // TAS-derivative EMA alpha
  float madgwickBeta = 0.011617f;      // MadgwickFusion.cpp betaDef
  float kalmanZVariance = 0.79078f;    // kalman.Configure() in setup()
  float kalmanZAccelVariance = 26.0638f;
  float kalmanZAccelBiasVariance = 1e-11f;
  bool oatAvailable = false;           // #define OAT_AVAILABLE (density-altitude TAS)
  // Not in the original; for testing and for opting into fixes:
  bool compensation = true;            // false: feed Madgwick uncompensated smoothed accel
  bool fixKnownBugs = false;           // see ahrs.cpp "BUG" comments
  // Experimental, not in the original: include the AOA in the centripetal compensation
  // (body velocity = TAS*(cos AOA, 0, sin AOA) instead of TAS along the body x axis).
  bool aoaCompensation = false;
};

struct AhrsState {
  float TAS = 0, prevTAS = 0, TASdiffSmoothed = 0;               // m/s
  float rollRateCorr = 0, pitchRateCorr = 0, yawRateCorr = 0;    // install-corrected, dps
  float aFwdCorr = 0, aLatCorr = 0, aVertCorr = 1;               // install-corrected, g
  float gRoll = 0, gPitch = 0, gYaw = 0;                         // 30-sample SMA of rates
  float aFwdCp = 0, aLatCp = 0, aVertCp = 0;                     // linear/centripetal comp, g
  float aFwdSmoothed = 0, aLatSmoothed = 0, aVertSmoothed = 1;
  float aFwdComp = 0, aLatComp = 0, aVertComp = 1;               // fed to Madgwick
  float smoothedPitch = 0, smoothedRoll = 0;                     // deg
  float q[4] = {1, 0, 0, 0};
  float earthVertG = 0;                                          // g, earth-up, gravity removed
  float kalmanAlt = 0, kalmanVSI = 0;                            // m, m/s
  float flightPath = 0, derivedAOA = 0;                          // deg
};

class Ahrs {
 public:
  explicit Ahrs(const AhrsConfig &cfg = AhrsConfig());
  // setup(): seed smoothed accels from one IMU read, attitude from calcPitch/calcRoll, and
  // the Kalman altitude from Palt (feet). prevTAS starts at 0, as in the original.
  void begin(const AircraftImu &imu, float Palt);
  // processAHRS(), called from ReadIMU() at imuSampleRate. IAS in knots (50 Hz global in the
  // original, held between updates), Palt in feet, OATc in deg C (only if oatAvailable),
  // AOA in degrees (only used if aoaCompensation).
  void processAHRS(const AircraftImu &imu, float IAS, float Palt, float OATc = 15, float AOA = 0);
  // pitchBias/rollBias change (sensor calibration in WifiSerial.ino); recomputes the cached
  // installation rotation.
  void setInstallBias(float pitchBias, float rollBias);
  const AhrsState &state() const { return s_; }
  const AhrsConfig &config() const { return cfg_; }
  Madgwick &filter() { return filter_; }
  KalmanFilter &kalman() { return kalman_; }
 private:
  struct Sma30 {  // RunningAverage(gyroSmoothing).getFastAverage()
    float ar[30] = {0}; float sum = 0; int idx = 0, cnt = 0;
    float add(float v);
  };
  AhrsConfig cfg_;
  AhrsState s_;
  Madgwick filter_;
  KalmanFilter kalman_;
  Sma30 gxAvg_, gyAvg_, gzAvg_;
  // install rotation, cached (the original recomputes 20 sin/cos per sample)
  float cP_ = 1, sP_ = 0, cR_ = 1, sR_ = 0;
  float lastPalt_ = 0;  // last finite Palt (fixKnownBugs)
  bool firstSample_ = true;
};

}  // namespace onspeed
