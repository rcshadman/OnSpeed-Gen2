// OnSpeed core logic, ported from OnSpeed-Gen2 (Software/OnSpeedTeensy_AHRS) v3.3.7g
// to plain C++ with no Arduino or Teensy dependencies, so it runs on ESP32-S3 (ESP-IDF)
// and on a desktop for testing. Function names follow the original sketch.
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace onspeed {

constexpr int MAX_AOA_CURVES = 5;
constexpr int MAX_CURVE_COEFF = 4;

constexpr float AOA_MAX_VALUE = 40;
constexpr float AOA_MIN_VALUE = -20;

// Tones.ino / OnSpeedTeensy_AHRS.ino
constexpr float HIGH_TONE_STALL_PPS = 20;
constexpr float HIGH_TONE_PPS_MAX = 6.2f;
constexpr float HIGH_TONE_PPS_MIN = 1.5f;
constexpr int   HIGH_TONE_HZ = 1600;
constexpr float LOW_TONE_PPS_MAX = 8.2f;
constexpr float LOW_TONE_PPS_MIN = 1.5f;
constexpr int   LOW_TONE_HZ = 400;
constexpr float TONE_RAMP_TIME = 15;   // ms
constexpr float STALL_RAMP_TIME = 5;   // ms
constexpr float LOW_TONE_VOLUME = .25f;
constexpr float HIGH_TONE_VOLUME_MIN = .25f;
constexpr float SOLID_TONE_VOLUME = .25f;
constexpr float HIGH_TONE_VOLUME_MAX = 1;

enum ToneMode : uint8_t { PULSE_TONE = 1, SOLID_TONE = 2, TONE_OFF = 3 };

struct CalibrationCurve {
  float Items[MAX_CURVE_COEFF];
  uint8_t curveType;  // 1 polynomial, 2 logarithmic, 3 exponential
};

// The subset of onspeed.cfg the core logic needs (default_config.h, RV-4).
struct Config {
  int aoaSmoothing = 10;
  int pressureSmoothing = 15;
  int flapCount = 3;
  int flapDegrees[MAX_AOA_CURVES] = {0, 20, 40};
  int flapPotPositions[MAX_AOA_CURVES] = {129, 158, 206};
  float LDmaxAOA[MAX_AOA_CURVES] = {8.03f, 5.73f, 4.78f};
  float onSpeedFastAOA[MAX_AOA_CURVES] = {11.25f, 9.74f, 8.62f};
  float onSpeedSlowAOA[MAX_AOA_CURVES] = {13.84f, 12.64f, 12.44f};
  float stallWarnAOA[MAX_AOA_CURVES] = {16.48f, 16.29f, 14.51f};
  CalibrationCurve aoaCurve[MAX_AOA_CURVES] = {
      {{0.0f, 8.4845f, 24.0804f, 4.6157f}, 1},
      {{0.0f, 4.5244f, 27.9928f, 2.7661f}, 1},
      {{0.0f, 1.2984f, 30.4239f, 1.4550f}, 1}};
  CalibrationCurve casCurve = {{0.0f, 0.0f, 0.9770f, 4.1620f}, 1};
  bool casCurveEnabled = true;
  int pFwdBias = 8108;
  int p45Bias = 8058;
  float pStaticBias = -2.1997f;
  int muteAudioUnderIAS = 25;
};

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
long arduino_map(long x, long in_min, long in_max, long out_min, long out_max);
float curveCalc(float x, const CalibrationCurve &curve);

// Honeywell sensor transfer functions (SensorRead.ino). These take the raw I2C bytes,
// so the same code path runs whether the bytes come from a real sensor or the simulator.
int parseDiffPressure(const uint8_t b[2]);       // 14-bit counts, status bits masked
float parseStaticPressure(const uint8_t b[4]);   // millibars, SSCSRNN1.6BA7A3

// RunningMedian + RunningAverage(10) as used for pressure despiking/smoothing.
class PressureFilter {
 public:
  void init(int medianWindow);
  float add(float v);
 private:
  float med_[32]; int medN_ = 0, medPos_ = 0, medWin_ = 15;
  float avg_[10]; int avgN_ = 0, avgPos_ = 0;
};

// Port addition: flap position from the analog flap sensor (pot, Hall sensor or detent-switch
// resistor ladder, all on Gen2's 13-bit / 3.3 V scale). Gen2 picks the nearest stage with no
// hysteresis or fault check; this adds a dead band around each boundary, a debounce, an
// out-of-range fault (open or shorted wire) that holds the last good stage, and learn mode.
class FlapDetector {
 public:
  void configure(const int *positions, int count);
  int update(int reading);           // returns the flap stage index
  bool fault() const { return fault_; }
  bool moving() const { return pending_ != idx_; }
  int lastReading() const { return last_; }
  static constexpr int kDebounce = 10;        // cycles (0.2 s at 50 Hz)
  static constexpr int kFaultCycles = 10;
 private:
  int nearest(int r) const;
  int pos_[5] = {0}, n_ = 1, idx_ = 0, pending_ = 0, stable_ = 0, faultRun_ = 0, last_ = 0;
  int lo_ = 0, hi_ = 8191, hyst_ = 0;
  bool fault_ = false, good_ = false;
};

// Port addition (not in Gen2): per-sensor read health. Gen2 masks the Honeywell status bits and
// feeds a failed read (-1) straight into the filters; here a failed or diagnostic read reuses the
// last good value and a run of them raises a fault that mutes the tones instead of sounding wrong ones.
struct SensorHealth {
  uint32_t readErrors = 0;   // I2C transaction failed (NACK, timeout, bus error)
  uint32_t diagnostic = 0;   // status bits 11: sensor reports a fault
  uint32_t stale = 0;        // status bits 10: read faster than the sensor updates (harmless)
  uint16_t badRun = 0;       // consecutive bad reads
  bool fault = false;
  float lastGood = 0;
  bool haveGood = false;
};
constexpr int SENSOR_FAULT_READS = 25;  // 0.5 s of bad reads at 50 Hz

struct State {
  // inputs
  int flapPotRaw = 0;
  // derived
  int flapsIndex = 0, flapsPos = 0;
  float Pfwd = 0, P45 = 0, PfwdSmoothed = 0, P45Smoothed = 0;
  float coeffP = 0, AOA = 0, IAS = 0;
  float Pstatic = 0, Palt = 0;
  float LDmaxAOA = 0, onSpeedAOAfast = 0, onSpeedAOAslow = 0, stallWarningAOA = 0;
  // tone selection (updateTones)
  ToneMode toneMode = TONE_OFF;
  bool highTone = false;
  float pps = 20;
  SensorHealth pfwdHealth, p45Health, staticHealth;
  bool sensorFault = false;
  bool flapFault = false;    // flap sensor reading out of range: last good stage held  // pitot-side pressure sensor failed: tones muted, display shows 0 AOA
};

class Core {
 public:
  explicit Core(const Config &cfg);
  // One 50 Hz sensor cycle (SensorRead.ino): raw sensor counts in, AOA/IAS out.
  void sensorRead(int pfwdRaw, int p45Raw, float pStaticMbar, int flapPotRaw);
  // Same cycle from the raw sensor bytes plus each read's I2C result, with fault handling.
  void sensorReadBytes(const uint8_t pfwd[2], bool pfwdOk, const uint8_t p45[2], bool p45Ok,
                       const uint8_t pstatic[4], bool staticOk, int flapPotRaw);
  void updateTones();  // Tones.ino updateTones()
  int percentLift() const;  // DisplaySerial.ino
  // Build the 80-byte "#1" ONSPEED display sentence (DisplaySerial.ino) incl. checksum + CRLF.
  // Returns the length written (80).
  int buildDisplaySentence(char *out, size_t cap, float pitch, float roll, float palt,
                           float yawRate, float lateralG, float verticalG,
                           float vsiFpm, float flightPath, int dataMark) const;
  const State &state() const { return s_; }
  const Config &config() const { return cfg_; }
  // Learn mode (ground only): store the current flap reading as stage i's position.
  bool learnFlapPosition(int stage, int degrees);
 private:
  int getFlapsIndex(int pot) const;
  void setAOApoints(int i);
  void calcAOA(float Pfwd, float P45);
  Config cfg_;
  State s_;
  PressureFilter fwdF_, p45F_;
  FlapDetector flaps_;
};

}  // namespace onspeed
