// Ported from OnSpeed-Gen2 v3.3.7g: AOAcalc.ino, CurveCalc.ino, Flaps.ino, SensorRead.ino,
// Tones.ino (updateTones), DisplaySerial.ino, HelperFunctions.ino.
#include "onspeed_core.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace onspeed {

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if ((in_max - in_min) + out_min == 0) return 0;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Arduino map(): integer math, float arguments are truncated by the caller's implicit conversion.
long arduino_map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float curveCalc(float x, const CalibrationCurve &curve) {
  float result = 0.00f;
  if (curve.curveType == 1) {  // polynomial
    for (int i = 0; i < MAX_CURVE_COEFF; i++) result += curve.Items[i] * powf(x, MAX_CURVE_COEFF - i - 1);
  } else if (curve.curveType == 2) {  // logarithmic
    result = curve.Items[MAX_CURVE_COEFF - 2] * logf(x) + curve.Items[MAX_CURVE_COEFF - 1];
  } else if (curve.curveType == 3) {  // exponential
    result = curve.Items[MAX_CURVE_COEFF - 2] * expf(curve.Items[MAX_CURVE_COEFF - 1] * x);
  }
  return result;
}

int parseDiffPressure(const uint8_t b[2]) {
  int reading = b[0] & 63;  // mask status bits
  reading <<= 8;
  reading |= b[1];
  return reading;
}

float parseStaticPressure(const uint8_t b[4]) {
  uint32_t result = (((uint32_t)(b[0] & 0x3f)) << 24) | ((uint32_t)b[1] << 16);
  result |= ((((uint16_t)b[2]) << 8) | b[3]) >> 5;
  int rawPressure = result >> 16;
  return (rawPressure - 1638.4f) / 8192 * 1000;  // SSCSRNN1.6BA7A3 transfer function
}

void PressureFilter::init(int medianWindow) {
  medWin_ = medianWindow > 31 ? 31 : medianWindow;
  medN_ = medPos_ = avgN_ = avgPos_ = 0;
}

float PressureFilter::add(float v) {
  med_[medPos_] = v;
  medPos_ = (medPos_ + 1) % medWin_;
  if (medN_ < medWin_) medN_++;
  float sorted[32];
  memcpy(sorted, med_, sizeof(float) * medN_);
  for (int i = 1; i < medN_; i++) {  // insertion sort, window is small
    float k = sorted[i]; int j = i - 1;
    while (j >= 0 && sorted[j] > k) { sorted[j + 1] = sorted[j]; j--; }
    sorted[j + 1] = k;
  }
  float median = (medN_ & 1) ? sorted[medN_ / 2] : (sorted[medN_ / 2] + sorted[medN_ / 2 - 1]) / 2;
  avg_[avgPos_] = median;
  avgPos_ = (avgPos_ + 1) % 10;
  if (avgN_ < 10) avgN_++;
  float sum = 0;
  for (int i = 0; i < avgN_; i++) sum += avg_[i];
  return sum / avgN_;
}

Core::Core(const Config &cfg) : cfg_(cfg) {
  fwdF_.init(cfg_.pressureSmoothing);
  p45F_.init(cfg_.pressureSmoothing);
  setAOApoints(0);
}

int Core::getFlapsIndex(int pos) const {
  if (cfg_.flapCount <= 1) return 0;
  int i = 0;
  bool reverseOrder = cfg_.flapPotPositions[0] > cfg_.flapPotPositions[cfg_.flapCount - 1];
  for (i = 1; i < cfg_.flapCount; i++) {
    int betweenZone = (cfg_.flapPotPositions[i] + cfg_.flapPotPositions[i - 1]) / 2;
    if (!reverseOrder) { if (pos <= betweenZone) return i - 1; }
    else if (pos >= betweenZone) return i - 1;
  }
  return i - 1;
}

void Core::setAOApoints(int i) {
  s_.LDmaxAOA = cfg_.LDmaxAOA[i];
  s_.onSpeedAOAfast = cfg_.onSpeedFastAOA[i];
  s_.onSpeedAOAslow = cfg_.onSpeedSlowAOA[i];
  s_.stallWarningAOA = cfg_.stallWarnAOA[i];
}

void Core::calcAOA(float Pfwd, float P45) {
  if (Pfwd > 0) {
    s_.coeffP = P45 / Pfwd;  // PCOEFF CP3, ratiometric
  } else {
    s_.AOA = AOA_MIN_VALUE;
    s_.coeffP = 0;
    return;
  }
  float smoothingAlpha = cfg_.aoaSmoothing == 0 ? 1.0f : 1.0f / cfg_.aoaSmoothing;
  s_.AOA = curveCalc(s_.coeffP, cfg_.aoaCurve[s_.flapsIndex]) * smoothingAlpha + (1 - smoothingAlpha) * s_.AOA;
  if (s_.AOA > AOA_MAX_VALUE) s_.AOA = AOA_MAX_VALUE;
  else if (s_.AOA < AOA_MIN_VALUE) s_.AOA = AOA_MIN_VALUE;
  if (isnan(s_.AOA)) s_.AOA = AOA_MIN_VALUE;
}

void Core::sensorRead(int pfwdRaw, int p45Raw, float pStaticMbar, int flapPotRaw) {
  s_.Pfwd = pfwdRaw - cfg_.pFwdBias;
  s_.P45 = p45Raw - cfg_.p45Bias;
  s_.flapPotRaw = flapPotRaw;
  s_.flapsIndex = getFlapsIndex(flapPotRaw);
  s_.flapsPos = cfg_.flapDegrees[s_.flapsIndex];
  setAOApoints(s_.flapsIndex);

  s_.PfwdSmoothed = fwdF_.add(s_.Pfwd);
  s_.P45Smoothed = p45F_.add(s_.P45);
  calcAOA(s_.PfwdSmoothed, s_.P45Smoothed);

  float PfwdPascal = ((s_.PfwdSmoothed + cfg_.pFwdBias - 0.1f * 16383) * 2 / (0.8f * 16383) - 1) * 6894.757f;
  if (PfwdPascal > 0) {
    s_.IAS = sqrtf(2 * PfwdPascal / 1.225f) * 1.94384f;  // knots
    if (cfg_.casCurveEnabled) s_.IAS = curveCalc(s_.IAS, cfg_.casCurve);
  } else {
    s_.IAS = 0;
  }
  s_.Pstatic = pStaticMbar;
  s_.Palt = 145366.45f * (1 - powf((s_.Pstatic + cfg_.pStaticBias) / 1013.25f, 0.190284f));
}

static float checkRead(SensorHealth &h, bool ok, int status, float value) {
  if (ok && status != 3) {
    if (status == 2) h.stale++;
    h.badRun = 0;
    h.fault = false;
    h.lastGood = value;
    h.haveGood = true;
    return value;
  }
  if (!ok) h.readErrors++; else h.diagnostic++;
  if (h.badRun < 0xFFFF) h.badRun++;
  if (h.badRun >= SENSOR_FAULT_READS || !h.haveGood) h.fault = true;
  return h.lastGood;
}

void Core::sensorReadBytes(const uint8_t pfwd[2], bool pfwdOk, const uint8_t p45[2], bool p45Ok,
                           const uint8_t pstatic[4], bool staticOk, int flapPotRaw) {
  float f = checkRead(s_.pfwdHealth, pfwdOk, pfwd[0] >> 6, (float)parseDiffPressure(pfwd));
  float a = checkRead(s_.p45Health, p45Ok, p45[0] >> 6, (float)parseDiffPressure(p45));
  float p = checkRead(s_.staticHealth, staticOk, pstatic[0] >> 6, parseStaticPressure(pstatic));
  s_.sensorFault = s_.pfwdHealth.fault || s_.p45Health.fault;
  if (!s_.staticHealth.haveGood) p = 1013.25f - cfg_.pStaticBias;  // no static yet: standard day, sea level
  sensorRead((int)f, (int)a, p, flapPotRaw);
}

void Core::updateTones() {
  float AOA = s_.AOA;
  if (s_.IAS <= cfg_.muteAudioUnderIAS || s_.sensorFault) {
    s_.toneMode = TONE_OFF;
    s_.pps = 20;
    return;
  }
  if (AOA >= s_.stallWarningAOA) {
    s_.highTone = true;
    s_.pps = HIGH_TONE_STALL_PPS;
    s_.toneMode = PULSE_TONE;
  } else if (AOA > s_.onSpeedAOAslow) {
    s_.highTone = true;
    s_.toneMode = PULSE_TONE;
    s_.pps = mapfloat(AOA, s_.onSpeedAOAslow, s_.stallWarningAOA, HIGH_TONE_PPS_MIN, HIGH_TONE_PPS_MAX);
  } else if (AOA >= s_.onSpeedAOAfast) {
    s_.highTone = false;
    s_.toneMode = SOLID_TONE;
    s_.pps = LOW_TONE_PPS_MAX;
  } else if (AOA >= s_.LDmaxAOA && s_.LDmaxAOA < s_.onSpeedAOAfast) {
    s_.toneMode = PULSE_TONE;
    s_.highTone = false;
    s_.pps = mapfloat(AOA, s_.LDmaxAOA, s_.onSpeedAOAfast, LOW_TONE_PPS_MIN, LOW_TONE_PPS_MAX);
  } else {
    s_.toneMode = TONE_OFF;
    s_.pps = HIGH_TONE_STALL_PPS;
  }
}

int Core::percentLift() const {
  if (s_.IAS < cfg_.muteAudioUnderIAS) return 0;
  float AOA = s_.AOA;
  long pl;
  if (AOA < s_.LDmaxAOA) pl = arduino_map(AOA, 0, s_.LDmaxAOA, 0, 50);
  else if (AOA >= s_.LDmaxAOA && AOA <= s_.onSpeedAOAfast) pl = arduino_map(AOA, s_.LDmaxAOA, s_.onSpeedAOAfast, 50, 55);
  else if (AOA > s_.onSpeedAOAfast && AOA <= s_.onSpeedAOAslow) pl = arduino_map(AOA, s_.onSpeedAOAfast, s_.onSpeedAOAslow, 55, 66);
  else if (AOA > s_.onSpeedAOAslow && AOA <= s_.stallWarningAOA) pl = arduino_map(AOA, s_.onSpeedAOAslow, s_.stallWarningAOA, 66, 90);
  else pl = arduino_map(AOA, s_.stallWarningAOA, s_.stallWarningAOA * 100 / 90, 90, 100);
  if (pl < 0) pl = 0;
  if (pl > 99) pl = 99;
  return (int)pl;
}

// Port addition: Gen2 prints values straight into fixed-width fields, so an out-of-range value
// (IAS over 999.9 kt, VSI over 9990 fpm, a NaN altitude) widens a field and shifts every later one
// while the checksum still passes. Scale, round toward zero like the int() casts, and clamp to the field.
static int fit(float v, float scale, int lo, int hi) {
  if (isnan(v)) return 0;
  float x = v * scale;
  if (x < lo) return lo;
  if (x > hi) return hi;
  return (int)x;
}

int Core::buildDisplaySentence(char *out, size_t cap, float pitch, float roll, float palt,
                               float yawRate, float lateralG, float verticalG, float vsiFpm,
                               float flightPath, int dataMark) const {
  float displayAOA = s_.IAS >= cfg_.muteAudioUnderIAS && !s_.sensorFault ? s_.AOA : 0;
  int displayVerticalG = fit(ceilf(verticalG * 10.0f), 1, -9, 99);
  int gOnsetRate = 0, spinRecoveryCue = 0, OATc = 0;
  char buf[96];
  snprintf(buf, sizeof buf,
           "#1%+04i%+05i%04u%+06i%+05i%+03i%+03i%02u%+04i%+04i%+03i%+04i%+03i%+04i%+04i%+04i%+04i%+04i%+02i%02u",
           fit(pitch, 10, -999, 999), fit(roll, 10, -9999, 9999), unsigned(fit(s_.IAS, 10, 0, 9999)),
           fit(palt, 1, -99999, 99999), fit(yawRate, 10, -9999, 9999), fit(-lateralG, 100, -99, 99),
           displayVerticalG, unsigned(s_.sensorFault ? 0 : percentLift()), fit(displayAOA, 10, -999, 999),
           int(fit(floorf(vsiFpm / 10), 1, -999, 999)), int(OATc), fit(flightPath, 10, -999, 999),
           fit(s_.flapsPos, 1, -99, 99), fit(s_.stallWarningAOA, 10, -999, 999),
           fit(s_.onSpeedAOAslow, 10, -999, 999), fit(s_.onSpeedAOAfast, 10, -999, 999),
           fit(s_.LDmaxAOA, 10, -999, 999), int(gOnsetRate * 100), int(spinRecoveryCue), unsigned(dataMark % 100));
  uint8_t crc = 0;
  for (int i = 0; i <= 75; i++) crc += (uint8_t)buf[i];
  return snprintf(out, cap, "%.76s%02X\r\n", buf, crc);
}

}  // namespace onspeed
