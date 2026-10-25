#include "sensors.h"
#include <math.h>
#include "driver/i2c_master.h"
#if !defined(ONSPEED_SIM) || ONSPEED_SIM
#define SENSORS_SIM_BUILD 1
#else
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#endif

// ---------------- simulated sensors ----------------

int SimSensorBus::noise() {
  lcg_ = lcg_ * 1664525u + 1013904223u;
  // +/-2 counts uniform = 1.41 counts RMS, close to the 1.5 counts RMS estimated for the MS4525DO
  int v = (int)((lcg_ >> 16) % 5) - 2;
  return v;
}

// Differential pressure in 14-bit counts above the sensor zero: TE MS4525DO 4525DO-DS5AI001DP,
// +/-1 psi, type A (10-90%) transfer function, same status-bit layout as the Honeywell parts.
int SimSensorBus::fwdCounts() const {
  float iasInd = fs_.ias > 5 ? (fs_.ias - cfg_.casCurve.Items[3]) / cfg_.casCurve.Items[2] : fs_.ias;
  float v = iasInd / 1.94384f;
  float q = 0.5f * 1.225f * v * v;  // Pa
  return (int)lroundf(q / 6894.757f * (0.8f * 16383 / 2));
}

static void pack14(int counts, uint8_t b[2]) {
  if (counts < 0) counts = 0;
  if (counts > 16383) counts = 16383;
  b[0] = (counts >> 8) & 0x3f;  // status bits 00 = normal
  b[1] = counts & 0xff;
}

// Scripted faults, so the run exercises the core's fault handling:
//   58.0-58.2 s  Pfwd I2C reads fail (NACK): 10 reads, the tone must ride through
//   61.0-61.8 s  P45 reports diagnostic status (11): fault after 0.5 s, tones mute, then recover
//   every 3rd Pfwd read carries the stale status (10), which is harmless
static bool inWindow(float t, float a, float b) { return t >= a - 1e-4f && t < b - 1e-4f; }

bool SimSensorBus::readPfwd(uint8_t b[2]) {
  n_++;
  int c = fwdCounts();
  int raw = cfg_.pFwdBias + c + noise();
  if (n_ % 97 == 0) raw += 400;  // occasional spike, the median filter should reject it
  pack14(raw, b);
  if (n_ % 3 == 0) b[0] |= 0x80;  // stale status
  if (inWindow(fs_.t, 58.0f, 58.2f)) { b[0] = b[1] = 0xFF; return false; }
  return true;
}

bool SimSensorBus::readP45(uint8_t b[2]) {
  // Invert the active calibration curve: AOA = A*CP^2 + B*CP + C  ->  CP for the true AOA.
  int fi = fs_.flaps >= 40 ? 2 : fs_.flaps >= 20 ? 1 : 0;
  const auto &c = cfg_.aoaCurve[fi].Items;
  float A = c[1], B = c[2], C = c[3];
  float disc = B * B - 4 * A * (C - fs_.aoa);
  float cp = disc > 0 ? (-B + sqrtf(disc)) / (2 * A) : -B / (2 * A);
  int raw = cfg_.p45Bias + (int)lroundf(cp * fwdCounts()) + noise();
  pack14(raw, b);
  if (inWindow(fs_.t, 61.0f, 61.8f)) { b[0] = 0xFF; b[1] = 0xFF; }  // diagnostic status, garbage counts
  return true;
}

bool SimSensorBus::readStatic(uint8_t b[4]) {
  float mbar = 1013.25f * powf(1 - fs_.palt / 145366.45f, 1 / 0.190284f) - cfg_.pStaticBias;
  int raw = (int)lroundf(mbar / 1000 * 8192 + 1638.4f) + noise() / 2;  // ~1 count RMS (estimate, datasheet gives none)
  b[0] = (raw >> 8) & 0x3f;
  b[1] = raw & 0xff;
  int temp = 1024;  // ~25 C
  b[2] = (temp << 5) >> 8;
  b[3] = (temp << 5) & 0xff;
  return true;
}

int SimSensorBus::readFlapPot() {
  return fs_.flaps >= 40 ? cfg_.flapPotPositions[2] : fs_.flaps >= 20 ? cfg_.flapPotPositions[1] : cfg_.flapPotPositions[0];
}

// ---------------- real ESP32-S3 I2C sensors ----------------
// Each read reports the I2C result so the core can hold the last good value and flag a fault
// (Core::sensorReadBytes). After 10 consecutive failures on a bus the driver resets it: a sensor
// that browned out mid-byte can hold SDA low, and the reset clocks it free.

static constexpr int kI2cTimeoutMs = 2;
static constexpr int kResetAfterFails = 10;

bool I2cSensorBus::begin(int sdaA, int sclA, int sdaB, int sclB, int flapAdcChannel) {
  i2c_master_bus_config_t bc = {};
  bc.clk_source = I2C_CLK_SRC_DEFAULT;
  bc.glitch_ignore_cnt = 7;
  bc.flags.enable_internal_pullup = true;  // ~45k, too weak alone: the board needs 4.7k externals
  bc.i2c_port = I2C_NUM_0; bc.sda_io_num = (gpio_num_t)sdaA; bc.scl_io_num = (gpio_num_t)sclA;
  i2c_master_bus_handle_t a, bb;
  if (i2c_new_master_bus(&bc, &a) != ESP_OK) return false;
  bc.i2c_port = I2C_NUM_1; bc.sda_io_num = (gpio_num_t)sdaB; bc.scl_io_num = (gpio_num_t)sclB;
  if (i2c_new_master_bus(&bc, &bb) != ESP_OK) return false;
  busA_ = a; busB_ = bb;
  i2c_device_config_t dc = {};
  dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dc.scl_speed_hz = 400000;
  i2c_master_dev_handle_t h;
  dc.device_address = 0x28; i2c_master_bus_add_device(a, &dc, &h); pfwd_ = h;    // MS4525DO, bus A
  dc.device_address = 0x78; i2c_master_bus_add_device(a, &dc, &h); pstatic_ = h; // SSCSRNN1.6BA7A3, bus A
  dc.device_address = 0x28; i2c_master_bus_add_device(bb, &dc, &h); p45_ = h;    // MS4525DO, bus B
#ifndef SENSORS_SIM_BUILD
  adc_oneshot_unit_init_cfg_t uc = {};
  uc.unit_id = ADC_UNIT_1;
  adc_oneshot_unit_handle_t ah;
  if (adc_oneshot_new_unit(&uc, &ah) != ESP_OK) return false;
  adc_oneshot_chan_cfg_t cc = {};
  cc.atten = ADC_ATTEN_DB_12; cc.bitwidth = ADC_BITWIDTH_12;
  adc_oneshot_config_channel(ah, (adc_channel_t)flapAdcChannel, &cc);
  adc_ = ah; flapCh_ = flapAdcChannel;
  // factory curve-fitting calibration: raw -> millivolts, corrects the S3's gain and offset spread
  adc_cali_curve_fitting_config_t cf = {};
  cf.unit_id = ADC_UNIT_1; cf.chan = (adc_channel_t)flapAdcChannel; cf.atten = ADC_ATTEN_DB_12; cf.bitwidth = ADC_BITWIDTH_12;
  adc_cali_handle_t cal;
  if (adc_cali_create_scheme_curve_fitting(&cf, &cal) == ESP_OK) cali_ = cal;
#endif
  return true;
}

bool I2cSensorBus::rx(void *dev, void *bus, int &fails, uint8_t *b, int n) {
  if (i2c_master_receive((i2c_master_dev_handle_t)dev, b, n, kI2cTimeoutMs) == ESP_OK) { fails = 0; return true; }
  if (++fails % kResetAfterFails == 0) { i2c_master_bus_reset((i2c_master_bus_handle_t)bus); resets_++; }
  return false;
}
bool I2cSensorBus::readPfwd(uint8_t b[2]) { return rx(pfwd_, busA_, failA_, b, 2); }
bool I2cSensorBus::readP45(uint8_t b[2]) { return rx(p45_, busB_, failB_, b, 2); }
bool I2cSensorBus::readStatic(uint8_t b[4]) { return rx(pstatic_, busA_, failA_, b, 4); }

// Gen2 read the flap pot with analogReadResolution(13) on a 3.3 V reference, and its stored flap
// positions are on that scale. Return the same scale from calibrated millivolts so existing
// configurations carry over, averaging 32 samples against the S3 ADC's noise.
int I2cSensorBus::readFlapPot() {
  int sum = 0, n = 0;
#ifndef SENSORS_SIM_BUILD
  for (int i = 0; i < 32; i++) {
    int raw = 0;
    if (adc_oneshot_read((adc_oneshot_unit_handle_t)adc_, (adc_channel_t)flapCh_, &raw) != ESP_OK) continue;
    int mv = raw * 3300 / 4095;
    if (cali_) adc_cali_raw_to_voltage((adc_cali_handle_t)cali_, raw, &mv);
    sum += mv; n++;
  }
#endif
  if (!n) return -1;
  return (sum / n) * 8191 / 3300;
}
