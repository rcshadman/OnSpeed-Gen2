#include "sensors.h"
#include <math.h>
#include "driver/i2c_master.h"
#if !defined(ONSPEED_SIM) || ONSPEED_SIM
#define SENSORS_SIM_BUILD 1
#else
#include "esp_adc/adc_oneshot.h"
#endif

// ---------------- simulated sensors ----------------

int SimSensorBus::noise() {
  lcg_ = lcg_ * 1664525u + 1013904223u;
  int v = (int)((lcg_ >> 16) % 5) - 2;  // +/-2 counts of sensor noise
  return v;
}

// Differential pressure in 14-bit counts above the sensor zero (+/-1 psi, 10-90% transfer).
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

bool SimSensorBus::readPfwd(uint8_t b[2]) {
  n_++;
  int c = fwdCounts();
  int raw = cfg_.pFwdBias + c + noise();
  if (n_ % 97 == 0) raw += 400;  // occasional spike, the median filter should reject it
  pack14(raw, b);
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
  return true;
}

bool SimSensorBus::readStatic(uint8_t b[4]) {
  float mbar = 1013.25f * powf(1 - fs_.palt / 145366.45f, 1 / 0.190284f) - cfg_.pStaticBias;
  int raw = (int)lroundf(mbar / 1000 * 8192 + 1638.4f);
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

bool I2cSensorBus::begin(int sdaA, int sclA, int sdaB, int sclB, int flapAdcChannel) {
  i2c_master_bus_config_t bc = {};
  bc.clk_source = I2C_CLK_SRC_DEFAULT;
  bc.glitch_ignore_cnt = 7;
  bc.flags.enable_internal_pullup = true;
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
  dc.device_address = 0x28; i2c_master_bus_add_device(a, &dc, &h); pfwd_ = h;
  dc.device_address = 0x28; i2c_master_bus_add_device(bb, &dc, &h); p45_ = h;
  dc.device_address = 0x78; i2c_master_bus_add_device(a, &dc, &h); pstatic_ = h;
#ifndef SENSORS_SIM_BUILD
  adc_oneshot_unit_init_cfg_t uc = {};
  uc.unit_id = ADC_UNIT_1;
  adc_oneshot_unit_handle_t ah;
  if (adc_oneshot_new_unit(&uc, &ah) != ESP_OK) return false;
  adc_oneshot_chan_cfg_t cc = {};
  cc.atten = ADC_ATTEN_DB_12; cc.bitwidth = ADC_BITWIDTH_12;
  adc_oneshot_config_channel(ah, (adc_channel_t)flapAdcChannel, &cc);
  adc_ = ah; flapCh_ = flapAdcChannel;
#endif
  return true;
}

bool I2cSensorBus::readPfwd(uint8_t b[2]) {
  return i2c_master_receive((i2c_master_dev_handle_t)pfwd_, b, 2, 2) == ESP_OK;
}
bool I2cSensorBus::readP45(uint8_t b[2]) {
  return i2c_master_receive((i2c_master_dev_handle_t)p45_, b, 2, 2) == ESP_OK;
}
bool I2cSensorBus::readStatic(uint8_t b[4]) {
  return i2c_master_receive((i2c_master_dev_handle_t)pstatic_, b, 4, 2) == ESP_OK;
}
int I2cSensorBus::readFlapPot() {
  int raw = 0;
#ifndef SENSORS_SIM_BUILD
  adc_oneshot_read((adc_oneshot_unit_handle_t)adc_, (adc_channel_t)flapCh_, &raw);
#endif
  return raw >> 2;  // 12-bit -> Gen2's 10-bit scale
}
