// Sensor bus abstraction. The real backend talks to the Gen2 sensors over ESP32-S3 I2C;
// the simulated backend produces the same raw bytes those sensors would return, from the
// flight model, so the parsing and filtering code above it is identical.
#pragma once
#include <stdint.h>
#include "flight_model.h"
#include "onspeed_core.h"

class SensorBus {
 public:
  virtual ~SensorBus() {}
  virtual bool readPfwd(uint8_t b[2]) = 0;     // TE MS4525DO +/-1 psi, I2C 0x28, bus A
  virtual bool readP45(uint8_t b[2]) = 0;      // TE MS4525DO +/-1 psi, I2C 0x28, bus B (or the 0x36 variant)
  virtual bool readStatic(uint8_t b[4]) = 0;   // SSCSRNN1.6BA7A3, I2C 0x78
  virtual int readFlapPot() = 0;               // Gen2's 13-bit / 3.3 V scale; -1 if unreadable
};

class SimSensorBus : public SensorBus {
 public:
  SimSensorBus(const onspeed::Config &cfg) : cfg_(cfg) {}
  void setFlight(const FlightState &s) { fs_ = s; }
  bool readPfwd(uint8_t b[2]) override;
  bool readP45(uint8_t b[2]) override;
  bool readStatic(uint8_t b[4]) override;
  int readFlapPot() override;
 private:
  int noise();
  int fwdCounts() const;
  onspeed::Config cfg_;
  FlightState fs_{};
  uint32_t lcg_ = 12345, n_ = 0;
};

// ESP32-S3 I2C backend for the real hardware build (compiled, not exercised in QEMU).
class I2cSensorBus : public SensorBus {
 public:
  bool begin(int sdaA, int sclA, int sdaB, int sclB, int flapAdcChannel);
  bool readPfwd(uint8_t b[2]) override;
  bool readP45(uint8_t b[2]) override;
  bool readStatic(uint8_t b[4]) override;
  int readFlapPot() override;
  uint32_t busResets() const { return resets_; }
 private:
  bool rx(void *dev, void *bus, int &fails, uint8_t *b, int n);
  void *busA_ = nullptr, *busB_ = nullptr;
  void *pfwd_ = nullptr, *p45_ = nullptr, *pstatic_ = nullptr;
  void *adc_ = nullptr, *cali_ = nullptr;
  int flapCh_ = 0, failA_ = 0, failB_ = 0;
  uint32_t resets_ = 0;
};
