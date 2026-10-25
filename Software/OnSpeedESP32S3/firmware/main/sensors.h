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
  virtual bool readPfwd(uint8_t b[2]) = 0;     // Honeywell diff sensor, I2C 0x28
  virtual bool readP45(uint8_t b[2]) = 0;      // Honeywell diff sensor, I2C 0x28 on 2nd bus (or 0x38 variant)
  virtual bool readStatic(uint8_t b[4]) = 0;   // SSCSRNN1.6BA7A3, I2C 0x78
  virtual int readFlapPot() = 0;               // 10-bit scale, like Gen2 analogRead
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
 private:
  void *busA_ = nullptr, *busB_ = nullptr;
  void *pfwd_ = nullptr, *p45_ = nullptr, *pstatic_ = nullptr;
  void *adc_ = nullptr;
  int flapCh_ = 0;
};
