// Garmin G5 / G3X "=11" attitude sentence (59 bytes incl. CRLF), as parsed by
// OnSpeed-Gen2 EfisSerial.ino (efisID==4). The parser is a port of that code; the
// builder exists so the simulator can act as a G5 on the EFIS serial line.
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace onspeed {

struct EfisData {
  float IAS = 0, pitch = 0, roll = 0, lateralG = 0, verticalG = 0;
  int heading = 0, palt = 0, vsi = 0;
  char time[12] = {0};
  uint32_t goodFrames = 0, badFrames = 0;
};

int buildG5Sentence(char *out, size_t cap, int hh, int mm, int ss, int ff, float pitch, float roll,
                    int heading, float ias, int palt, float lateralG, float verticalG, int vsiFpm);

// Byte-at-a-time line assembler + parser, same framing rules as EfisSerial.ino.
class G5Parser {
 public:
  // Returns true when a complete, CRC-valid sentence was parsed into data().
  bool feed(char c);
  const EfisData &data() const { return d_; }
 private:
  bool parseLine();
  char buf_[240];
  int len_ = 0;
  char last_ = 0x0A;
  EfisData d_;
};

}  // namespace onspeed
