#include "g5.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace onspeed {

int buildG5Sentence(char *out, size_t cap, int hh, int mm, int ss, int ff, float pitch, float roll,
                    int heading, float ias, int palt, float lateralG, float verticalG, int vsiFpm) {
  char b[64];
  // =11 hhmmssff pitch(4) roll(5) hdg(3) ias(4) palt(6) turn(4) latG(3) vertG(3) pctLift(2) vsi(4) oat..(6)
  snprintf(b, sizeof b, "=11%02d%02d%02d%02d%+04d%+05d%03d%04d%+06d____%+03d%+03d__%+04d______",
           hh, mm, ss, ff, (int)(pitch * 10), (int)(roll * 10), heading, (int)(ias * 10), palt,
           (int)(lateralG * 100), (int)(verticalG * 10), vsiFpm / 10);
  uint8_t crc = 0;
  for (int i = 0; i <= 54; i++) crc += (uint8_t)b[i];
  return snprintf(out, cap, "%.55s%02X\r\n", b, crc);
}

static float subf(const char *s, int a, int b, bool *ok) {
  char t[16]; int n = b - a;
  memcpy(t, s + a, n); t[n] = 0;
  *ok = strspn(t, "_") != (size_t)n;
  return (float)atof(t);
}

bool G5Parser::feed(char c) {
  if (len_ > 230) len_ = 0;  // prevent buffer overflow
  bool parsed = false;
  if (len_ > 0 || last_ == 0x0A) {  // sync on LF, like EfisSerial.ino
    buf_[len_++] = c;
    if (c == 0x0A) {
      parsed = parseLine();
      len_ = 0;
    }
  }
  last_ = c;
  return parsed;
}

bool G5Parser::parseLine() {
  if (!(len_ == 59 && buf_[0] == '=' && buf_[1] == '1' && buf_[2] == '1')) return false;
  int calcCRC = 0;
  for (int i = 0; i <= 54; i++) calcCRC += (uint8_t)buf_[i];
  calcCRC &= 0xFF;
  char hex[3] = {buf_[55], buf_[56], 0};
  if (calcCRC != (int)strtol(hex, NULL, 16)) { d_.badFrames++; return false; }
  bool ok; float v;
  v = subf(buf_, 23, 27, &ok); if (ok) d_.IAS = v / 10;
  v = subf(buf_, 11, 15, &ok); if (ok) d_.pitch = v / 10;
  v = subf(buf_, 15, 20, &ok); if (ok) d_.roll = v / 10;
  v = subf(buf_, 20, 23, &ok); if (ok) d_.heading = (int)v;
  v = subf(buf_, 37, 40, &ok); if (ok) d_.lateralG = v / 100;
  v = subf(buf_, 40, 43, &ok); if (ok) d_.verticalG = v / 10;
  v = subf(buf_, 27, 33, &ok); if (ok) d_.palt = (int)v;
  v = subf(buf_, 45, 49, &ok); if (ok) d_.vsi = (int)v * 10;
  snprintf(d_.time, sizeof d_.time, "%.2s:%.2s:%.2s.%.2s", buf_ + 3, buf_ + 5, buf_ + 7, buf_ + 9);
  d_.goodFrames++;
  return true;
}

}  // namespace onspeed
