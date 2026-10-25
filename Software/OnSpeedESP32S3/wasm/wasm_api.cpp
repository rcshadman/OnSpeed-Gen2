// WebAssembly entry points for the browser demo: the same onspeed_core sources that run on
// the ESP32-S3, compiled for wasm32 so the page can fly them live.
#include "onspeed_core.h"
#include "tone_synth.h"
#include "g5.h"
#include <string.h>
#include <math.h>
#define EXPORT(name) extern "C" __attribute__((export_name(#name)))
using namespace onspeed;

static Config g_cfg;
static Core *g_core;
static ToneSynth *g_synth;
static G5Parser g_g5;
static char g_line[128];
static int16_t g_pcm[2 * 4096];

EXPORT(os_init) void os_init(int sampleRate) {
  g_cfg.pFwdBias = 8192;  // simulated sensors are ideal: zero at mid-scale
  g_cfg.p45Bias = 8192;
  static Core core(g_cfg);
  static ToneSynth synth(sampleRate);
  g_core = &core;
  g_synth = &synth;
}
// One 50 Hz sensor cycle from raw sensor bytes, same order as the firmware's sensorTask.
EXPORT(os_step) void os_step(int f0, int f1, int p0, int p1, int s0, int s1, int s2, int s3, int flapPot, float latG) {
  uint8_t pf[2] = {(uint8_t)f0, (uint8_t)f1}, p45[2] = {(uint8_t)p0, (uint8_t)p1};
  uint8_t ps[4] = {(uint8_t)s0, (uint8_t)s1, (uint8_t)s2, (uint8_t)s3};
  g_core->sensorRead(parseDiffPressure(pf), parseDiffPressure(p45), parseStaticPressure(ps), flapPot);
  g_core->updateTones();
  const State &st = g_core->state();
  g_synth->setToneState(st.toneMode, st.highTone, st.pps);
  g_synth->update3DAudio(latG);
}
EXPORT(os_aoa) float os_aoa() { return g_core->state().AOA; }
EXPORT(os_ias) float os_ias() { return g_core->state().IAS; }
EXPORT(os_palt) float os_palt() { return g_core->state().Palt; }
EXPORT(os_tone_mode) int os_tone_mode() { return g_core->state().toneMode; }
EXPORT(os_high_tone) int os_high_tone() { return g_core->state().highTone; }
EXPORT(os_pps) float os_pps() { return g_core->state().pps; }
EXPORT(os_flaps) int os_flaps() { return g_core->state().flapsPos; }
EXPORT(os_percent_lift) int os_percent_lift() { return g_core->percentLift(); }
EXPORT(os_setpoint) float os_setpoint(int i) {
  const State &s = g_core->state();
  return i == 0 ? s.LDmaxAOA : i == 1 ? s.onSpeedAOAfast : i == 2 ? s.onSpeedAOAslow : s.stallWarningAOA;
}
// Garmin G5 sentence out and through the Gen2 parser; returns the sentence.
EXPORT(os_g5) const char *os_g5(int sec, float pitch, float roll, int hdg, float ias, int palt, float latG, float vertG, int vsi) {
  buildG5Sentence(g_line, sizeof g_line, 14, (sec / 60) % 60, sec % 60, 0, pitch, roll, hdg, ias, palt, latG, vertG, vsi);
  for (char *p = g_line; *p; p++) g_g5.feed(*p);
  g_line[strcspn(g_line, "\r\n")] = 0;
  return g_line;
}
EXPORT(os_g5_pitch) float os_g5_pitch() { return g_g5.data().pitch; }
EXPORT(os_g5_roll) float os_g5_roll() { return g_g5.data().roll; }
EXPORT(os_g5_latg) float os_g5_latg() { return g_g5.data().lateralG; }
EXPORT(os_g5_vertg) float os_g5_vertg() { return g_g5.data().verticalG; }
EXPORT(os_g5_vsi) int os_g5_vsi() { return g_g5.data().vsi; }
EXPORT(os_g5_good) int os_g5_good() { return (int)g_g5.data().goodFrames; }
// "#1" display sentence for the M5Stack, attitude from the G5 like the firmware.
EXPORT(os_display) const char *os_display(float fpa) {
  const EfisData &e = g_g5.data();
  g_core->buildDisplaySentence(g_line, sizeof g_line, e.pitch, e.roll, g_core->state().Palt, 0, e.lateralG, e.verticalG, e.vsi, fpa, 0);
  g_line[strcspn(g_line, "\r\n")] = 0;
  return g_line;
}
EXPORT(os_render) int16_t *os_render(int n) {
  if (n > 4096) n = 4096;
  g_synth->render(g_pcm, n);
  return g_pcm;
}
EXPORT(os_volume) void os_volume(float v) { g_synth->setMasterVolume(v); }

// ---- virtual sensors for the live mode: same transfer functions as main/sensors.cpp SimSensorBus ----
static uint32_t g_lcg = 12345, g_reads = 0;
static int simNoise() {
  g_lcg = g_lcg * 1664525u + 1013904223u;
  return (int)((g_lcg >> 16) % 5) - 2;
}
static void pack14(int counts, uint8_t b[2]) {
  if (counts < 0) counts = 0;
  if (counts > 16383) counts = 16383;
  b[0] = (counts >> 8) & 0x3f;
  b[1] = counts & 0xff;
}
// One live 50 Hz cycle: flight truth in, raw sensor bytes made, then the same os_step as the firmware loop.
EXPORT(os_fly) void os_fly(float ias, float aoa, float flapPot, float palt, float latG) {
  float iasInd = ias > 5 ? (ias - g_cfg.casCurve.Items[3]) / g_cfg.casCurve.Items[2] : ias;
  if (iasInd < 0) iasInd = 0;
  float v = iasInd / 1.94384f, q = 0.5f * 1.225f * v * v;
  int fwd = (int)lroundf(q / 6894.757f * (0.8f * 16383 / 2));
  int fi = g_core->state().flapsIndex;
  const float *c = g_cfg.aoaCurve[fi].Items;
  float A = c[1], B = c[2], C = c[3], disc = B * B - 4 * A * (C - aoa);
  float cp = disc > 0 ? (-B + sqrtf(disc)) / (2 * A) : -B / (2 * A);
  uint8_t pf[2], p45[2], ps[4];
  int raw = g_cfg.pFwdBias + fwd + simNoise();
  if (++g_reads % 97 == 0) raw += 400;
  pack14(raw, pf);
  pack14(g_cfg.p45Bias + (int)lroundf(cp * fwd) + simNoise(), p45);
  float mbar = 1013.25f * powf(1 - palt / 145366.45f, 1 / 0.190284f) - g_cfg.pStaticBias;
  int sr = (int)lroundf(mbar / 1000 * 8192 + 1638.4f), temp = 1024;
  ps[0] = (sr >> 8) & 0x3f; ps[1] = sr & 0xff; ps[2] = (temp << 5) >> 8; ps[3] = (temp << 5) & 0xff;
  os_step(pf[0], pf[1], p45[0], p45[1], ps[0], ps[1], ps[2], ps[3], (int)lroundf(flapPot), latG);
}
EXPORT(os_flap_pot) int os_flap_pot(int i) { return g_cfg.flapPotPositions[i]; }
