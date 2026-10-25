// Host unit tests for onspeed_core: parsing, filters, AOA, tone rules, display sentence,
// G5 parser and the tone synth. Build: make (runs under ASan/UBSan).
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <string>
#include <algorithm>
#include "onspeed_core.h"
#include "tone_synth.h"
#include "g5.h"
using namespace onspeed;

static int g_fail = 0, g_pass = 0;
static std::string g_group;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL %s:%d [%s] ", __FILE__, __LINE__, g_group.c_str()); printf(__VA_ARGS__); printf("\n"); } } while (0)
static void group(const char *n) { g_group = n; printf("- %s\n", n); }

static void pack14(int c, uint8_t b[2], int status = 0) { b[0] = (uint8_t)((status << 6) | ((c >> 8) & 0x3f)); b[1] = c & 0xff; }
static void packStatic(float mbar, uint8_t b[4], int status = 0) {
  int raw = (int)lroundf(mbar / 1000 * 8192 + 1638.4f);
  b[0] = (uint8_t)((status << 6) | ((raw >> 8) & 0x3f)); b[1] = raw & 0xff; int t = 1024; b[2] = (t << 5) >> 8; b[3] = (t << 5) & 0xff;
}

static void testParsing() {
  group("Honeywell byte parsing");
  uint8_t b[2];
  for (int c : {0, 1, 1638, 8192, 14745, 16383}) for (int st = 0; st < 4; st++) {
    pack14(c, b, st);
    CHECK(parseDiffPressure(b) == c, "counts %d status %d -> %d", c, st, parseDiffPressure(b));
  }
  uint8_t s[4];
  for (float mb : {200.f, 500.f, 800.f, 1013.25f, 1100.f, 1500.f}) {
    packStatic(mb, s);
    float back = parseStaticPressure(s);
    CHECK(fabsf(back - mb) < 0.07f, "static %.2f mbar -> %.3f (1 count = 0.122 mbar)", mb, back);
  }
  // Datasheet transfer function for SSCSRNN1.6BA7A3: P = (counts - 1638) * 1.6 bar / (14746 - 1638)
  for (int c : {1638, 8192, 14745}) {
    s[0] = c >> 8; s[1] = c & 255; s[2] = 0; s[3] = 0;
    float ds = (c - 1638.4f) * 1600.f / 13107.2f;
    CHECK(fabsf(parseStaticPressure(s) - ds) < 0.01f, "datasheet formula at %d: %.3f vs %.3f", c, parseStaticPressure(s), ds);
  }
}

static void testFilter() {
  group("pressure filter: median 15 then average 10");
  PressureFilter f; f.init(15);
  float out = 0;
  for (int i = 0; i < 40; i++) out = f.add(1000);
  CHECK(out == 1000, "settles to input");
  // isolated spikes of any size are rejected completely
  for (int k = 0; k < 200; k++) { float v = (k % 9 == 4) ? 1e6f : 1000; out = f.add(v); CHECK(out == 1000, "spike leaked: %f", out); }
  // up to 7 consecutive bad samples in 15 are still rejected (median of 15)
  for (int i = 0; i < 7; i++) out = f.add(-5000);
  CHECK(out == 1000, "7-sample burst leaked: %f", out);
  for (int i = 0; i < 30; i++) f.add(1000);
  // step response: count cycles until within 1% of a step
  int n = 0; do { out = f.add(2000); n++; } while (fabsf(out - 2000) > 10 && n < 100);
  CHECK(n >= 8 && n <= 20, "step settle %d cycles", n);
  printf("    step settles in %d cycles (%.0f ms at 50 Hz)\n", n, n * 20.0);
}

static void testCurve() {
  group("AOA curve and flaps");
  Config cfg;
  for (float cp : {-0.2f, 0.f, 0.3f, 0.6f}) {
    const float *c = cfg.aoaCurve[0].Items;
    float want = c[0] * cp * cp * cp + c[1] * cp * cp + c[2] * cp + c[3];
    CHECK(fabsf(curveCalc(cp, cfg.aoaCurve[0]) - want) < 1e-4f, "poly at %f", cp);
  }
  CalibrationCurve lg = {{0, 0, 2, 3}, 2}, ex = {{0, 0, 2, 0.5f}, 3};
  CHECK(fabsf(curveCalc(2.f, lg) - (2 * logf(2) + 3)) < 1e-5f, "log curve");
  CHECK(fabsf(curveCalc(2.f, ex) - (2 * expf(1))) < 1e-5f, "exp curve");
  Core core(cfg);
  struct { int pot, idx; } cases[] = {{0, 0}, {129, 0}, {143, 0}, {144, 1}, {158, 1}, {182, 1}, {183, 2}, {206, 2}, {1023, 2}};
  for (auto &k : cases) {
    core.sensorRead(8192, 8192, 1013, k.pot);
    CHECK(core.state().flapsIndex == k.idx, "pot %d -> index %d, want %d", k.pot, core.state().flapsIndex, k.idx);
  }
  Config rev = cfg; rev.flapPotPositions[0] = 206; rev.flapPotPositions[2] = 129;
  Core r(rev); r.sensorRead(8192, 8192, 1013, 200); CHECK(r.state().flapsIndex == 0, "reversed pot order");
}

// Counts that make a given IAS and true AOA, same physics as the simulator
static void countsFor(const Config &cfg, float ias, float aoa, int fi, int &fwd, int &p45) {
  float iasInd = (ias - cfg.casCurve.Items[3]) / cfg.casCurve.Items[2];
  float v = iasInd / 1.94384f, q = 0.5f * 1.225f * v * v;
  int c = (int)lroundf(q / 6894.757f * (0.8f * 16383 / 2));
  const float *k = cfg.aoaCurve[fi].Items;
  float A = k[1], B = k[2], C = k[3], d = B * B - 4 * A * (C - aoa);
  float cp = (-B + sqrtf(d)) / (2 * A);
  fwd = cfg.pFwdBias + c; p45 = cfg.p45Bias + (int)lroundf(cp * c);
}

static void testAOA() {
  group("AOA and IAS from counts");
  Config cfg; cfg.pFwdBias = cfg.p45Bias = 8192;
  float worst = 0, worstIas = 0;
  for (float ias = 50; ias <= 200; ias += 10) for (float aoa = -2; aoa <= 18; aoa += 2) {
    if (ias > 120 && aoa > 8) continue;  // outside the envelope
    Core core(cfg); int f, p;
    countsFor(cfg, ias, aoa, 0, f, p);
    for (int i = 0; i < 200; i++) core.sensorRead(f, p, 1013.25f, 129);
    worst = std::max(worst, fabsf(core.state().AOA - aoa));
    worstIas = std::max(worstIas, fabsf(core.state().IAS - ias));
  }
  CHECK(worst < 0.25f, "steady AOA error %.3f deg", worst);
  CHECK(worstIas < 1.0f, "steady IAS error %.2f kt", worstIas);
  printf("    steady-state error over 50-200 kt: AOA %.3f deg, IAS %.2f kt (count quantisation)\n", worst, worstIas);
  // AOA resolution at low speed: one count of P45 is how many degrees?
  for (float ias : {50.f, 60.f, 80.f}) {
    int f, p; countsFor(cfg, ias, 12, 0, f, p);
    float cp = (float)(p - 8192) / (f - 8192), cp1 = (float)(p + 1 - 8192) / (f - 8192);
    float d = curveCalc(cp1, cfg.aoaCurve[0]) - curveCalc(cp, cfg.aoaCurve[0]);
    printf("    at %.0f kt one P45 count = %.3f deg AOA (Pfwd %d counts above zero)\n", ias, d, f - 8192);
  }
  Core core(cfg);
  for (int i = 0; i < 50; i++) core.sensorRead(8192, 8192, 1013.25f, 129);
  // Gen2 quirk kept: zero differential pressure still reads ~5.9 kt because of the CAS curve offset (+4.16 kt)
  CHECK(core.state().IAS < 10 && core.state().AOA == AOA_MIN_VALUE, "zero airspeed: IAS %.2f (below the 25 kt mute), AOA floor", core.state().IAS);
  CHECK(fabsf(core.state().Palt - (145366.45f * (1 - powf((1013.25f - 2.1997f) / 1013.25f, 0.190284f)))) < 0.5f, "Palt formula");
}

static void testTones() {
  group("tone rules (Tones.ino updateTones)");
  Config cfg; cfg.pFwdBias = cfg.p45Bias = 8192;
  const float ld = cfg.LDmaxAOA[0], fa = cfg.onSpeedFastAOA[0], sl = cfg.onSpeedSlowAOA[0], st = cfg.stallWarnAOA[0];
  struct { float aoa; ToneMode m; bool high; float pps; } cases[] = {
      {ld - 0.05f, TONE_OFF, false, 20}, {ld + 0.05f, PULSE_TONE, false, 1.5f}, {fa - 0.05f, PULSE_TONE, false, 8.2f},
      {fa + 0.05f, SOLID_TONE, false, 8.2f}, {sl - 0.05f, SOLID_TONE, false, 8.2f}, {sl + 0.05f, PULSE_TONE, true, 1.5f},
      {st - 0.05f, PULSE_TONE, true, 6.2f}, {st + 0.05f, PULSE_TONE, true, 20}, {30, PULSE_TONE, true, 20}};
  for (auto &k : cases) {
    Core core(cfg); int f, p;
    for (int i = 0; i < 400; i++) {
      // drive true AOA so the filtered value lands on the target, IAS consistent with 1 g lift
      float ias = 249.5f * sqrtf(1 / std::max(0.5f, k.aoa + 2));
      countsFor(cfg, std::max(ias, 45.f), k.aoa, 0, f, p);
      core.sensorRead(f, p, 1013.25f, 129);
    }
    core.updateTones();
    const State &s = core.state();
    float a = s.AOA;
    CHECK(s.toneMode == k.m && s.highTone == k.high, "AOA %.3f (target %.2f): mode %d high %d, want %d %d", a, k.aoa, s.toneMode, s.highTone, k.m, k.high);
    if (k.m != TONE_OFF) CHECK(fabsf(s.pps - k.pps) < 0.3f, "AOA %.2f pps %.2f want ~%.2f", a, s.pps, k.pps);
  }
  // mute under 25 kt regardless of AOA
  Core core(cfg); int f, p; countsFor(cfg, 24, 16, 0, f, p);
  for (int i = 0; i < 300; i++) core.sensorRead(f, p, 1013.25f, 129);
  core.updateTones(); CHECK(core.state().toneMode == TONE_OFF, "muted at %.1f kt", core.state().IAS);
}

static bool sentenceOk(const char *s) {
  if (strlen(s) != 80 || s[78] != '\r' || s[79] != '\n' || strncmp(s, "#1", 2)) return false;
  unsigned crc = 0; for (int i = 0; i <= 75; i++) crc += (uint8_t)s[i];
  char h[3]; snprintf(h, 3, "%02X", crc & 255); return !strncmp(h, s + 76, 2);
}

static void testDisplaySentence() {
  group("display sentence #1 (DisplaySerial.ino)");
  Config cfg; cfg.pFwdBias = cfg.p45Bias = 8192;
  Core core(cfg); int f, p; countsFor(cfg, 70, 12, 0, f, p);
  for (int i = 0; i < 300; i++) core.sensorRead(f, p, 900, 129);
  char out[128];
  core.buildDisplaySentence(out, sizeof out, 5.5f, -30.2f, 3000, 1.5f, 0.05f, 1.4f, -500, -2.5f, 3);
  CHECK(sentenceOk(out), "normal sentence: %s", out);
  CHECK(!strncmp(out, "#1+055-0302", 11), "pitch/roll fields: %.11s", out);
  // field positions the M5 parser uses
  CHECK(atoi(std::string(out + 26, 3).c_str()) == -5, "lateral G field (sign flipped) %.3s", out + 26);
  CHECK(atoi(std::string(out + 29, 3).c_str()) == 14, "vertical G field %.3s", out + 29);
  // extreme and invalid inputs must keep every field in place
  float bad[] = {NAN, INFINITY, -INFINITY, 1e9f, -1e9f, 0};
  int n = 0, okc = 0;
  for (float v : bad) for (int which = 0; which < 8; which++) {
    float a[8] = {5, -30, 3000, 1.5f, 0.05f, 1.4f, -500, -2.5f}; a[which] = v;
    core.buildDisplaySentence(out, sizeof out, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], 99);
    n++; okc += sentenceOk(out) && !strncmp(out + 52, "+164", 4);
  }
  CHECK(okc == n, "%d of %d extreme-value sentences kept their layout", okc, n);
}

static void testG5() {
  group("Garmin G5 parser (EfisSerial.ino)");
  G5Parser g; char line[80];
  buildG5Sentence(line, sizeof line, 12, 34, 56, 78, -4.5f, 33.3f, 271, 123.4f, 4500, -0.03f, 1.3f, -750);
  CHECK(strlen(line) == 59, "sentence length %zu", strlen(line));
  int got = 0; for (char *c = line; *c; c++) got += g.feed(*c);
  // first line is ignored until a LF has been seen, then parsed
  for (char *c = line; *c; c++) got += g.feed(*c);
  const EfisData &d = g.data();
  CHECK(got == 1 || got == 2, "parsed lines %d", got);
  CHECK(fabsf(d.pitch + 4.5f) < 0.01f && fabsf(d.roll - 33.3f) < 0.01f && d.heading == 271, "attitude %f %f %d", d.pitch, d.roll, d.heading);
  CHECK(fabsf(d.IAS - 123.4f) < 0.01f && d.palt == 4500 && d.vsi == -750, "air data %f %d %d", d.IAS, d.palt, d.vsi);
  CHECK(fabsf(d.lateralG + 0.03f) < 1e-4f && fabsf(d.verticalG - 1.3f) < 1e-4f, "g %f %f", d.lateralG, d.verticalG);
  // corrupted byte: CRC rejects it and keeps the last good data
  uint32_t bad0 = d.badFrames; std::string c(line); c[20] = c[20] == '1' ? '2' : '1';
  for (char ch : c) g.feed(ch);
  CHECK(d.badFrames == bad0 + 1 && d.heading == 271, "corrupt frame rejected");
  // random noise, overlong lines and truncated lines must not crash or parse
  srand(7); uint32_t good0 = d.goodFrames;
  for (int i = 0; i < 200000; i++) g.feed((char)(rand() & 0xff));
  for (int i = 0; i < 1000; i++) g.feed('=');
  g.feed('\n');
  CHECK(d.goodFrames == good0, "noise produced %u frames", d.goodFrames - good0);
  for (char *c2 = line; *c2; c2++) g.feed(*c2);
  CHECK(d.goodFrames == good0 + 1, "resyncs after noise");
}

static void testFaults() {
  group("sensor faults (port addition)");
  Config cfg; cfg.pFwdBias = cfg.p45Bias = 8192;
  Core core(cfg); int f, p; countsFor(cfg, 65, 12.5f, 0, f, p);
  uint8_t pf[2], pa[2], ps[4]; pack14(f, pf); pack14(p, pa); packStatic(900, ps);
  for (int i = 0; i < 300; i++) core.sensorReadBytes(pf, true, pa, true, ps, true, 129);
  core.updateTones(); float a0 = core.state().AOA;
  CHECK(core.state().toneMode == SOLID_TONE, "OnSpeed before the fault");
  // short dropout (10 reads): last good value held, tone unchanged
  for (int i = 0; i < 10; i++) core.sensorReadBytes(pf, false, pa, true, ps, true, 129);
  core.updateTones();
  CHECK(!core.state().sensorFault && core.state().toneMode == SOLID_TONE && fabsf(core.state().AOA - a0) < 0.05f, "short dropout rides through");
  // stale-status reads are counted but accepted
  pack14(f, pf, 2); for (int i = 0; i < 20; i++) core.sensorReadBytes(pf, true, pa, true, ps, true, 129);
  CHECK(core.state().pfwdHealth.stale == 20 && !core.state().sensorFault, "stale reads accepted");
  // diagnostic status for 0.5 s: fault, tones muted, display AOA zero
  pack14(16383, pa, 3); for (int i = 0; i < SENSOR_FAULT_READS; i++) core.sensorReadBytes(pf, true, pa, true, ps, true, 129);
  core.updateTones();
  CHECK(core.state().sensorFault && core.state().toneMode == TONE_OFF, "fault mutes tones");
  char out[128]; core.buildDisplaySentence(out, sizeof out, 0, 0, 0, 0, 0, 1, 0, 0, 0);
  CHECK(!strncmp(out + 34, "+000", 4) && !strncmp(out + 32, "00", 2), "display shows 0 AOA / 0%% lift on fault: %.6s", out + 32);
  // recovery
  pack14(p, pa); for (int i = 0; i < 100; i++) core.sensorReadBytes(pf, true, pa, true, ps, true, 129);
  core.updateTones(); CHECK(!core.state().sensorFault && core.state().toneMode == SOLID_TONE, "recovers");
  // a bus that never answered from power-up: fault immediately, no garbage AOA
  Core cold(cfg); uint8_t z[4] = {0xff, 0xff, 0xff, 0xff};
  for (int i = 0; i < 5; i++) cold.sensorReadBytes(z, false, z, false, z, false, 129);
  cold.updateTones(); CHECK(cold.state().sensorFault && cold.state().toneMode == TONE_OFF && !isnan(cold.state().Palt), "cold bus failure");
}

// ---- tone synth: frequency, pulse rate, level, clicks ----
static float goertzel(const std::vector<float> &x, float f, int sr) {
  float w = 2 * M_PI * f / sr, c = 2 * cosf(w), s1 = 0, s2 = 0;
  for (float v : x) { float s = v + c * s1 - s2; s2 = s1; s1 = s; }
  return sqrtf(s1 * s1 + s2 * s2 - c * s1 * s2) / x.size();
}
static void testSynth() {
  group("tone synth (Tones.ino + Teensy Audio envelope)");
  const int SR = 22050;
  struct { ToneMode m; bool hi; float pps; int hz; } cases[] = {{SOLID_TONE, false, 8.2f, 400}, {PULSE_TONE, false, 1.5f, 400}, {PULSE_TONE, false, 8.2f, 400}, {PULSE_TONE, true, 3, 1600}, {PULSE_TONE, true, 20, 1600}};
  for (auto &k : cases) {
    ToneSynth s(SR); s.setToneState(k.m, k.hi, k.pps);
    // render in 20 ms blocks like the audio task, draining the event queue (32 deep) each block
    std::vector<int16_t> pcm(2 * SR * 4); int starts = 0; onspeed::ToneEvent e;
    for (int i = 0; i < SR * 4; i += 441) { s.render(pcm.data() + 2 * i, std::min(441, SR * 4 - i)); while (s.popEvent(e)) starts++; }
    std::vector<float> l(SR * 4); int peak = 0; float maxStep = 0;
    for (int i = 0; i < SR * 4; i++) { l[i] = pcm[2 * i] / 32768.f; peak = std::max(peak, abs((int)pcm[2 * i])); if (i) maxStep = std::max(maxStep, fabsf(l[i] - l[i - 1])); }
    std::vector<float> tail(l.begin() + SR, l.end());
    float on = goertzel(tail, k.hz, SR), off = goertzel(tail, k.hz == 400 ? 1600 : 400, SR);
    CHECK(on > 20 * off, "%d Hz dominant (%.4f vs %.4f)", k.hz, on, off);
    // a click shows up as a step bigger than the steepest sine slope at full amplitude
    float slope = 2 * M_PI * k.hz / SR * (peak / 32768.f);
    CHECK(maxStep <= slope * 1.05f + 1e-3f, "%d Hz %.1f pps: max step %.4f vs sine slope %.4f", k.hz, k.pps, maxStep, slope);
    CHECK(peak < 32767, "no clipping, peak %d", peak);
    if (k.m == PULSE_TONE) {
      float rate = starts / 4.0f;
      CHECK(fabsf(rate - k.pps) <= std::max(0.5f, k.pps * 0.1f), "pulse rate %.2f/s want %.2f", rate, k.pps);
      printf("    %4d Hz %4.1f pps: %.2f pulses/s, peak %d (%.1f dBFS)\n", k.hz, k.pps, rate, peak, 20 * log10f(peak / 32768.f));
    } else printf("    %4d Hz solid: peak %d (%.1f dBFS)\n", k.hz, peak, 20 * log10f(peak / 32768.f));
  }
  ToneSynth s(SR); s.setToneState(TONE_OFF, false, 20);
  std::vector<int16_t> pcm(2 * SR); s.render(pcm.data(), SR);
  CHECK(std::all_of(pcm.begin(), pcm.end(), [](int16_t v) { return v == 0; }), "silent when off");
  // 3D audio: gains stay bounded and pan the right way
  ToneSynth p(SR); p.setToneState(SOLID_TONE, false, 8.2f);
  for (int i = 0; i < 400; i++) p.update3DAudio(0.05f);
  CHECK(p.gainRight() > p.gainLeft() && p.gainRight() <= 2.0f, "pans right for +latG (L %.2f R %.2f)", p.gainLeft(), p.gainRight());
  for (int i = 0; i < 400; i++) p.update3DAudio(1.0f);
  CHECK(p.gainLeft() >= 0 && p.gainRight() <= 2.0f, "large latG stays bounded (L %.2f R %.2f)", p.gainLeft(), p.gainRight());
}

int main() {
  testParsing(); testFilter(); testCurve(); testAOA(); testTones(); testDisplaySentence(); testG5(); testFaults(); testSynth();
  printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
