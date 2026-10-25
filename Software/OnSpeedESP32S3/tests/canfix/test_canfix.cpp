// Host tests for onspeed::canfix. Build: see run_tests.sh
//   canfix_test selftest
//   canfix_test encode IN OUT   IN lines: "id value flags meta"  -> OUT lines: "id dlc hexbytes"
//   canfix_test decode IN OUT   IN lines: "id dlc hexbytes"      -> OUT lines: "id node index fn value"
#include "canfix.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

using namespace onspeed;
using namespace onspeed::canfix;

static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void roundTrip() {
  const uint16_t ids[] = {PITCH, ROLL, AOA, IAS, ALT, HEAD, VS, ANORM, ALAT, TAS, PALT, TAT, SAT};
  int n = 0;
  for (uint16_t id : ids) {
    const ParamDef *p = findParam(id);
    double lo = p->min, hi = p->max;
    if (isnan(lo)) { lo = -327.68; hi = 327.67; }
    if (p->wrap360) hi = 359.9;
    for (int k = 0; k <= 2000; k++) {
      double v = lo + (hi - lo) * k / 2000.0;
      CanFrame f; Param d;
      CHECK(encodeParam(7, id, (float)v, f), "encode %x", id);
      CHECK(f.dlc == 3 + typeSize(p->type), "dlc %x", id);
      CHECK(decodeParam(f, d), "decode %x", id);
      double tol = p->mult * 0.5 + fabs(v) * 1e-6 + 1e-6;
      CHECK(fabs(d.value - v) <= tol, "rt %x v=%g got=%g", id, v, d.value);
      CHECK(!d.failure && !d.quality && !d.annunciate && d.node == 7 && d.meta == 0, "flags %x v=%g", id, v);
      n++;
    }
  }
  printf("round-trip: %d values over %zu parameters\n", n, sizeof ids / sizeof ids[0]);
}

static void edges() {
  CanFrame f; Param d;
  // exact bytes: AOA 12.34 deg -> 1234 = 0x04D2, node 0x50
  encodeParam(0x50, AOA, 12.34f, f);
  CHECK(f.id == 0x182 && f.dlc == 5 && f.data[0] == 0x50 && f.data[1] == 0 && f.data[2] == 0 &&
        f.data[3] == 0xD2 && f.data[4] == 0x04, "AOA bytes");
  // negative
  encodeParam(1, AOA, -5.5f, f); decodeParam(f, d);
  CHECK(f.data[3] == 0xDA && f.data[4] == 0xFD && fabsf(d.value + 5.5f) < 1e-4, "neg AOA %g", d.value);
  // limits exactly
  encodeParam(1, ROLL, 180, f); decodeParam(f, d);
  CHECK(d.value == 180 && !d.quality, "roll 180");
  encodeParam(1, ROLL, -180, f); decodeParam(f, d);
  CHECK(d.value == -180 && !d.quality, "roll -180");
  // out of spec range -> clamp + QUALITY
  encodeParam(1, AOA, 95, f); decodeParam(f, d);
  CHECK(fabsf(d.value - 90) < 1e-4 && d.quality && !d.failure, "AOA clamp hi");
  encodeParam(1, AOA, -1000, f); decodeParam(f, d);
  CHECK(fabsf(d.value + 90) < 1e-4 && d.quality, "AOA clamp lo");
  encodeParam(1, IAS, -3, f); decodeParam(f, d);
  CHECK(d.value == 0 && d.quality, "IAS neg clamp");
  encodeParam(1, IAS, 1200, f); decodeParam(f, d);
  CHECK(fabsf(d.value - 999.9f) < 1e-3 && d.quality, "IAS hi clamp");
  encodeParam(1, ALT, 75000, f); decodeParam(f, d);
  CHECK(d.value == 60000 && d.quality, "ALT clamp");
  encodeParam(1, ALT, -999, f); decodeParam(f, d);
  CHECK(d.value == -999 && f.dlc == 7 && !d.quality, "ALT neg DINT");
  // TAT has no spec range: INT limit clamp
  encodeParam(1, TAT, 400, f); decodeParam(f, d);
  CHECK(fabsf(d.value - 327.67f) < 1e-3 && d.quality, "TAT int clamp %g", d.value);
  // NaN / Inf -> FAILURE, zero payload
  encodeParam(1, AOA, NAN, f); decodeParam(f, d);
  CHECK(d.failure && d.value == 0 && f.data[3] == 0 && f.data[4] == 0, "NaN");
  encodeParam(1, PITCH, INFINITY, f); decodeParam(f, d);
  CHECK(d.failure, "Inf");
  // heading wrap
  encodeParam(1, HEAD, 360, f); decodeParam(f, d); CHECK(d.value == 0 && !d.quality, "hdg 360");
  encodeParam(1, HEAD, -10, f); decodeParam(f, d); CHECK(fabsf(d.value - 350) < 1e-3, "hdg -10");
  encodeParam(1, HEAD, 359.97f, f); decodeParam(f, d); CHECK(d.value == 0, "hdg 359.97");
  encodeParam(1, HEAD, 725.5f, f); decodeParam(f, d); CHECK(fabsf(d.value - 5.5f) < 1e-3, "hdg 725.5");
  // flags + meta
  encodeParam(9, AOA, 16.5f, f, FN_ANNUNCIATE, AOA_META_WARN); decodeParam(f, d);
  CHECK(f.data[2] == 0x81 && d.meta == 8 && d.annunciate, "meta warn");
  CHECK(!encodeParam(1, AOA, 1, f, 0, 16), "meta>15 rejected");
  CHECK(!encodeParam(1, 0x999, 1, f), "unknown id rejected");
  // round half even like python: 0.125 deg is not exact in float; use VS (mult 1)
  encodeParam(1, VS, 2.5f, f); decodeParam(f, d); CHECK(d.value == 2, "half-even 2.5 -> 2");
  encodeParam(1, VS, 3.5f, f); decodeParam(f, d); CHECK(d.value == 4, "half-even 3.5 -> 4");
  encodeParam(1, VS, -30000, f); decodeParam(f, d); CHECK(d.value == -30000, "VS min");
  // decoder rejects short dlc / unknown / alarm-range ids
  encodeParam(1, ALT, 100, f); f.dlc = 5; CHECK(!decodeParam(f, d), "short dlc");
  f.id = 0x0FF; f.dlc = 8; CHECK(!decodeParam(f, d), "alarm id");
  f.id = 0x6E5; CHECK(!decodeParam(f, d), "node specific id");
}

struct Bus { std::vector<CanFrame> tx; };
static bool busSend(const CanFrame &f, void *ctx) { ((Bus *)ctx)->tx.push_back(f); return true; }

static void node() {
  Bus bus;
  CanfixNode::Config cfg; cfg.node = 0x50;
  CanfixNode n(cfg, busSend, &bus);
  OnSpeedOutputs o;
  o.AOA = 10.2f; o.IAS = 72.3f; o.stallWarnAOA = 14.5f; o.zeroLiftAOA = -2.0f;
  o.gaugeMinAOA = -10; o.gaugeMaxAOA = 25; o.stallAOA = 17;
  n.publish(o, 0);
  // AOA, IAS, 5 meta, node status
  CHECK(bus.tx.size() == 8, "first publish frames %zu", bus.tx.size());
  Param d;
  decodeParam(bus.tx[0], d); CHECK(d.id == AOA && fabsf(d.value - 10.2f) < 1e-3 && !d.annunciate, "aoa");
  int metas = 0;
  for (auto &f : bus.tx) if (decodeParam(f, d) && d.id == AOA && d.meta) metas++;
  CHECK(metas == 5, "metas %d", metas);
  CHECK(bus.tx.back().id == 0x6E0 + 0x50 && bus.tx.back().data[0] == 6 && bus.tx.back().dlc == 5, "status");
  bus.tx.clear();
  o.AOA = 15; n.publish(o, 50);
  CHECK(bus.tx.size() == 2, "steady frames %zu", bus.tx.size());
  decodeParam(bus.tx[0], d); CHECK(d.annunciate, "annunciate above stall warn");
  bus.tx.clear();
  o.AOA = NAN; n.publish(o, 100); decodeParam(bus.tx[0], d); CHECK(d.failure, "nan aoa fail");
  bus.tx.clear();
  n.publish(o, 1000); CHECK(bus.tx.size() == 8, "periodic meta+status %zu", bus.tx.size());
  // Node identification request from node 145 (FIX-Gateway default) to us
  bus.tx.clear();
  CanFrame rq; rq.id = 0x6E0 + 145; rq.dlc = 2; rq.data[0] = 0; rq.data[1] = 0x50;
  n.onFrame(rq, 1001);
  CHECK(bus.tx.size() == 1 && bus.tx[0].id == 0x6E0 + 0x50 && bus.tx[0].dlc == 8 &&
        bus.tx[0].data[1] == 145 && bus.tx[0].data[2] == 1, "node id response");
  rq.data[1] = 0x33; bus.tx.clear(); n.onFrame(rq, 1002); CHECK(bus.tx.empty(), "not for us");
  // Disable IAS, then enable
  CanFrame dis; dis.id = 0x6E0 + 145; dis.dlc = 4; dis.data[0] = 3; dis.data[1] = 0x50;
  dis.data[2] = IAS & 0xFF; dis.data[3] = IAS >> 8;
  bus.tx.clear(); n.onFrame(dis, 1003);
  CHECK(bus.tx.size() == 1 && bus.tx[0].dlc == 3 && bus.tx[0].data[2] == 0, "disable ack");
  CHECK(!n.enabled(IAS), "IAS disabled");
  bus.tx.clear(); o.AOA = 5; n.publish(o, 1100); CHECK(bus.tx.size() == 1, "only AOA %zu", bus.tx.size());
  dis.data[0] = 4; bus.tx.clear(); n.onFrame(dis, 1101); CHECK(n.enabled(IAS), "IAS enabled");
  dis.data[2] = 0x80; dis.data[3] = 0x01; bus.tx.clear(); n.onFrame(dis, 1102);
  CHECK(bus.tx.size() == 1 && bus.tx[0].data[2] == 1, "unknown param nack");
  // Node report -> meta resent on next publish
  CanFrame rep; rep.id = 0x6E0 + 145; rep.dlc = 2; rep.data[0] = 5; rep.data[1] = 0;
  n.onFrame(rep, 1200); bus.tx.clear(); n.publish(o, 1201);
  CHECK(bus.tx.size() == 8, "report frames %zu", bus.tx.size());

  // EFIS input from an AHRS node 0x20
  CanFrame f;
  struct { uint16_t id; float v; } in[] = {{PITCH, 3.25f}, {ROLL, -20.5f}, {IAS, 95.4f}, {ALT, 4500},
      {HEAD, 271.3f}, {VS, -650}, {ANORM, 1.234f}, {ALAT, -0.051f}, {TAS, 101.2f}, {TAT, -12.5f}};
  uint32_t t = 2000;
  for (auto &e : in) { encodeParam(0x20, e.id, e.v, f); CHECK(n.onFrame(f, t), "rx %x", e.id); }
  const EfisData &ef = n.efis();
  CHECK(fabsf(ef.pitch - 3.25f) < 1e-3 && fabsf(ef.roll + 20.5f) < 1e-3 && fabsf(ef.IAS - 95.4f) < 1e-3, "att");
  CHECK(ef.palt == 4500 && ef.heading == 271 && ef.vsi == -650, "alt/hdg/vs %d %d %d", ef.palt, ef.heading, ef.vsi);
  CHECK(fabsf(ef.verticalG - 1.234f) < 1e-4 && fabsf(ef.lateralG + 0.051f) < 1e-4, "g");
  CHECK(fabsf(n.data().TAS - 101.2f) < 1e-3 && fabsf(n.data().OAT + 12.5f) < 1e-3, "tas/oat");
  CHECK(n.fresh(t + 100) && !n.fresh(t + 600), "freshness");
  encodeParam(0x20, PALT, 4380, f); n.onFrame(f, t); CHECK(ef.palt == 4380, "palt preferred");
  encodeParam(0x20, ALT, 4600, f); n.onFrame(f, t); CHECK(ef.palt == 4380 && n.data().indicatedAlt == 4600, "ind alt kept separate");
  // failure frame: value kept, validity dropped
  encodeParam(0x20, PITCH, NAN, f); CHECK(!n.onFrame(f, t + 10), "fail frame");
  CHECK(fabsf(ef.pitch - 3.25f) < 1e-3 && !n.fresh(t + 20), "fail -> not fresh");
  // quality frame accepted but flagged
  encodeParam(0x20, PITCH, 4, f, FN_QUALITY); n.onFrame(f, t + 30);
  CHECK(ef.pitch == 4 && (n.data().suspectMask & 1), "quality flagged");
  // own frames and meta/indexed frames ignored
  encodeParam(0x50, PITCH, 9, f); CHECK(!n.onFrame(f, t), "own node ignored");
  encodeParam(0x20, PITCH, 9, f, 0, 1); CHECK(!n.onFrame(f, t), "meta ignored");
  encodeParam(0x20, PITCH, 9, f, 0, 0, 1); CHECK(!n.onFrame(f, t), "index ignored");
  // DisplaySink interface usable polymorphically
  DisplaySink *sink = &n; bus.tx.clear(); sink->publish(o, 5000); CHECK(!bus.tx.empty(), "sink");
}

static int hexval(char c) { return c <= '9' ? c - '0' : (c | 0x20) - 'a' + 10; }

static int cmdEncode(const char *in, const char *out) {
  FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
  if (!fi || !fo) return 2;
  unsigned id, flags, meta; char vs[64];
  while (fscanf(fi, "%x %63s %u %u", &id, vs, &flags, &meta) == 4) {
    CanFrame f;
    if (!encodeParam(0x50, (uint16_t)id, strtof(vs, nullptr), f, (uint8_t)flags, (uint8_t)meta)) return 3;
    fprintf(fo, "%03x %d ", f.id, f.dlc);
    for (int i = 0; i < f.dlc; i++) fprintf(fo, "%02x", f.data[i]);
    fprintf(fo, "\n");
  }
  fclose(fi); fclose(fo);
  return 0;
}

static int cmdDecode(const char *in, const char *out) {
  FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
  if (!fi || !fo) return 2;
  unsigned id, dlc; char hx[40];
  while (fscanf(fi, "%x %u %39s", &id, &dlc, hx) == 3) {
    CanFrame f; f.id = (uint16_t)id; f.dlc = (uint8_t)dlc;
    for (unsigned i = 0; i < dlc; i++) f.data[i] = (uint8_t)(hexval(hx[2 * i]) * 16 + hexval(hx[2 * i + 1]));
    Param p;
    if (!decodeParam(f, p)) { fprintf(fo, "%03x ERR\n", id); continue; }
    fprintf(fo, "%03x %u %u %u %.9g\n", p.id, p.node, p.index,
            (unsigned)(f.data[2]), p.value);
  }
  fclose(fi); fclose(fo);
  return 0;
}

int main(int argc, char **argv) {
  if (argc == 4 && !strcmp(argv[1], "encode")) return cmdEncode(argv[2], argv[3]);
  if (argc == 4 && !strcmp(argv[1], "decode")) return cmdDecode(argv[2], argv[3]);
  roundTrip();
  edges();
  node();
  printf("%d/%d checks passed\n", checks - fails, checks);
  return fails ? 1 : 0;
}
