#include "canfix.h"
#include <math.h>
#include <string.h>

namespace onspeed {
namespace canfix {

static const ParamDef kParams[] = {
    {PITCH, Type::INT, 0.01, -90, 90, false},
    {ROLL, Type::INT, 0.01, -180, 180, false},
    {AOA, Type::INT, 0.01, -90, 90, false},
    {IAS, Type::UINT, 0.1, 0, 999.9f, false},
    {ALT, Type::DINT, 1, -1000, 60000, false},
    {HEAD, Type::UINT, 0.1, 0, 359.9f, true},
    {VS, Type::INT, 1, -30000, 30000, false},
    {ANORM, Type::INT, 0.001, -30, 30, false},
    {ALAT, Type::INT, 0.001, -30, 30, false},
    {TAS, Type::UINT, 0.1, 0, 999.9f, false},
    {CAS, Type::UINT, 0.1, 0, 999.9f, false},
    {PALT, Type::DINT, 1, -1000, 60000, false},
    {TAT, Type::INT, 0.01, kNaN, kNaN, false},   // spec gives no range; INT limits apply
    {SAT, Type::INT, 0.01, -300, 300, false},
};

const ParamDef *findParam(uint16_t id) {
  for (const auto &p : kParams)
    if (p.id == id) return &p;
  return nullptr;
}

int typeSize(Type t) {
  switch (t) {
    case Type::SHORT: case Type::USHORT: return 1;
    case Type::INT: case Type::UINT: return 2;
    default: return 4;
  }
}

static void typeLimits(Type t, double &lo, double &hi) {
  switch (t) {
    case Type::SHORT: lo = -128; hi = 127; break;
    case Type::USHORT: lo = 0; hi = 255; break;
    case Type::INT: lo = -32768; hi = 32767; break;
    case Type::UINT: lo = 0; hi = 65535; break;
    case Type::DINT: lo = -2147483648.0; hi = 2147483647.0; break;
    case Type::UDINT: lo = 0; hi = 4294967295.0; break;
    default: lo = -INFINITY; hi = INFINITY; break;
  }
}

bool encodeParam(uint8_t node, uint16_t id, float value, CanFrame &out, uint8_t flags,
                 uint8_t meta, uint8_t index) {
  const ParamDef *p = findParam(id);
  if (!p || meta > 15) return false;
  out = CanFrame();
  out.id = id;
  out.data[0] = node;
  out.data[1] = index;
  int n = typeSize(p->type);
  out.dlc = (uint8_t)(3 + n);
  flags &= 0x07;
  double v = value;
  if (!isfinite(v)) {
    flags |= FN_FAILURE;
    v = 0;
  } else {
    if (p->wrap360) {
      v = fmod(v, 360.0);
      if (v < 0) v += 360.0;
      if (nearbyint(v / p->mult) * p->mult >= 360.0) v = 0;   // 359.97 -> 360.0 -> 0
    } else {
      if (!isnan(p->min) && v < p->min) { v = p->min; flags |= FN_QUALITY; }
      if (!isnan(p->max) && v > p->max) { v = p->max; flags |= FN_QUALITY; }
    }
  }
  out.data[2] = (uint8_t)(flags | (meta << FN_META_SHIFT));
  uint8_t *d = out.data + 3;
  if (p->type == Type::FLOAT) {
    float f = (float)(v / p->mult);
    memcpy(d, &f, 4);   // ESP32 and x86 are little endian
    return true;
  }
  double raw = nearbyint(v / (double)p->mult);   // round-half-even, as python round()
  double lo, hi;
  typeLimits(p->type, lo, hi);
  if (raw < lo) { raw = lo; out.data[2] |= FN_QUALITY; }
  if (raw > hi) { raw = hi; out.data[2] |= FN_QUALITY; }
  int64_t r = (int64_t)raw;
  for (int i = 0; i < n; i++) d[i] = (uint8_t)((uint64_t)r >> (8 * i));
  return true;
}

bool decodeParam(const CanFrame &f, Param &p) {
  if (f.id < 0x100 || f.id > 0x6DF) return false;
  const ParamDef *def = findParam(f.id);
  if (!def) return false;
  int n = typeSize(def->type);
  if (f.dlc < 3 + n || f.dlc > 8) return false;
  p.id = f.id;
  p.node = f.data[0];
  p.index = f.data[1];
  uint8_t fn = f.data[2];
  p.annunciate = fn & FN_ANNUNCIATE;
  p.quality = fn & FN_QUALITY;
  p.failure = fn & FN_FAILURE;
  p.meta = fn >> FN_META_SHIFT;
  const uint8_t *d = f.data + 3;
  uint32_t u = 0;
  for (int i = 0; i < n; i++) u |= (uint32_t)d[i] << (8 * i);
  double raw;
  switch (def->type) {
    case Type::SHORT: raw = (int8_t)u; break;
    case Type::USHORT: raw = (uint8_t)u; break;
    case Type::INT: raw = (int16_t)u; break;
    case Type::UINT: raw = (uint16_t)u; break;
    case Type::DINT: raw = (int32_t)u; break;
    case Type::UDINT: raw = u; break;
    default: { float x; memcpy(&x, d, 4); raw = x; break; }
  }
  p.value = (float)(raw * def->mult);
  return true;
}

void encodeNodeStatus(uint8_t node, uint16_t statusWord, CanFrame &out) {
  out = CanFrame();
  out.id = NODE_SPECIFIC_BASE + node;
  out.dlc = 5;
  out.data[0] = 0x06;
  out.data[1] = 0;   // parameter type 0 = status word
  out.data[2] = 0;
  out.data[3] = statusWord & 0xFF;
  out.data[4] = statusWord >> 8;
}

}  // namespace canfix

// ---------------------------------------------------------------------------
using namespace canfix;

static int outBit(uint16_t id) { return id == AOA ? 0 : id == IAS ? 1 : id == TAS ? 2 : -1; }

bool CanfixNode::enabled(uint16_t id) const {
  int b = outBit(id);
  return b >= 0 && !(disabled_ & (1u << b));
}

void CanfixNode::sendParam(uint16_t id, float v, uint8_t flags, uint8_t meta) {
  if (!enabled(id)) return;
  CanFrame f;
  if (encodeParam(cfg_.node, id, v, f, flags, meta)) tx(f);
}

void CanfixNode::publish(const OnSpeedOutputs &o, uint32_t nowMs) {
  uint8_t q = o.dataSuspect ? FN_QUALITY : 0;
  uint8_t ann = (isfinite(o.AOA) && isfinite(o.stallWarnAOA) && o.AOA >= o.stallWarnAOA)
                    ? FN_ANNUNCIATE : 0;
  sendParam(AOA, o.AOA, q | ann);   // NaN AOA -> FAILURE bit
  if (!isnan(o.IAS)) sendParam(IAS, o.IAS < 0 ? 0 : o.IAS, q);
  if (!isnan(o.TAS)) sendParam(TAS, o.TAS < 0 ? 0 : o.TAS, q);

  if (metaDue_ || nowMs - lastMetaMs_ >= cfg_.metaPeriodMs) {
    metaDue_ = false;
    lastMetaMs_ = nowMs;
    const struct { uint8_t meta; float v; } m[] = {
        {AOA_META_MIN, o.gaugeMinAOA}, {AOA_META_MAX, o.gaugeMaxAOA},
        {AOA_META_0G, o.zeroLiftAOA},  {AOA_META_WARN, o.stallWarnAOA},
        {AOA_META_STALL, o.stallAOA}};
    for (const auto &e : m)
      if (isfinite(e.v)) sendParam(AOA, e.v, 0, e.meta);
    // LDmax / OnSpeed fast / slow have no CAN-FiX meta slot: not sent.
  }
  if (statusDue_ || nowMs - lastStatusMs_ >= cfg_.statusPeriodMs) {
    statusDue_ = false;
    lastStatusMs_ = nowMs;
    CanFrame f;
    encodeNodeStatus(cfg_.node, 0, f);
    tx(f);
  }
}

void CanfixNode::handleNodeSpecific(const CanFrame &f) {
  if (f.dlc < 2) return;
  uint8_t sender = (uint8_t)(f.id - NODE_SPECIFIC_BASE);
  uint8_t code = f.data[0], dest = f.data[1];
  bool forUs = dest == cfg_.node, bcast = dest == 0;
  CanFrame r;
  r.id = NODE_SPECIFIC_BASE + cfg_.node;
  r.data[0] = code;
  r.data[1] = sender;
  switch (code) {
    case 0x00:   // Node Identification request (dlc 2); responses are dlc 8
      if (f.dlc != 2 || !(forUs || bcast)) return;
      r.dlc = 8;
      r.data[2] = SPEC_REVISION;
      r.data[3] = cfg_.deviceType;
      r.data[4] = cfg_.fwRev;
      r.data[5] = cfg_.model & 0xFF;
      r.data[6] = (cfg_.model >> 8) & 0xFF;
      r.data[7] = (cfg_.model >> 16) & 0xFF;
      tx(r);
      return;
    case 0x03:   // Disable parameter (broadcast allowed)
    case 0x04: { // Enable parameter (addressed only)
      if (f.dlc != 4) return;
      if (!(forUs || (bcast && code == 0x03))) return;
      uint16_t id = f.data[2] | (f.data[3] << 8);
      int b = outBit(id);
      if (b >= 0) {
        if (code == 0x03) disabled_ |= (1u << b); else disabled_ &= ~(1u << b);
      } else if (bcast) {
        return;   // don't answer broadcasts for parameters we don't own
      }
      r.dlc = 3;
      r.data[2] = b >= 0 ? 0x00 : 0x01;
      tx(r);
      return;
    }
    case 0x05:   // Node Report
      if (forUs || bcast) metaDue_ = statusDue_ = true;
      return;
    default:
      return;    // Bit Rate Set / Node ID Set etc: configured locally, ignored
  }
}

void CanfixNode::updateValid(uint32_t nowMs) {
  uint16_t m = 0;
  for (int i = 0; i < CanfixEfisData::F_COUNT; i++)
    if ((d_.seenMask & (1u << i)) && nowMs - d_.lastMs[i] < cfg_.staleMs) m |= 1u << i;
  d_.validMask = m;
}

bool CanfixNode::onFrame(const CanFrame &f, uint32_t nowMs) {
  if (f.id >= NODE_SPECIFIC_BASE && f.id < NODE_SPECIFIC_BASE + 256) {
    if (f.id != NODE_SPECIFIC_BASE + cfg_.node) handleNodeSpecific(f);
    return false;
  }
  Param p;
  if (!decodeParam(f, p) || p.node == cfg_.node || p.index != 0 || p.meta != 0) return false;
  int fld;
  switch (p.id) {
    case PITCH: fld = CanfixEfisData::F_PITCH; break;
    case ROLL: fld = CanfixEfisData::F_ROLL; break;
    case IAS: fld = CanfixEfisData::F_IAS; break;
    case ALT: fld = CanfixEfisData::F_ALT; break;
    case PALT: fld = CanfixEfisData::F_PALT; break;
    case HEAD: fld = CanfixEfisData::F_HEAD; break;
    case VS: fld = CanfixEfisData::F_VS; break;
    case ANORM: fld = CanfixEfisData::F_ANORM; break;
    case ALAT: fld = CanfixEfisData::F_ALAT; break;
    case TAS: fld = CanfixEfisData::F_TAS; break;
    case TAT: case SAT: fld = CanfixEfisData::F_OAT; break;
    case AOA: fld = CanfixEfisData::F_AOA; break;
    default: return false;
  }
  uint16_t bit = 1u << fld;
  if (p.failure) {   // keep last value, drop validity
    d_.failedFrames++;
    d_.seenMask &= ~bit;
    updateValid(nowMs);
    return false;
  }
  float v = p.value;
  EfisData &e = d_.efis;
  switch (fld) {
    case CanfixEfisData::F_PITCH: e.pitch = v; break;
    case CanfixEfisData::F_ROLL: e.roll = v; break;
    case CanfixEfisData::F_IAS: e.IAS = v; break;
    case CanfixEfisData::F_ALT:
      d_.indicatedAlt = v;
      if (!(d_.seenMask & (1u << CanfixEfisData::F_PALT))) e.palt = (int)lroundf(v);
      break;
    case CanfixEfisData::F_PALT: e.palt = (int)lroundf(v); break;
    case CanfixEfisData::F_HEAD: e.heading = (int)lroundf(v) % 360; break;
    case CanfixEfisData::F_VS: e.vsi = (int)lroundf(v); break;
    case CanfixEfisData::F_ANORM: e.verticalG = v; break;
    case CanfixEfisData::F_ALAT: e.lateralG = v; break;
    case CanfixEfisData::F_TAS: d_.TAS = v; break;
    case CanfixEfisData::F_OAT: d_.OAT = v; break;
    case CanfixEfisData::F_AOA: d_.AOA = v; break;
  }
  if (p.quality) d_.suspectMask |= bit; else d_.suspectMask &= ~bit;
  d_.seenMask |= bit;
  d_.lastMs[fld] = nowMs;
  e.goodFrames++;
  updateValid(nowMs);
  return true;
}

bool CanfixNode::fresh(uint32_t nowMs) const {
  const uint16_t need = (1u << CanfixEfisData::F_PITCH) | (1u << CanfixEfisData::F_ROLL) |
                        (1u << CanfixEfisData::F_IAS);
  for (int i = 0; i < CanfixEfisData::F_COUNT; i++)
    if ((need & (1u << i)) && (!(d_.seenMask & (1u << i)) || nowMs - d_.lastMs[i] >= cfg_.staleMs))
      return false;
  return true;
}

}  // namespace onspeed
