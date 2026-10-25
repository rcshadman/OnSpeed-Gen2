// CAN-FiX (MakerPlane open-source avionics CAN protocol) encoder/decoder.
//
// Written from the CAN-FiX specification (github.com/makerplane/canfix-spec,
// spec rev 0.7, canfix.json). No code copied from python-canfix / FIX-Gateway
// (GPLv2); python-canfix is used only as a test oracle on the host.
//
// Wire format (normal parameter frame, 11-bit standard id):
//   id      = CAN-FiX parameter id (0x100..0x6DF), e.g. 0x182 = Angle of Attack
//   data[0] = sending node id
//   data[1] = index (0 for non-indexed parameters)
//   data[2] = function/status: bit0 annunciate, bit1 quality(suspect),
//             bit2 failure, bits7..4 meta-data selector (0 = the value itself)
//   data[3..] value, little endian, raw = round(value / multiplier)
//   dlc     = 3 + sizeof(type)
//
// Pure C++, no ESP-IDF dependencies. The ESP-IDF TWAI glue lives outside the
// component (see hosttest/canfix/main_twai_example.cpp).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "g5.h"   // EfisData

namespace onspeed {

struct CanFrame {
  uint16_t id = 0;     // 11-bit standard identifier
  uint8_t dlc = 0;
  uint8_t data[8] = {0};
};

namespace canfix {

constexpr uint32_t DEFAULT_BITRATE = 125000;   // spec: 125k default, 125k/250k mandatory
constexpr uint16_t NODE_SPECIFIC_BASE = 0x6E0; // 1760 + node id
constexpr uint8_t  SPEC_REVISION = 0x01;       // Node Identification byte 2

// Function/status byte bits
constexpr uint8_t FN_ANNUNCIATE = 0x01;
constexpr uint8_t FN_QUALITY = 0x02;   // sender believes value is suspect
constexpr uint8_t FN_FAILURE = 0x04;   // sender knows value is bad
constexpr uint8_t FN_META_SHIFT = 4;

// Parameter ids used here (canfix-spec canfix.json rev 0.7)
enum : uint16_t {
  PITCH = 0x180,      // INT  0.01 deg, +nose up, -90..90
  ROLL = 0x181,       // INT  0.01 deg, +right, -180..180
  AOA = 0x182,        // INT  0.01 deg, -90..90; meta 1 Min,2 Max,7 0g,8 Warn,9 Stall
  IAS = 0x183,        // UINT 0.1 kt, 0..999.9
  ALT = 0x184,        // DINT 1 ft (indicated), -1000..60000
  HEAD = 0x185,       // UINT 0.1 deg magnetic, 0..359.9
  VS = 0x186,         // INT  1 ft/min, -30000..30000
  ANORM = 0x18A,      // INT  0.001 g, -30..30
  ALAT = 0x18B,       // INT  0.001 g, -30..30
  TAS = 0x18D,        // UINT 0.1 kt
  CAS = 0x18E,        // UINT 0.1 kt
  PALT = 0x191,       // DINT 1 ft pressure altitude
  TAT = 0x406,        // INT  0.01 degC total air temp (FIX-Gateway maps to "OAT")
  SAT = 0x407,        // INT  0.01 degC static air temp
};

// AOA meta-data selectors (canfix-spec)
enum : uint8_t { AOA_META_MIN = 1, AOA_META_MAX = 2, AOA_META_0G = 7, AOA_META_WARN = 8,
                 AOA_META_STALL = 9 };

enum class Type : uint8_t { SHORT, USHORT, INT, UINT, DINT, UDINT, FLOAT };

struct ParamDef {
  uint16_t id;
  Type type;
  double mult;      // exact decimal multiplier (double, so rounding matches python-canfix)
  float min, max;   // spec range; NaN = unspecified
  bool wrap360;     // heading: wrap instead of clamp
};

const ParamDef *findParam(uint16_t id);   // nullptr if not in our table
int typeSize(Type t);

// Encode one parameter frame. Rules:
//  - NaN/Inf value: FAILURE bit set, value bytes 0.
//  - Value outside spec min/max: clamped to the limit and QUALITY bit set.
//  - Heading wraps into [0,360).
//  - Rounding is round-half-even (matches python-canfix's round()).
// `flags` may carry FN_ANNUNCIATE / FN_QUALITY / FN_FAILURE from the caller.
// Returns false for an unknown id or meta > 15.
bool encodeParam(uint8_t node, uint16_t id, float value, CanFrame &out, uint8_t flags = 0,
                 uint8_t meta = 0, uint8_t index = 0);

struct Param {
  uint16_t id = 0;
  uint8_t node = 0, index = 0, meta = 0;
  bool annunciate = false, quality = false, failure = false;
  float value = 0;
};
// Decode a normal parameter frame (id 0x100..0x6DF, known id, dlc >= 3+size).
bool decodeParam(const CanFrame &f, Param &p);

// Node Status (control code 6), status word 0 = good. Recommended 1 Hz heartbeat;
// spec requires *some* frame carrying our node id at least every 2 s.
void encodeNodeStatus(uint8_t node, uint16_t statusWord, CanFrame &out);

}  // namespace canfix

// ---------------------------------------------------------------------------
// Swappable display/EFIS output layer. The OnSpeed core fills OnSpeedOutputs;
// any sink (CAN-FiX today, a custom display link later) consumes it.
constexpr float kNaN = __builtin_nanf("");

struct OnSpeedOutputs {
  float AOA = kNaN;          // deg, NaN = invalid
  float IAS = kNaN;          // kt, NaN = don't send / invalid
  float TAS = kNaN;          // kt, NaN = don't send
  // Setpoints (deg). NaN = not configured.
  float LDmaxAOA = kNaN, onSpeedFastAOA = kNaN, onSpeedSlowAOA = kNaN;
  float stallWarnAOA = kNaN, stallAOA = kNaN, zeroLiftAOA = kNaN;
  float gaugeMinAOA = kNaN, gaugeMaxAOA = kNaN;
  float percentLift = kNaN;  // 0..100; no CAN-FiX slot (not transmitted)
  bool dataSuspect = false;     // e.g. sensor out of calibration -> QUALITY bit
};

class DisplaySink {
 public:
  virtual ~DisplaySink() = default;
  virtual void publish(const OnSpeedOutputs &o, uint32_t nowMs) = 0;
};

class EfisSource {
 public:
  virtual ~EfisSource() = default;
  virtual const EfisData &efis() const = 0;
  virtual bool fresh(uint32_t nowMs) const = 0;
};

// EfisData plus what CAN-FiX additionally carries.
struct CanfixEfisData {
  enum Field { F_PITCH, F_ROLL, F_IAS, F_ALT, F_PALT, F_HEAD, F_VS, F_ANORM, F_ALAT, F_TAS,
               F_OAT, F_AOA, F_COUNT };
  EfisData efis;            // IAS, pitch, roll, lateralG, verticalG, heading, palt, vsi
  float TAS = 0, OAT = 0, AOA = 0, indicatedAlt = 0;
  uint16_t validMask = 0;   // bit per Field: fresh (< staleMs) and not failed
  uint16_t suspectMask = 0; // QUALITY bit seen on last update
  uint16_t seenMask = 0;    // ever received
  uint32_t lastMs[F_COUNT] = {0};
  uint32_t failedFrames = 0;
};

typedef bool (*CanSendFn)(const CanFrame &f, void *ctx);

// CAN-FiX node: implements DisplaySink (AOA/IAS out) and EfisSource (attitude/air
// data in) and the mandatory node-specific responses (Node Identification,
// Disable/Enable Parameter, Node Report). Bit Rate Set / Node ID Set are
// acknowledged as failed (no response) since we keep them in our own config.
class CanfixNode : public DisplaySink, public EfisSource {
 public:
  struct Config {
    uint8_t node = 0x50;        // pick a free node id (1..255); FIX-Gateway defaults to 145
    uint8_t deviceType = 0x50;
    uint8_t fwRev = 1;
    uint32_t model = 0x4F5300;  // 'O''S'
    uint32_t metaPeriodMs = 1000;
    uint32_t statusPeriodMs = 1000;
    uint32_t staleMs = 500;     // EFIS field considered stale after this
  };
  CanfixNode(const Config &c, CanSendFn send, void *ctx) : cfg_(c), send_(send), ctx_(ctx) {}

  // DisplaySink: sends AOA (+IAS/TAS if given) every call; setpoint meta and
  // node status at their periods (or immediately after a Node Report request).
  void publish(const OnSpeedOutputs &o, uint32_t nowMs) override;
  // Feed every received frame. Returns true if it updated EFIS data.
  bool onFrame(const CanFrame &f, uint32_t nowMs);

  const EfisData &efis() const override { return d_.efis; }
  bool fresh(uint32_t nowMs) const override;   // pitch, roll and IAS fresh
  const CanfixEfisData &data() const { return d_; }
  uint32_t framesSent() const { return sent_; }
  bool enabled(uint16_t id) const;

 private:
  void tx(const CanFrame &f) { if (send_ && send_(f, ctx_)) sent_++; }
  void sendParam(uint16_t id, float v, uint8_t flags, uint8_t meta = 0);
  void handleNodeSpecific(const CanFrame &f);
  void updateValid(uint32_t nowMs);
  Config cfg_;
  CanSendFn send_;
  void *ctx_;
  CanfixEfisData d_;
  uint32_t sent_ = 0, lastMetaMs_ = 0, lastStatusMs_ = 0;
  bool metaDue_ = true, statusDue_ = true;
  uint8_t disabled_ = 0;   // bit0 AOA, bit1 IAS, bit2 TAS
};

}  // namespace onspeed
