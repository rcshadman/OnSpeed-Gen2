# OnSpeed S3 ↔ MakerPlane (CAN-FiX): results

Scope, as the user narrowed it: a CAN-FiX encoder/decoder sitting behind a small swappable
interface, cross-checked against python-canfix. I skipped the FIX-Gateway in-process run on
purpose: MakerPlane is a stopgap until the custom display.

## Sources (primary, cloned 2026-10-25)

| Repo | Commit | License | How it was used |
|---|---|---|---|
| github.com/makerplane/canfix-spec | 3fb7dfe (2024-09-07), spec rev 0.7 | CC BY-NC-SA 3.0 US | Facts only (ids, types, scaling, frame layout). No text or code copied. |
| python-canfix 0.3.0 (PyPI; home github.com/birkelbach/python-canfix) | — | GPLv2+ | Test oracle on the host only. Not linked, not shipped. |
| github.com/makerplane/FIX-Gateway | ed08cb1 (2026-05-02) | GPLv2 | Read for the CAN id → FIX key map. |
| github.com/makerplane/pyEfis | 2e1cbf5 (2026-05-02) | GPLv2 | Read to see how AOA is displayed. |

`git clone https://github.com/makerplane/python-canfix` asked for credentials, so I installed it from PyPI instead.
`canfix.cpp` is a clean-room implementation written from the spec tables.

Spec files: `src/framedef.rst`, `src/requirements.rst`, `src/tables/{dataframe,functioncode,distribution,nodecontrolcode,nodeidresponse,nodestatus,db9pins}.csv`, `src/canfix.json`.

## Protocol facts used

- **Bus:** CAN 2.0A (11-bit ids). **125 kbit/s is the default.** 125k and 250k are the only mandatory rates. A Bit Rate Set command can pick 1=125k, 2=250k, 3=500k, 4=1M.
- **DB9 connector pinout:** pin 2 CAN-L, pin 7 CAN-H, pin 3 CAN GND, pin 1 +12–30 V, pin 5 power GND, pin 8 shield.
- **Id ranges:**
  - 1–255: node alarms
  - 0x100–0x5FF: parameter data. The CAN id *is* the parameter id. 0x180–0x1BF is high-priority flight data.
  - 0x600–0x6DF: reserved
  - 0x6E0–0x7DF: node-specific messages (id = 0x6E0 + node)
  - 0x7E0–0x7FF: two-way channels
- **Parameter frame layout:**
  - `data[0]` = sender node id
  - `data[1]` = index
  - `data[2]` = function/status byte: bit0 annunciate, bit1 quality (suspect), bit2 failure, bits 7..4 metadata selector (0 = the value itself)
  - `data[3..]` = value, little endian, raw = value / multiplier
  - DLC = 3 + size of the type
- **Node requirements:**
  - Must respond to Node Identification (code 0; reply `[0, dest, 0x01 spec rev, device, fw, model(3)]`), Bit Rate Set, Node ID Set, Disable/Enable Parameter (3/4) and Node Report (5).
  - Must send a frame carrying its node id at least every 2 s. The recommendation is Node Status (code 6, status word 0) at 1 Hz.

### Parameter table used (canfix.json rev 0.7, matched against python-canfix's copy by the test)

| Id | Parameter | Type | LSB | Range | FIX-Gateway key | Direction |
|---|---|---|---|---|---|---|
| 0x182 | Angle of Attack | INT | 0.01° | ±90 | `AOA` (aux Min, Max, 0g, Warn, Stall) | **TX** |
| 0x183 | Indicated Airspeed | UINT | 0.1 kt | 0–999.9 | `IAS` | TX (optional) and RX |
| 0x18D | True Airspeed | UINT | 0.1 kt | 0–999.9 | `TAS` | TX (optional) and RX |
| 0x180 | Pitch | INT | 0.01° | ±90 (+ = nose up) | `PITCH` | RX |
| 0x181 | Roll | INT | 0.01° | ±180 (+ = right) | `ROLL` | RX |
| 0x184 | Indicated Altitude | DINT | 1 ft | −1000–60000 | `ALT` | RX |
| 0x191 | Pressure Altitude | DINT | 1 ft | −1000–60000 | (not mapped by default) | RX (preferred for `palt`) |
| 0x185 | Heading (magnetic) | UINT | 0.1° | 0–359.9 | `HEAD` | RX |
| 0x186 | Vertical Speed | INT | 1 ft/min | ±30000 | `VS` | RX |
| 0x18A | Normal Accel | INT | 0.001 g | ±30 | `ANORM` exists in the database but has **no input mapping** in the default map.yaml | RX |
| 0x18B | Lateral Accel | INT | 0.001 g | ±30 | `ALAT` | RX |
| 0x406 | Total Air Temp | INT | 0.01 °C | — | `OAT` | RX |
| 0x407 | Static Air Temp | INT | 0.01 °C | ±300 | — | RX |

AOA metadata selectors (sent at 1 Hz, and immediately after a Node Report):

| Selector | Meaning | OnSpeed value sent |
|---|---|---|
| 1 Min, 2 Max | Gauge range | `gaugeMinAOA`, `gaugeMaxAOA` |
| 7 0g | Zero-lift AOA | `zeroLiftAOA` |
| 8 Warn | Warning threshold | `stallWarnAOA` |
| 9 Stall | Stall AOA | `stallAOA` |

FIX-Gateway writes these into the `AOA` key's aux values with the same names.

**What CAN-FiX has no place for:** L/Dmax, OnSpeed fast/slow, and percent lift. I send none of them. I did not invent ids, because that would break compliance; the spec reserves 0x600–0x6DF.

The annunciate bit is set on AOA while AOA ≥ stall-warn AOA.

**FIX-Gateway** (`plugins/canfix/mapping.py` + `config/canfix/map.yaml`) maps an (canid, index) pair to a FIX key. Its defaults are `node: 145`, interface socketcan/vcan0. Each frame's annunciate, quality and failure bits go to the database item.

### How pyEFIS shows AOA

pyEFIS has **no dedicated AOA indicator**: a grep of the whole repo for "aoa" only finds docs/coding.rst. AOA can still be shown with a generic arc, bar or numeric gauge bound to dbkey `AOA`. Those gauges read the aux values `Min`/`Max` for their range and `lowWarn`/`highWarn`/`lowAlarm`/`highAlarm` for colour bands. They ignore `Warn`/`Stall`/`0g`.

To get colour bands, change FIX-Gateway config:
- Add `AOA aux: [Min, Max, 0g, Warn, Stall, highWarn, highAlarm]` in `database/ahrs.yaml`.
- Add `Warn: highWarn`, `Stall: highAlarm` under `meta replacements` in map.yaml. Only AOA uses Warn/Stall, so the replacement is safe.

Nothing changes on the OnSpeed side.

### Prior OnSpeed ↔ MakerPlane integration?

**Not found.** I checked:
- the onspeed-gen2 clone (no canfix/makerplane references)
- FIX-Gateway, pyEfis and canfix-spec
- makerplane/Documentation and e100/mp-canfix-devices: no "onspeed" match in either
- web searches, which turned up only an unrelated VAF thread on an OnSpeed RV-4 with a Dynon EFIS

Firecrawl search was out of credits, so the web search was shallow. If the user has a link or name, it is worth checking.

## Files

- `components/onspeed_core/include/canfix.h`, `components/onspeed_core/canfix.cpp` (new). Pure C++, no IDF dependencies.
  - `canfix::encodeParam` / `decodeParam` / `encodeNodeStatus`: low-level encode/decode.
  - `OnSpeedOutputs` plus two interfaces: `DisplaySink::publish()` and `EfisSource::efis()/fresh()`. These are the swap points for a future custom display link.
  - `CanfixNode` implements both interfaces. It:
    - sends AOA every `publish()`, plus IAS/TAS when they are not NaN
    - sends setpoint metadata and Node Status at 1 Hz
    - answers Node Identification, Disable/Enable Parameter and Node Report
    - decodes EFIS frames into `EfisData` (from g5.h) plus TAS, OAT and indicated altitude, with valid/suspect masks and staleness (500 ms)
    - ignores its own frames and any indexed or metadata frames
- **CMake:** in `components/onspeed_core/CMakeLists.txt`, add `"canfix.cpp"` to SRCS:
  `idf_component_register(SRCS "onspeed_core.cpp" "tone_synth.cpp" "g5.cpp" "canfix.cpp" INCLUDE_DIRS "include")`
  The component does not need TWAI. Whichever component holds the TWAI glue adds `driver esp_timer` to its REQUIRES/PRIV_REQUIRES (IDF 5.4 legacy `driver/twai.h`).
- `hosttest/canfix/main_twai_example.cpp`: example glue.
  - Installs TWAI at `TWAI_TIMING_CONFIG_125KBITS()` with TX=GPIO17, RX=GPIO18.
  - Uses an accept-all filter. The needed ids (0x18x, 0x406, 0x6E0–0x7DF) don't fit a single code/mask, so `onFrame()` filters in software.
  - Runs a non-blocking RX loop with a 20 Hz publish and bus-off recovery.
- `hosttest/canfix/test_canfix.cpp`, `crosscheck.py`, `xtensa_syntax_check.py`, `run_tests.sh`.

## Encoding rules

| Input | What is sent |
|---|---|
| NaN or Inf | Failure bit set, payload 0 |
| Outside the spec range | Clamped to the limit, quality bit set |
| Outside the integer type's range | Clamped, quality bit set (e.g. TAT, which has no spec range) |
| Heading | Wrapped into [0, 360), so 359.97 → 0 |

- Rounding is round-half-even with exact decimal multipliers (double), so the bytes are identical to python-canfix's `round(value/mult)`.
- Received failure frames keep the last value but clear validity. Quality frames update the value and set the suspect flag.

## Test results (`./run_tests.sh`, all pass)

- **Build:** g++ -Wall -Wextra -Werror with ASan and UBSan.
- **C++ unit tests:** 130131/130131 checks passed.
  - Round trip of 26013 values across the 13 parameters, error ≤ ½ LSB.
  - Edge cases: exact bytes (AOA 12.34° → `50 00 00 D2 04`), negatives, exact limits, clamping plus quality bit, NaN/Inf → failure bit, heading wrap, half-even rounding, rejecting bad DLC/ids/meta.
  - Node behaviour: frame counts, annunciate, Node ID response, disable/enable plus NACK for unknown ids, Node Report, EFIS decode, stale/failed/suspect handling, own-node and metadata frames ignored.
- **Cross-check against python-canfix 0.3.0:**
  - Our ids, types and multipliers equal python-canfix's canfix.json.
  - **7861 frames our C++ encoded decode in python-canfix** to the expected values and flags. In-range frames are byte-identical to python-canfix's own encoder. Clamp, NaN and metadata cases included.
  - **6539 frames python-canfix encoded** (random nodes, function bits, metadata) **decode in our C++** to the same values.
  - Node Status and Node Identification frames parse in python-canfix.
- **xtensa-esp32s3-elf-g++** `-fsyntax-only -Werror`, using the project's IDF 5.4.2 flags: `canfix.cpp` and `main_twai_example.cpp` both OK. This was not built or run on hardware.
- **FIX-Gateway virtual-bus run:** skipped, per the scope change.

## Hardware suggestion

- **Transceiver:**
  - Bench: TI SN65HVD230 or TCAN334, both 3.3 V.
  - Aircraft: TI TCAN1044AV-Q1 with VCC 5 V and VIO 3.3 V. It has ±58 V bus-fault protection, which suits 14/28 V wiring.
  - Wire to the CAN-FiX DB9 (pin 7 H, pin 2 L, pin 3 GND).
  - Fit a 120 Ω termination only if the box sits at the end of the bus.
- **Pins:** TWAI TX = GPIO17, RX = GPIO18.
  - These avoid the prototype's 1, 5, 6, 7, 8, 9, 43, 44, plus strapping pins 0/3/45/46, USB 19/20 and flash/PSRAM 26–37.
  - Any GPIO can be routed through the matrix, so this is easy to change in the final pin map.
- **Node id:** default 0x50. It must be unique on the bus; FIX-Gateway uses 145.
