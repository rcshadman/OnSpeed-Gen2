#!/usr/bin/env python3
"""Cross-validate onspeed::canfix (C++) against python-canfix (makerplane reference, GPLv2).
Usage: crosscheck.py ./canfix_test WORKDIR
python-canfix is only used as a test oracle; nothing from it is linked into the firmware."""
import math, os, random, subprocess, sys
import struct
import can, canfix

def f32(x):  # firmware API takes float32; feed both sides the identical value
    return struct.unpack('<f', struct.pack('<f', x))[0]

exe, work = sys.argv[1], sys.argv[2]
os.makedirs(work, exist_ok=True)
random.seed(1234)
# id -> (min, max) test ranges (spec ranges)
P = {0x180: (-90, 90), 0x181: (-180, 180), 0x182: (-90, 90), 0x183: (0, 999.9), 0x184: (-1000, 60000),
     0x185: (0, 359.9), 0x186: (-30000, 30000), 0x18A: (-30, 30), 0x18B: (-30, 30), 0x18D: (0, 999.9),
     0x191: (-1000, 60000), 0x406: (-327.68, 327.67), 0x407: (-300, 300)}
fails = 0
def fail(msg):
    global fails
    fails += 1
    if fails < 30: print("FAIL", msg)

# Spec sanity: our ids/types/multipliers equal python-canfix's canfix.json
OURS = {0x180: ("INT", .01), 0x181: ("INT", .01), 0x182: ("INT", .01), 0x183: ("UINT", .1), 0x184: ("DINT", 1),
        0x185: ("UINT", .1), 0x186: ("INT", 1), 0x18A: ("INT", .001), 0x18B: ("INT", .001), 0x18D: ("UINT", .1),
        0x191: ("DINT", 1), 0x406: ("INT", .01), 0x407: ("INT", .01)}
for pid, (t, m) in OURS.items():
    d = canfix.protocol.parameters[pid]
    if d.type != t or (d.multiplier or 1.0) != m: fail(f"table {pid:#x}: {d.type} {d.multiplier} vs {t} {m}")
print("parameter table vs python-canfix canfix.json:", "OK" if not fails else "MISMATCH")
print("  AOA meta in python-canfix:", canfix.protocol.parameters[0x182].metadata)

def mult(pid): return canfix.protocol.parameters[pid].multiplier or 1.0

# ---- 1. C++ encodes -> python-canfix decodes
cases = []
for pid, (lo, hi) in P.items():
    for k in range(301):
        cases.append((pid, lo + (hi - lo) * k / 300, 0, 0))
    for _ in range(300):
        cases.append((pid, random.uniform(lo, hi), random.choice([0, 1, 2, 3]), 0))
    cases += [(pid, lo - 50, 0, 0), (pid, hi + 50, 0, 0), (pid, float("nan"), 0, 0)]
    if pid == 0x185: cases += [(pid, 360.0, 0, 0), (pid, -10.0, 0, 0), (pid, 725.5, 0, 0), (pid, 359.97, 0, 0)]
for meta in (1, 2, 7, 8, 9):
    cases.append((0x182, random.uniform(-10, 25), 0, meta))
cases = [(pid, f32(v), fl, me) for pid, v, fl, me in cases]
inp = os.path.join(work, "cpp_enc_in.txt"); out = os.path.join(work, "cpp_enc_out.txt")
with open(inp, "w") as f:
    for pid, v, fl, me in cases: f.write(f"{pid:x} {v!r} {fl} {me}\n")
subprocess.check_call([exe, "encode", inp, out])
n1 = 0
for (pid, v, fl, me), line in zip(cases, open(out)):
    cid, dlc, hx = line.split()
    data = bytes.fromhex(hx)
    msg = can.Message(arbitration_id=int(cid, 16), data=data, dlc=int(dlc), is_extended_id=False)
    p = canfix.parseMessage(msg)
    lo, hi = P[pid]
    exp_fail = math.isnan(v)
    if pid == 0x185 and not exp_fail:
        exp = v % 360
        if round(exp / mult(pid)) * mult(pid) >= 360: exp = 0
    else:
        exp = 0 if exp_fail else min(max(v, lo), hi)
    exp_q = (not exp_fail) and pid != 0x185 and (v < f32(lo) or v > f32(hi))
    if not isinstance(p, canfix.Parameter) or p.identifier != pid or p.node != 0x50:
        fail(f"py parse {line.strip()}"); continue
    if abs(p.value - exp) > mult(pid) / 2 + 1e-6: fail(f"value {pid:#x} v={v} py={p.value}")
    if p.failure != exp_fail: fail(f"failure flag {pid:#x} v={v}")
    if bool(p.quality) != (exp_q or bool(fl & 2)): fail(f"quality flag {pid:#x} v={v} fl={fl}")
    if bool(p.annunciate) != bool(fl & 1): fail(f"annunc flag {pid:#x}")
    if me and p.meta != canfix.protocol.parameters[pid].metadata[me]: fail(f"meta {me} -> {p.meta}")
    # byte-exact against python-canfix's own encoder for in-range values
    if not exp_fail and not exp_q:
        q = canfix.Parameter(); q.identifier = pid; q.node = 0x50; q.value = exp if pid == 0x185 else v
        q.function = (fl & 3) | (me << 4)
        if bytes(q.msg.data) != data: fail(f"bytes {pid:#x} v={v}: py {bytes(q.msg.data).hex()} cpp {hx}")
    n1 += 1
print(f"C++ encode -> python-canfix decode: {n1} frames checked")

# ---- 2. python-canfix encodes -> C++ decodes
frames, exp = [], []
for pid, (lo, hi) in P.items():
    vals = [lo, hi, 0] + [random.uniform(lo, hi) for _ in range(500)]
    for v in vals:
        q = canfix.Parameter(); q.identifier = pid; q.node = random.randint(1, 255)
        q.value = v
        q.function = random.choice([0, 1, 2, 4, 3])
        if pid == 0x182 and random.random() < .1: q.meta = random.choice([1, 2, 7, 8, 9])
        m = q.msg
        frames.append(m); exp.append((pid, q.node, q.function, round(v / mult(pid)) * mult(pid)))
inp = os.path.join(work, "py_frames.txt"); out = os.path.join(work, "cpp_dec_out.txt")
with open(inp, "w") as f:
    for m in frames: f.write(f"{m.arbitration_id:x} {m.dlc} {bytes(m.data).hex()}\n")
subprocess.check_call([exe, "decode", inp, out])
n2 = 0
for (pid, node, fn, v), line in zip(exp, open(out)):
    s = line.split()
    if s[1] == "ERR": fail(f"C++ rejected {pid:#x}"); continue
    if int(s[0], 16) != pid or int(s[1]) != node or int(s[3]) != fn: fail(f"hdr {line.strip()}")
    if abs(float(s[4]) - v) > mult(pid) * 1e-3 + abs(v) * 1e-7: fail(f"value {pid:#x} exp={v} cpp={s[4]}")
    n2 += 1
print(f"python-canfix encode -> C++ decode: {n2} frames checked")

# ---- 3. Node identification / status frames parse in python-canfix
ns = canfix.parseMessage(can.Message(arbitration_id=0x6E0 + 0x50, data=bytes([6, 0, 0, 0, 0]), is_extended_id=False))
if not isinstance(ns, canfix.NodeStatus) or ns.sendNode != 0x50 or ns.value != [False] * 16: fail("node status")
ni = canfix.parseMessage(can.Message(arbitration_id=0x6E0 + 0x50, data=bytes([0, 145, 1, 0x50, 1, 0, 0x53, 0x4F]), is_extended_id=False))
if not isinstance(ni, canfix.NodeIdentification) or ni.device != 0x50 or ni.model != 0x4F5300: fail("node id")
print("node status / node identification frames: python-canfix parses OK")
print("CROSSCHECK", "PASS" if fails == 0 else f"FAIL ({fails})")
sys.exit(1 if fails else 0)
