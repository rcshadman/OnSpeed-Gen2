#!/usr/bin/env python3
"""Check the virtual ESP32-S3 run against the original OnSpeed Gen2 rules and build the
demo trace. Usage: check_run.py run.log out_dir"""
import base64, json, math, sys, os
import numpy as np

log, out = sys.argv[1], sys.argv[2]
D, S, G, P, PH, W, M, CAN, A = [], [], [], [], [], [], "", [], []
for line in open(log, errors="replace"):
    line = line.rstrip("\r\n")
    if line.startswith("D,"):
        f = line.split(",")[1:]
        D.append(dict(t=int(f[0]), aoaTrue=float(f[1]), aoa=float(f[2]), ias=float(f[3]), iasTrue=float(f[4]),
                      palt=float(f[5]), mode=int(f[6]), high=int(f[7]), pps=float(f[8]), flaps=int(f[9]),
                      pitch=float(f[10]), bank=float(f[11]), latG=float(f[12]), vertG=float(f[13]),
                      pl=int(f[14]), cp=float(f[15]), fault=int(f[16]) if len(f) > 16 else 0))
    elif line.startswith("S,"): S.append(line[2:])
    elif line.startswith("G,"): G.append(line[2:])
    elif line.startswith("P,"):
        f = line.split(",")[1:]; P.append(dict(s=int(f[0]), f=int(f[1]), m=int(f[2]), v=float(f[3])))
    elif line.startswith("PH,"):
        _, t, txt = line.split(",", 2); PH.append(dict(t=int(t), text=txt))
    elif line.startswith("W,"): W.append(line[2:])
    elif line.startswith("M,"): M = line
    elif line.startswith("C,"): CAN.append(line[2:])
    elif line.startswith("A,"):
        f = line.split(",")[1:]
        A.append(dict(t=int(f[0]), p=float(f[1]), r=float(f[2]), pT=float(f[3]), rT=float(f[4]), vsi=float(f[5]), vsiT=float(f[6])))

checks = []
def check(name, ok, detail):
    checks.append(dict(name=name, ok=bool(ok), detail=detail))
    print(("PASS " if ok else "FAIL ") + name + ": " + detail)

print(f"{len(D)} sensor cycles, {len(S)} display sentences, {len(G)} G5 sentences, {len(W)} audio blocks")

# Gen2 setpoints (default_config.h, RV-4) per flap index
SP = {0: (8.03, 11.25, 13.84, 16.48), 20: (5.73, 9.74, 12.64, 16.29), 40: (4.78, 8.62, 12.44, 14.51)}
MUTE = 25

def gen2_tone(aoa, ias, flaps):
    """Tones.ino updateTones(), written independently from the C++ port."""
    ld, fast, slow, stall = SP[flaps]
    if ias <= MUTE: return 3, 20.0
    if aoa >= stall: return 1, 20.0
    if aoa > slow: return 1, (aoa - slow) * (6.2 - 1.5) / (stall - slow) + 1.5
    if aoa >= fast: return 2, 8.2
    if aoa >= ld and ld < fast: return 1, (aoa - ld) * (8.2 - 1.5) / (fast - ld) + 1.5
    return 3, 20.0

# 1. Tone decision matches the Gen2 rules on every cycle
def on_edge(d):  # the log keeps 4 decimals; a value this close to a setpoint can't be judged from it
    return any(abs(d["aoa"] - v) < 0.0006 for v in SP[d["flaps"]]) or abs(d["ias"] - MUTE) < 0.006
bad = [d for d in D if not on_edge(d) and not d["fault"] and (lambda m, p: m != d["mode"] or abs(p - d["pps"]) > 0.02)(*gen2_tone(d["aoa"], d["ias"], d["flaps"]))]
edge = sum(on_edge(d) for d in D)
nf = sum(d["fault"] for d in D)
check("Tone choice follows Gen2 rules", not bad, f"{len(D) - len(bad) - edge - nf}/{len(D) - edge - nf} cycles match updateTones() ({edge} cycles sit exactly on a setpoint, {nf} in the injected sensor fault)")

# 2. AOA tracks the true AOA once the Gen2 filters settle (steady parts of the flight)
flying = [d for d in D if d["iasTrue"] > 40]
steady = []
for i in range(50, len(D)):
    win = D[i - 50:i + 1]
    if D[i]["iasTrue"] > 40 and max(w["aoaTrue"] for w in win) - min(w["aoaTrue"] for w in win) < 0.05:
        steady.append(D[i])
err = [abs(d["aoa"] - d["aoaTrue"]) for d in steady]
check("AOA matches the probe's true AOA (steady flight)", err and max(err) < 0.3,
      f"max error {max(err):.2f} deg over {len(steady)} steady cycles")
# Same AOA as an independent Python re-implementation of the Gen2 pipeline fed the same counts
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen2_reference import reference_aoa
ref = reference_aoa(D)
re_ = [abs(a - d["aoa"]) for a, d in zip(ref, D) if d["iasTrue"] > 40]
check("AOA matches Gen2 math cycle by cycle (incl. manoeuvres)", max(re_) < 0.1,
      f"max difference {max(re_):.3f} deg, mean {np.mean(re_):.4f} deg over {len(re_)} cycles vs an independent Gen2 re-implementation")
idx = [i for i in range(60, len(D)) if D[i]["iasTrue"] > 40 and D[i - 60]["iasTrue"] > 40]
best = min(range(0, 40), key=lambda k: float(np.mean([abs(D[i]["aoa"] - D[i - k]["aoaTrue"]) for i in idx])))
print(f"info: board AOA trails true AOA by {best*0.02:.2f} s, the Gen2 filter delay")

# 3. Spike rejection: +400 count spike every 97 samples must not show up as AOA jumps
jumps = [abs(D[i]["aoa"] - D[i - 1]["aoa"]) for i in range(1, len(D)) if D[i]["iasTrue"] > 40 and D[i - 1]["iasTrue"] > 40]
check("Pressure spikes rejected by median filter", max(jumps) < 0.35,
      f"largest AOA step between 20 ms cycles {max(jumps):.2f} deg with {len(D)//97} injected spikes")

# 4. IAS from the pitot pressure
ie = [abs(D[i]["ias"] - D[i - 12]["iasTrue"]) for i in range(12, len(D)) if D[i]["iasTrue"] > 40]
check("Airspeed from pressure sensor", max(ie) < 3, f"max IAS error {max(ie):.1f} kt above 40 kt (allowing the 0.24 s pressure filter delay)")

# 5. Display sentence: parse like the M5Stack display (SerialRead.ino)
def m5parse(s):
    s = s + "\r\n"
    if len(s) != 80 or s[:2] != "#1": return None
    crc = sum(ord(c) for c in s[:76]) & 0xFF
    if crc != int(s[76:78], 16): return None
    g = lambda a, b: float(s[a:b])
    return dict(pitch=g(2, 6) / 10, roll=g(6, 11) / 10, ias=g(11, 15) / 10, palt=g(15, 21), turn=g(21, 26) / 10,
                latG=g(26, 29) / 100, vertG=g(29, 32) / 10, pl=int(s[32:34]), aoa=g(34, 38) / 10, vsi=g(38, 42) * 10,
                oat=int(s[42:45]), fpa=g(45, 49) / 10, flaps=int(s[49:52]), stall=g(52, 56) / 10, slow=g(56, 60) / 10,
                fast=g(60, 64) / 10, tonesOn=g(64, 68) / 10, gOnset=g(68, 72) / 100, spin=int(s[72:74]), mark=int(s[74:76]))
parsed = [m5parse(s) for s in S]
okn = sum(p is not None for p in parsed)
check("Display sentences decode on the M5Stack parser", okn == len(S), f"{okn}/{len(S)} sentences: 80 bytes, checksum valid")

# 6. G5 sentences: checksum and parse round trip
def g5parse(s):
    s = s + "\r\n"
    if len(s) != 59 or s[:3] != "=11": return None
    if (sum(ord(c) for c in s[:55]) & 0xFF) != int(s[55:57], 16): return None
    return dict(pitch=float(s[11:15]) / 10, roll=float(s[15:20]) / 10, hdg=int(s[20:23]), ias=float(s[23:27]) / 10, palt=int(s[27:33]))
g5ok = [g5parse(s) for s in G]
mm = dict(x.split("=") for x in M[2:].split(",")) if M else {}
check("G5 EFIS sentences received", all(g5ok) and mm.get("g5_good") == str(len(G)) and mm.get("g5_bad") == "0",
      f"{mm.get('g5_good')} of {len(G)} G5 frames parsed on the board, {mm.get('g5_bad')} CRC failures")
att = [abs(p["pitch"] - D[i * 5]["pitch"]) for i, p in enumerate(parsed) if p and i * 5 < len(D)]
check("Display attitude comes from the G5", max(att) < 0.15, f"max pitch difference G5 vs display {max(att):.2f} deg")

# 7. Audio analysis of the PCM the board produced (11025 Hz stereo copy)
pcm = np.frombuffer(b"".join(base64.b64decode(w) for w in W), dtype=np.uint8).astype(np.float32) - 128
L, R = pcm[0::2], pcm[1::2]
fs = 11025
mono = (np.abs(L) + np.abs(R))
blk = fs // 50  # samples per 20 ms cycle
def seg(t0, t1):
    a, b = int(t0 * fs), int(t1 * fs); return L[a:b], R[a:b]
def dom_freq(x):
    if len(x) < 512 or np.max(np.abs(x)) < 2: return 0
    sp = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    return np.fft.rfftfreq(len(x), 1 / fs)[np.argmax(sp)]
def pulses(x, thr=4):
    env = np.convolve(np.abs(x), np.ones(40) / 40, "same") > thr
    edges = np.flatnonzero(np.diff(env.astype(int)) == 1)
    return edges
def window_report(t0, t1):
    l, r = seg(t0, t1)
    e = pulses(l + r)
    rate = (len(e) - 1) / ((e[-1] - e[0]) / fs) if len(e) > 2 else 0
    on = float(np.mean(np.convolve(np.abs(l + r), np.ones(40) / 40, "same") > 4))
    return dom_freq(l + r), rate, on, float(np.sqrt(np.mean(l ** 2)) + 1e-9), float(np.sqrt(np.mean(r ** 2)) + 1e-9)

def tone_at(t):
    d = D[int(t * 50)]; return d

segs = [("Cruise, AOA 1.5: silent", 28, 33, "off"),
        ("L/Dmax, AOA ~9.5: low-tone pulses", 50.5, 53.5, "low"),
        ("OnSpeed, AOA 12.5: solid low tone (incl. a 0.2 s sensor dropout)", 57.5, 61.4, "solid"),
        ("Slow, AOA ~15: high-tone pulses", 68.2, 69.5, "high"),
        ("Stall warning, AOA 17.5: 20 pulses/s", 73.5, 75.8, "stall"),
        ("Taxi under 25 kt: muted", 0.5, 3.5, "off")]
audio_rows = []
for name, t0, t1, kind in segs:
    f, rate, on, rl, rr = window_report(t0, t1)
    exp_pps = np.mean([d["pps"] for d in D[int(t0 * 50):int(t1 * 50)]])
    if kind == "off": ok = on < 0.01; det = f"sound on {on*100:.1f}% of the time"
    elif kind == "solid": ok = on > 0.98 and abs(f - 400) < 15; det = f"{f:.0f} Hz, sound on {on*100:.0f}% of the time"
    else:
        ef = 400 if kind == "low" else 1600
        ok = abs(f - ef) < 20 and abs(rate - exp_pps) < 0.15 * exp_pps + 0.3
        det = f"{f:.0f} Hz, measured {rate:.1f} pulses/s vs {exp_pps:.1f} commanded"
    check("Audio: " + name, ok, det)
    audio_rows.append(dict(name=name, freq=round(f), rate=round(rate, 2), on=round(on, 3)))

# 3D audio: ball right (latG +0.05) around t=101.5..103, left around t=105..106
def latwin(sign):
    ts = [d["t"] / 1000 for d in D if d["latG"] * sign >= 0.049]
    return ts[0] + 0.5, ts[-1]
_, _, _, rl1, rr1 = window_report(*latwin(1))
_, _, _, rl2, rr2 = window_report(*latwin(-1))
check("3D audio pans with the slip ball", rr1 > 1.3 * rl1 and rl2 > 1.3 * rr2,
      f"ball right: R/L level {rr1/rl1:.2f}, ball left: L/R level {rl2/rr2:.2f}")

# flaps 20 setpoints used on approach
ap = [d for d in D if 121 <= d["t"] / 1000 <= 127]
check("Flap position selects flaps-20 curve and setpoints", all(d["flaps"] == 20 for d in ap) and all(d["mode"] == 2 for d in ap),
      f"flaps {set(d['flaps'] for d in ap)} and solid OnSpeed tone throughout the approach")

# Sensor faults injected by the simulator (sensors.cpp): 58.0-58.2 s Pfwd I2C failures, 61.0-61.8 s P45 diagnostic
ride = [d for d in D if 57.9 <= d["t"] / 1000 <= 58.5]
check("Short sensor dropout rides through", all(not d["fault"] and d["mode"] == 2 for d in ride) and mm.get("pfwd_err") == "10",
      f"{mm.get('pfwd_err')} failed Pfwd reads, no fault raised, solid tone held; {mm.get('pfwd_stale')} stale-status reads accepted")
ft = [d["t"] / 1000 for d in D if d["fault"]]
rec = [d for d in D if 62.6 <= d["t"] / 1000 <= 63.0]
okf = ft and 61.45 <= ft[0] <= 61.55 and ft[-1] < 61.85 and all(d["mode"] == 3 for d in D if d["fault"]) and all(d["mode"] == 2 for d in rec)
check("Failed sensor mutes the tones, then recovers", okf,
      f"P45 diagnostic status from 61.0 s: fault raised at {ft[0] if ft else 0:.2f} s, tones off while faulted, cleared at {ft[-1] + 0.02 if ft else 0:.2f} s, OnSpeed tone back by 62.6 s")

# CAN-FiX to the MakerPlane EFIS, decoded with MakerPlane's own python-canfix library
try:
    import can, canfix
    aoaErr, sent, efisOk, status = [], 0, 0, 0
    for i, line in enumerate(CAN):
        frames, efis = line.split("|")
        for fr in frames.split():
            cid, data = fr.split("#")
            msg = can.Message(arbitration_id=int(cid, 16), data=bytes.fromhex(data), is_extended_id=False)
            pm = canfix.parseMessage(msg)
            if isinstance(pm, canfix.Parameter) and pm.node == 0x50 and pm.identifier == 0x182 and not pm.meta:
                aoaErr.append(abs(pm.value - D[i * 5]["aoa"])) if not D[i * 5]["fault"] else None
            if int(cid, 16) == 0x6E0 + 0x50: status += 1
            sent += 1
        p_, r_, fresh = efis.split(",")
        efisOk += abs(float(p_) - D[i * 5]["pitch"]) < 0.02 and fresh == "1"
    check("MakerPlane EFIS (CAN-FiX): AOA out", aoaErr and max(aoaErr) <= 0.006 and status >= len(CAN) // 10 - 1,
          f"{len(aoaErr)} AOA frames decoded by python-canfix, max error {max(aoaErr):.3f} deg; {status} node status frames (1 Hz)")
    check("MakerPlane EFIS (CAN-FiX): attitude in", efisOk == len(CAN),
          f"{efisOk}/{len(CAN)} cycles: the board's CAN-FiX decoder holds the EFIS pitch/roll the simulator sent")
except ImportError:
    print("info: python-canfix not installed, CAN-FiX checks skipped")

# On-board AHRS (Gen2 Madgwick/Kalman port) fed by the simulated ISM330DHCX
if A:
    # airborne: rotation at 10 s to the flare at 148 s (the takeoff roll's 0.65 g acceleration is reported separately)
    up = lambda i: 10000 <= D[i]["t"] <= 148000
    air = [a for i, a in enumerate(A) if up(i)]
    def steady(i, key, tol):  # truth unchanged for the last 3 s
        w = A[max(0, i - 150):i + 1]
        return len(w) > 140 and max(x[key] for x in w) - min(x[key] for x in w) < tol
    spd = lambda i: max(D[j]["iasTrue"] for j in range(max(0, i - 150), i + 1)) - min(D[j]["iasTrue"] for j in range(max(0, i - 150), i + 1)) < 1
    wl = [a for i, a in enumerate(A) if up(i) and abs(a["rT"]) < 0.5 and steady(i, "pT", 0.5) and steady(i, "rT", 0.5)]
    lvl = [a for i, a in enumerate(A) if up(i) and abs(a["rT"]) < 0.5 and steady(i, "pT", 0.5) and steady(i, "rT", 0.5) and spd(i)]
    trn = [a for i, a in enumerate(A) if abs(a["rT"] - 45) < 0.1 and steady(i, "pT", 0.5) and steady(i, "rT", 0.1)]
    e = lambda xs, k: max(abs(x[k] - x[k + "T"]) for x in xs)
    check("On-board AHRS, straight flight", lvl and e(lvl, "p") < 1.5 and e(lvl, "r") < 1.5,
          f"max pitch error {e(lvl, 'p'):.2f} deg, roll {e(lvl, 'r'):.2f} deg over {len(lvl)} cycles at constant speed (incl. the IMU's 10 mg offset, about 0.6 deg)")
    print(f"info: AHRS wings level while speeding up or slowing down: max pitch error {e(wl, 'p'):.2f} deg (Gen2 compensates dTAS/dt from smoothed, lagging IAS)")
    check("On-board AHRS, steady 45 deg turn", trn and e(trn, "p") < 1.5 and e(trn, "r") < 1.5,
          f"max pitch error {e(trn, 'p'):.2f} deg, roll {e(trn, 'r'):.2f} deg over {len(trn)} cycles")
    # the flight model stops altitude at its 1000 ft floor while still reporting the descent rate; skip those cycles
    vs = [a for i, a in enumerate(A) if up(i) and D[i]["palt"] > 1015 and steady(i, "vsiT", 20)]
    check("On-board AHRS, vertical speed (Kalman)", vs and max(abs(a["vsi"] - a["vsiT"]) for a in vs) < 150,
          f"max VSI error {max(abs(a['vsi'] - a['vsiT']) for a in vs):.0f} fpm over {len(vs)} steady cycles")
    wp = max(air, key=lambda a: abs(a["p"] - a["pT"])); wr = max(air, key=lambda a: abs(a["r"] - a["rT"]))
    roll_ = [a for i, a in enumerate(A) if 4000 <= D[i]["t"] < 10000]
    print(f"info: AHRS pitch error during the takeoff roll (0.65 g along the runway): max {max(abs(a['p'] - a['pT']) for a in roll_):.2f} deg")
    print(f"info: worst AHRS error in flight incl. manoeuvres: pitch {abs(wp['p'] - wp['pT']):.2f} deg at {wp['t'] / 1000:.1f} s, roll {abs(wr['r'] - wr['rT']):.2f} deg at {wr['t'] / 1000:.1f} s")

print(M)
os.makedirs(out, exist_ok=True)
# compact trace for the demo page: every cycle (50 Hz) and display sentences (10 Hz)
trace = dict(
    dt=0.02, phases=PH,
    cols=["aoaTrue", "aoa", "ias", "pitch", "bank", "latG", "vertG", "palt", "mode", "high", "pps", "flaps", "pl"],
    rows=[[round(d["aoaTrue"], 2), round(d["aoa"], 2), round(d["ias"], 1), round(d["pitch"], 2), round(d["bank"], 1),
           round(d["latG"], 3), round(d["vertG"], 2), round(d["palt"]), d["mode"], d["high"], round(d["pps"], 2), d["flaps"], d["pl"]] for d in D],
    ahrs=[[round(a["p"], 2), round(a["r"], 2)] for a in A],
    display=S, g5=G[::1], can=[c.split("|")[0].strip() for c in CAN], checks=checks, metrics=M, audioRate=fs)
json.dump(trace, open(os.path.join(out, "trace.json"), "w"), separators=(",", ":"))
# board audio as 8-bit stereo WAV
import wave
wf = wave.open(os.path.join(out, "board_audio.wav"), "wb")
wf.setnchannels(2); wf.setsampwidth(1); wf.setframerate(fs)
wf.writeframes((pcm + 128).astype(np.uint8).tobytes()); wf.close()
print("passed", sum(c["ok"] for c in checks), "of", len(checks))
