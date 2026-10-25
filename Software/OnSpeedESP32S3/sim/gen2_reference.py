"""Independent Python re-implementation of the Gen2 sensor-to-AOA path (SensorRead.ino,
AOAcalc.ino, CurveCalc.ino with RunningMedian/RunningAverage), fed with the same simulated
sensor counts as the virtual board. Returns AOA per 50 Hz cycle."""
import numpy as np
FLAP_LAG = 9  # cycles: FlapDetector needs 10 matching reads; AOA uses the previous cycle's stage
CURVES = {0: (8.4845, 24.0804, 4.6157), 20: (4.5244, 27.9928, 2.7661), 40: (1.2984, 30.4239, 1.4550)}


def reference_aoa(D, bias=8192):
    lcg = 12345
    def noise():
        nonlocal lcg
        lcg = (lcg * 1664525 + 1013904223) & 0xFFFFFFFF
        return ((lcg >> 16) % 5) - 2
    med_f, med_p, avg_f, avg_p = [], [], [], []
    aoa, out, n = 0.0, [], 0
    last_f = last_p = 0
    # The simulator builds P45 from the true flap stage; the board switches curves FLAP_LAG cycles later
    # (FlapDetector debounce), so the true stage is the board's stage FLAP_LAG cycles ahead.
    true_flaps = [D[min(i + FLAP_LAG, len(D) - 1)]["flaps"] for i in range(len(D))]
    for i, d in enumerate(D):
        ias = d["iasTrue"]
        ind = (ias - 4.162) / 0.977 if ias > 5 else ias
        v = ind / 1.94384; q = 0.5 * 1.225 * v * v
        fwd = int(np.round(np.float32(q / 6894.757 * (0.8 * 16383 / 2))))
        n += 1
        rf = bias + fwd + noise()
        if n % 97 == 0: rf += 400
        t = d["t"] / 1000
        drop_f = 58.0 - 1e-4 <= t < 58.2 - 1e-4  # simulator's injected faults: the port holds the last good read
        drop_p = 61.0 - 1e-4 <= t < 61.8 - 1e-4
        A, B, C = CURVES[true_flaps[i]]
        disc = B * B - 4 * A * (C - d["aoaTrue"])
        cp = (-B + disc ** .5) / (2 * A)
        A, B, C = CURVES[d["flaps"]]
        rp = bias + int(np.round(cp * fwd)) + noise()
        noise()  # the static sensor read draws from the same generator
        rf = min(max(rf, 0), 16383); rp = min(max(rp, 0), 16383)
        pf = last_f if drop_f else rf - bias
        p4 = last_p if drop_p else rp - bias
        last_f, last_p = pf, p4
        med_f = (med_f + [pf])[-15:]; med_p = (med_p + [p4])[-15:]
        avg_f = (avg_f + [float(np.median(med_f))])[-10:]; avg_p = (avg_p + [float(np.median(med_p))])[-10:]
        sf, sp = np.mean(avg_f), np.mean(avg_p)
        if sf > 0:
            x = sp / sf
            aoa = (A * x * x + B * x + C) * 0.1 + 0.9 * aoa
            aoa = min(max(aoa, -20), 40)
        else:
            aoa = -20
        out.append(aoa)
    return out
