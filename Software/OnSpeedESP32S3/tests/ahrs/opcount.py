#!/usr/bin/env python3
"""Static instruction mix of the ESP32-S3 build of ahrs.cpp (objdump -d output).

Counts every instruction per function and splits them into FPU arithmetic, FPU moves/loads,
integer/other and calls. processAHRS() is mostly straight-line code, so its static count is a
fair proxy for the dynamic count of the default (no OAT, no aoaCompensation) path; the OAT and
AOA branches add a few dozen instructions that are skipped at run time.
"""
import re
import sys
from collections import Counter, defaultdict

FP_ARITH = {"add.s", "sub.s", "mul.s", "madd.s", "msub.s", "maddn.s", "neg.s", "abs.s", "div0.s", "divn.s",
            "recip0.s", "rsqrt0.s", "sqrt0.s", "nexp01.s", "addexp.s", "addexpm.s", "mkdadj.s", "mksadj.s"}
FP_CMP = {"oeq.s", "olt.s", "ole.s", "ueq.s", "ult.s", "ule.s", "un.s", "movf.s", "movt.s", "moveqz.s", "movnez.s"}
FP_MOVE = {"wfr", "rfr", "mov.s", "const.s", "float.s", "ufloat.s", "trunc.s", "utrunc.s", "round.s", "floor.s", "ceil.s"}
FP_MEM = {"lsi", "ssi", "lsx", "ssx", "lsip", "ssip", "lsxp", "ssxp"}


def main(path):
    funcs = defaultdict(Counter)
    cur = None
    for line in open(path):
        m = re.match(r"^[0-9a-f]+ <(.+)>:$", line)
        if m:
            cur = m.group(1)
            continue
        m = re.match(r"^\s+[0-9a-f]+:\s+[0-9a-f]+\s+(\S+)", line)
        if m and cur and not cur.startswith(".literal"):
            funcs[cur][m.group(1)] += 1
    hot = ["processAHRS", "updateIMU", "computeAngles", "KalmanFilter::Update", "invSqrt", "Sma30::add"]
    print(f"{'function':58s} {'insns':>6s} {'fp-arith':>8s} {'fp-cmp':>6s} {'fp-mov':>6s} {'fp-mem':>6s} {'calls':>5s}")
    tot = Counter()
    for f, c in funcs.items():
        n = sum(c.values())
        fa = sum(v for k, v in c.items() if k in FP_ARITH)
        fc = sum(v for k, v in c.items() if k in FP_CMP)
        fm = sum(v for k, v in c.items() if k in FP_MOVE)
        fl = sum(v for k, v in c.items() if k in FP_MEM)
        calls = sum(v for k, v in c.items() if k.startswith("call"))
        mark = "*" if any(h in f for h in hot) else " "
        print(f"{mark}{f[:57]:57s} {n:6d} {fa:8d} {fc:6d} {fm:6d} {fl:6d} {calls:5d}")
        if mark == "*":
            tot.update({"insns": n, "fpa": fa, "fpc": fc, "fpm": fm, "fpl": fl, "calls": calls})
            madd = c["madd.s"] + c["msub.s"]
            tot["madd"] += madd
    print(f"\n* per-sample path total: {tot['insns']} instructions, {tot['fpa']} FPU arithmetic "
          f"({tot['madd']} fused madd/msub), {tot['fpl']} FPU loads/stores, {tot['fpm']} FPU moves, "
          f"{tot['calls']} calls (__divsf3, atan2f, asinf, ...)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "ahrs_s3.dis")
