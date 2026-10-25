"""Manufacturing outputs for JLCPCB: Gerbers + drill (zipped), BOM and CPL in JLC's format, STEP, PDFs."""
import csv, os, re, shutil, subprocess, sys, zipfile
from collections import OrderedDict
import design as D

board = sys.argv[1] if len(sys.argv) > 1 else "out/onspeed_final.kicad_pcb"
OUT = "out/fab"
shutil.rmtree(OUT, ignore_errors=True)
os.makedirs(OUT + "/gerber")


def run(*a):
    r = subprocess.run(["kicad-cli", *a], capture_output=True, text=True)
    if r.returncode:
        raise SystemExit(r.stdout + r.stderr)


layers = "F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts"
run("pcb", "export", "gerbers", "--layers", layers, "--subtract-soldermask", "--no-x2", "-o", OUT + "/gerber/", board)
run("pcb", "export", "drill", "--format", "excellon", "--excellon-separate-th", "--generate-map", "--map-format", "gerberx2",
    "--excellon-units", "mm", "-o", OUT + "/gerber/", board)
with zipfile.ZipFile(OUT + "/onspeed_gerbers.zip", "w", zipfile.ZIP_DEFLATED) as z:
    for f in sorted(os.listdir(OUT + "/gerber")):
        z.write(os.path.join(OUT, "gerber", f), f)

# BOM: one line per (value, footprint, LCSC); parts with no LCSC number go through JLC Global Sourcing
rows = OrderedDict()
for p in D.parts:
    if p.ref.startswith("#") or not p.in_bom or not p.footprint:
        continue
    key = (p.value, p.footprint.split(":")[1], p.fields.get("LCSC", ""), p.fields.get("MPN", ""))
    rows.setdefault(key, []).append(p.ref)
bom_refs = {r for v in rows.values() for r in v}


def natkey(r):
    m = re.match(r"([A-Z]+)(\d+)", r)
    return (m.group(1), int(m.group(2)))


with open(OUT + "/onspeed_bom_jlc.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #", "MPN", "Qty"])
    for (val, fp, lcsc, mpn), refs in sorted(rows.items(), key=lambda kv: natkey(sorted(kv[1], key=natkey)[0])):
        w.writerow([val, ",".join(sorted(refs, key=natkey)), fp, lcsc, mpn, len(refs)])

# CPL: KiCad position file -> JLC columns, with JLC's rotation offsets for packages whose library zero
# differs from KiCad's (the order page shows the placement preview; check it before paying)
run("pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "both", "-o", OUT + "/pos.csv", board)
ROT = [(r"^SOT-223", 180), (r"^SOT-23", 180), (r"^SOIC-", 270), (r"^TSSOP-", 270), (r"^VSSOP-", 270),
       (r"^CP_Elec_", 180), (r"^SOIC-8-1EP", 270), (r"^USB_C_Receptacle_HRO", 180)]
with open(OUT + "/pos.csv") as f, open(OUT + "/onspeed_cpl_jlc.csv", "w", newline="") as g:
    w = csv.writer(g)
    w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
    for row in csv.DictReader(f):
        ref = row["Ref"]
        if ref not in bom_refs:
            continue
        rot = float(row["Rot"])
        for pat, off in ROT:
            if re.match(pat, row["Package"]):
                rot = (rot + off) % 360
                break
        w.writerow([ref, f'{float(row["PosX"]):.3f}mm', f'{float(row["PosY"]):.3f}mm',
                    "Top" if row["Side"] == "top" else "Bottom", f"{rot:g}"])
os.remove(OUT + "/pos.csv")

run("pcb", "export", "step", "--subst-models", "--force", "-o", OUT + "/onspeed_board.step", board)
run("pcb", "export", "pdf", "--layers", "F.Cu,F.SilkS,F.Fab,Edge.Cuts", "-o", OUT + "/assembly_top.pdf", board)
for L in ("F.Cu", "In1.Cu", "In2.Cu", "B.Cu"):
    run("pcb", "export", "pdf", "--layers", f"{L},Edge.Cuts", "-o", f"{OUT}/copper_{L.replace('.', '_')}.pdf", board)
print("fab outputs in", OUT, "BOM lines:", len(rows), "placed parts:", len(bom_refs))
