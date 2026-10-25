"""Build the placed (unrouted) board: out/onspeed_placed.kicad_pcb, plus a courtyard overlap report."""
import sys, pcbnew
import design as D
import place
from pcb import build
from sch import Schematic

MM, TO = pcbnew.FromMM, pcbnew.ToMM
sch_uuid = Schematic("OnSpeed ESP32-S3 main board").uuid
pos, errors, missing = place.resolve()
assert not errors and not missing, (errors, missing)
out = "out/onspeed_placed.kicad_pcb"
build(D.parts, pos, sch_uuid, out)
b = pcbnew.LoadBoard(out)  # reload: footprints created in this process are not safe to iterate


# antenna keep-out: no copper under or left of the ESP32 antenna (see footprints.esp32_body_courtyard)
esp = [f for f in b.GetFootprints() if f.GetValue().startswith("ESP32")][0]
ka = pcbnew.ZONE(b)
ka.SetIsRuleArea(True)
ka.SetDoNotAllowCopperPour(True); ka.SetDoNotAllowTracks(True); ka.SetDoNotAllowVias(True)
ka.SetDoNotAllowPads(False); ka.SetDoNotAllowFootprints(False)
ls = pcbnew.LSET()
for L in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
    ls.AddLayer(L)
ka.SetLayerSet(ls)
ka.SetZoneName("antenna keep-out")
o = ka.Outline(); o.NewOutline()
ex, ey = TO(esp.GetPosition().x), TO(esp.GetPosition().y)
for x, y in ((0, ey - 9.3), (ex - 6.75, ey - 9.3), (ex - 6.75, ey + 9.3), (0, ey + 9.3)):
    o.Append(MM(x), MM(y))
b.Add(ka)

# courtyard overlap report (front and back separately)
boxes = []
for f in b.GetFootprints():
    xs, ys = [], []
    for g in f.GraphicalItems():
        if g.GetLayer() in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
            gb = g.GetBoundingBox()
            xs += [TO(gb.GetLeft()), TO(gb.GetRight())]; ys += [TO(gb.GetTop()), TO(gb.GetBottom())]
    if not xs:
        continue
    boxes.append((f.GetReference(), f.IsFlipped(), min(xs), min(ys), max(xs), max(ys)))
bad = 0
for i, a in enumerate(boxes):
    if a[2] < 0.2 or a[4] > 99.8:
        print(f"EDGE {a[0]} x {a[2]:.2f}..{a[4]:.2f}")
    for c in boxes[i + 1:]:
        if a[1] != c[1]:
            continue
        ox = min(a[4], c[4]) - max(a[2], c[2]); oy = min(a[5], c[5]) - max(a[3], c[3])
        if ox > 0.01 and oy > 0.01:
            bad += 1
            print(f"OVERLAP {a[0]} / {c[0]}: {ox:.2f} x {oy:.2f} mm")
print("overlaps:", bad)
pcbnew.SaveBoard(out, b)
