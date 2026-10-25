"""List outer-layer ground pour pieces that have no via or through-hole pad (they would float)."""
import sys, pcbnew
TO = pcbnew.ToMM
b = pcbnew.LoadBoard(sys.argv[1])
vias = [t for t in b.GetTracks() if t.GetClass() == "PCB_VIA" and t.GetNetname() == "GND"]
tht = [p for f in b.GetFootprints() for p in f.Pads() if p.GetNetname() == "GND" and p.GetDrillSize().x > 0]
smd = [(f.GetReference(), p) for f in b.GetFootprints() for p in f.Pads() if p.GetNetname() == "GND"]
for z in b.Zones():
    if z.GetIsRuleArea() or z.GetNetname() != "GND" or z.GetLayer() not in (pcbnew.F_Cu, pcbnew.B_Cu):
        continue
    P = z.GetFilledPolysList(z.GetLayer())
    for i in range(P.OutlineCount()):
        ch = P.Outline(i)
        if sum(ch.PointInside(v.GetPosition()) for v in vias) + sum(ch.PointInside(p.GetPosition()) for p in tht):
            continue
        bb = ch.BBox()
        pads = [f"{r}.{p.GetNumber()}@({TO(p.GetPosition().x):.2f},{TO(p.GetPosition().y):.2f})" for r, p in smd if ch.PointInside(p.GetPosition())]
        print(z.GetLayerName(), f"x {TO(bb.GetLeft()):.2f}-{TO(bb.GetRight()):.2f} y {TO(bb.GetTop()):.2f}-{TO(bb.GetBottom()):.2f}", pads)
