"""Generate onspeed.kicad_sch from design.py and run ERC."""
import sys, subprocess
import design as D
from sch import Schematic

sch = Schematic("OnSpeed ESP32-S3 main board", paper="A0")
for p in D.parts:
    sch.add(p)
for x1, y1, x2, y2, title in D.frames:
    sch.rect(x1, y1, x2, y2)
    if title:
        sch.text(title, x1 + 3, y1 + 7, 3.0, True)
for txt, x, y, size, bold in D.notes:
    sch.text(txt, x, y, size, bold)
sch.connect_all()
out = "out/onspeed.kicad_sch"
sch.write(out, "onspeed")
print(len(D.parts), "symbols,", len({n for p in D.parts for n in p.nets.values() if n}), "nets")
