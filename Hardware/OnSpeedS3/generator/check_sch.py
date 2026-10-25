"""Electrical checks, since kicad-cli 7 has no ERC: (1) the drawn schematic's netlist, as KiCad reads it,
must equal design.py; (2) rule checks on the design data (single-pin nets, driver conflicts,
unconnected inputs, power nets without a source)."""
import sys, xml.etree.ElementTree as ET
from collections import defaultdict
import design as D

root = ET.parse(sys.argv[1] if len(sys.argv) > 1 else "out/onspeed.xml").getroot()
kn = {}
for net in root.iter("net"):
    for node in net.iter("node"):
        kn[(node.get("ref"), node.get("pin"))] = net.get("name").lstrip("/")
errors, warns = [], []
pin_types = {}
for p in D.parts:
    for pin in p.pin_list():
        pin_types[(p.ref, pin["number"])] = (pin["type"], pin["name"])
for p in D.parts:
    for pin, n in p.nets.items():
        if (p.ref, pin) not in pin_types or p.ref.startswith("#"):
            continue  # footprint-only pad
        k = kn.get((p.ref, pin))
        if n is None:
            if k and not k.startswith("unconnected-"):
                errors.append(f"{p.ref}.{pin} should be NC, KiCad has {k}")
        elif k != n:
            errors.append(f"{p.ref}.{pin}: design {n}, KiCad {k}")
nets = defaultdict(list)
for p in D.parts:
    for pin, n in p.nets.items():
        if n:
            nets[n].append((p.ref, pin))
drivers = {"output", "power_out"}
for n, nodes in sorted(nets.items()):
    if len(nodes) < 2:
        errors.append(f"net {n} has one connection: {nodes}")
    types = [pin_types.get(x, ("passive", ""))[0] for x in nodes]
    outs = [x for x, t in zip(nodes, types) if t == "output"]
    if len(outs) > 1:
        errors.append(f"net {n}: several outputs {outs}")
    if "input" in types and not any(t in ("output", "bidirectional", "passive", "power_out", "open_collector") for t in types):
        errors.append(f"net {n}: input with no driver")
    if "power_in" in types and not any(t in ("power_out",) for t in types) and not any(x[0].startswith("#FLG") for x in nodes):
        warns.append(f"net {n}: power input without power_out or PWR_FLAG")
print(f"{len(D.parts)} parts, {len(nets)} nets, {len(kn)} pins in KiCad netlist")
for e in errors: print("ERROR", e)
for w in warns: print("WARN ", w)
sys.exit(1 if errors else 0)
