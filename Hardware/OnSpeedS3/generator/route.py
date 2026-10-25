"""Net classes, planes, autoroute with Freerouting, import the result.

Stack-up (JLC 4-layer, 1.6 mm, JLC04161H-7628): F.Cu signals + parts, In1 solid GND, In2 +3V3 plane,
B.Cu signals. Top and bottom get GND pours stitched to In1 after routing (finish.py).
"""
import os, subprocess, sys, pcbnew

MM = pcbnew.FromMM
src, dsn, ses, out = "out/onspeed_placed.kicad_pcb", "out/onspeed.dsn", "out/onspeed.ses", "out/onspeed_routed.kicad_pcb"
# second stage (ROUTE_GND=1): start from the fanned-out board and let the router finish the last ground pins
GND_STAGE = os.environ.get("ROUTE_GND") == "1"
if GND_STAGE:
    src, dsn, ses = "out/onspeed_fanout.kicad_pcb", "out/onspeed_gnd.dsn", "out/onspeed_gnd.ses"
b = pcbnew.LoadBoard(src)
ds = b.GetDesignSettings()

CLASSES = {
    # name: (track, clearance, via dia, via drill, nets)
    # +3V3_S and +5V_S (sensor rails, a few mA) stay in Default so they can escape the 0.5 mm-pitch IMU pads
    "Power": (0.5, 0.2, 0.8, 0.4, ["VIN_RAW", "VIN_F", "VIN_P", "VIN", "+5V", "BUCK_SW", "VBUS", "3V3_OUT_F", "X_3V3"]),
    "Rail": (0.3, 0.2, 0.6, 0.3, ["+3V3_A", "+3V3", "DAC_VNEG", "DAC_LDOO", "DAC_CAPP", "DAC_CAPM",
                                  "RS_VP", "RS_VN", "RS_C1P", "RS_C1N", "RS_C2P", "RS_C2N", "AUDIO_GND"]),
}
ns = ds.m_NetSettings
dflt = ns.m_DefaultNetClass
dflt.SetTrackWidth(MM(0.2)); dflt.SetClearance(MM(0.2)); dflt.SetViaDiameter(MM(0.6)); dflt.SetViaDrill(MM(0.3))
for name, (tw, cl, vd, vdr, nets) in CLASSES.items():
    nc = pcbnew.NETCLASS(name)
    nc.SetTrackWidth(MM(tw)); nc.SetClearance(MM(cl)); nc.SetViaDiameter(MM(vd)); nc.SetViaDrill(MM(vdr))
    ns.m_NetClasses[name] = nc
    for n in nets:
        ni = b.FindNet(n)
        if ni:
            ni.SetNetClass(nc)


def plane(net, layer, prio=0):
    z = pcbnew.ZONE(b)
    z.SetLayer(layer)
    z.SetNet(b.FindNet(net))
    z.SetLocalClearance(MM(0.3))
    z.SetMinThickness(MM(0.2))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(MM(0.3)); z.SetThermalReliefSpokeWidth(MM(0.4))
    z.SetAssignedPriority(prio)
    o = z.Outline(); o.NewOutline()
    for x, y in ((0.4, 0.4), (99.6, 0.4), (99.6, 79.6), (0.4, 79.6)):
        o.Append(MM(x), MM(y))
    b.Add(z)
    return z


if not any(z.GetLayer() == pcbnew.In1_Cu and not z.GetIsRuleArea() for z in b.Zones()):
    plane("GND", pcbnew.In1_Cu)
b.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_POWER)   # solid GND plane: the router must not run tracks on it
# In2 is a routing layer; finish.py pours +3V3 over the space the tracks leave.
filler = pcbnew.ZONE_FILLER(b)
filler.Fill(b.Zones())
if not GND_STAGE:
    pcbnew.SaveBoard(src.replace("placed", "planes"), b)
if "--prepare-only" in sys.argv:
    sys.exit(0)   # build_board.sh fans out the ground pins before routing
assert pcbnew.ExportSpecctraDSN(b, dsn), "DSN export failed"
# KiCad 7's Python API cannot assign nets to classes by pattern, so move the nets in the DSN itself.
import re
t = open(dsn).read()
m = re.search(r'\(class kicad_default "" (.*?)\n\s*\(circuit', t, re.S)
names = m.group(1).split()
moved = {n for c in CLASSES.values() for n in c[4]}
keep = [n for n in names if n not in moved]
t = t[:m.start(1)] + " ".join(keep) + t[m.end(1):]
for name, c in CLASSES.items():
    t = re.sub(rf'\(class {name}\n', f'(class {name} ' + " ".join(n for n in c[4] if n in names) + '\n', t)
open(dsn, "w").write(t)
print("DSN written")
# GND is not routed: In1 is the ground plane and finish.py drops a via beside every ground pad.
i = t.index("    (net GND\n") if not GND_STAGE else None; d = 0; j = i
while i is not None:
    d += {"(": 1, ")": -1}.get(t[j], 0)
    if d == 0 and t[j] == ")":
        break
    j += 1
if i is not None:
    t = t[:i] + t[j + 1:]
    t = re.sub(r"(\(class \w+[^\n]*?) GND(?=[ \n])", r"\1", t)
open(dsn, "w").write(t)
# Freerouting 1.9 honours the pass limit in batch mode (2.1 ignores it); it needs a display, hence xvfb.
r = subprocess.run(["xvfb-run", "-a", "java", "-Xmx10g", "-jar", "tools/freerouting-1.9.0.jar", "-de", dsn, "-do", ses,
                    "-mp", sys.argv[1] if len(sys.argv) > 1 else "80"], capture_output=True, text=True, timeout=3600)
print(r.stdout[-1500:], r.stderr[-1500:])
assert os.path.exists(ses), "no SES"
print("SES written; run ses_import.py")
