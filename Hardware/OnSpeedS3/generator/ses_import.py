"""Import a Freerouting .ses into the board (KiCad 7's ImportSpecctraSES only works inside the editor)."""
import re, sys, pcbnew
from symlib import parse, unq

MM = pcbnew.FromMM
src, ses, out = sys.argv[1], sys.argv[2], sys.argv[3]
b = pcbnew.LoadBoard(src)
t = parse(open(ses).read())


def find(node, key):
    return [e for e in node if isinstance(e, list) and e[0] == key]


routes = find(t, "routes")[0]
res = find(routes, "resolution")[0]
scale = {"um": 1e-3, "mil": 0.0254, "mm": 1.0}[res[1]] / float(res[2])   # file units -> mm
layers = {b.GetLayerName(i): i for i in range(pcbnew.PCB_LAYER_ID_COUNT)}
for tr in list(b.GetTracks()):
    b.Remove(tr)
nt = nv = 0
for net in find(find(routes, "network_out")[0], "net"):
    ni = b.FindNet(unq(net[1]))
    for w in find(net, "wire"):
        for p in find(w, "path"):
            layer, width = layers[unq(p[1])], float(p[2]) * scale
            pts = [float(v) * scale for v in p[3:] if not isinstance(v, list)]
            xy = [(pts[i], -pts[i + 1]) for i in range(0, len(pts), 2)]
            for (x1, y1), (x2, y2) in zip(xy, xy[1:]):
                s = pcbnew.PCB_TRACK(b)
                s.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1))); s.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
                s.SetWidth(MM(width)); s.SetLayer(layer); s.SetNet(ni)
                b.Add(s); nt += 1
    for v in find(net, "via"):
        m = re.search(r"_(\d+):(\d+)_um", unq(v[1]))
        dia, drill = int(m.group(1)) / 1000, int(m.group(2)) / 1000
        x, y = float(v[2]) * scale, -float(v[3]) * scale
        via = pcbnew.PCB_VIA(b)
        via.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        via.SetWidth(MM(dia)); via.SetDrill(MM(drill)); via.SetNet(ni)
        via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        b.Add(via); nv += 1
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(out, b)
print(f"imported {nt} segments, {nv} vias")
