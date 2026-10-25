"""Finish the routed board: GND pours on the outer layers, stitching and edge vias, silkscreen labels,
project file with the net classes, then DRC."""
import json, math, os, sys, pcbnew

MM, TO = pcbnew.FromMM, pcbnew.ToMM
src, out = sys.argv[1], sys.argv[2]
b = pcbnew.LoadBoard(src)
gnd = b.FindNet("GND")
b.GetDesignSettings().m_MinThroughDrill = MM(0.2)

# ------------------------------------------------------------ outer GND pours
for layer, net in ((pcbnew.F_Cu, gnd), (pcbnew.B_Cu, gnd), (pcbnew.In2_Cu, b.FindNet("+3V3"))):
    z = pcbnew.ZONE(b)
    z.SetLayer(layer); z.SetNet(net)
    z.SetLocalClearance(MM(0.3)); z.SetMinThickness(MM(0.25))
    # SMD pads join the pour solidly (reflow soldered, so no thermal problem); through-hole pads keep reliefs
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THT_THERMAL)
    z.SetThermalReliefGap(MM(0.3)); z.SetThermalReliefSpokeWidth(MM(0.4))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    o = z.Outline(); o.NewOutline()
    for x, y in ((0.4, 0.4), (99.6, 0.4), (99.6, 79.6), (0.4, 79.6)):
        o.Append(MM(x), MM(y))
    b.Add(z)

# ------------------------------------------------------------ stitching vias
obst = []  # (kind, geometry, net) in mm
for t in b.GetTracks():
    if t.GetClass() == "PCB_VIA":
        obst.append(("c", (TO(t.GetPosition().x), TO(t.GetPosition().y), TO(t.GetWidth()) / 2), t.GetNetname()))
    else:
        obst.append(("s", (TO(t.GetStart().x), TO(t.GetStart().y), TO(t.GetEnd().x), TO(t.GetEnd().y), TO(t.GetWidth()) / 2),
                     t.GetNetname(), {t.GetLayer()}))
holes = [(TO(t.GetPosition().x), TO(t.GetPosition().y), TO(t.GetDrillValue()) / 2) for t in b.GetTracks()
         if t.GetClass() == "PCB_VIA"]
court = []
for f in b.GetFootprints():
    for p in f.Pads():
        if p.GetDrillSize().x > 0:
            holes.append((TO(p.GetPosition().x), TO(p.GetPosition().y), TO(max(p.GetDrillSize().x, p.GetDrillSize().y)) / 2))
        bb = p.GetBoundingBox()
        lay = None if p.GetDrillSize().x > 0 else {L for L in (pcbnew.F_Cu, pcbnew.B_Cu) if p.IsOnLayer(L)}
        obst.append(("r", (TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom())), p.GetNetname(), lay))
    xs, ys = [], []
    for g in f.GraphicalItems():
        if g.GetLayer() in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
            gb = g.GetBoundingBox()
            xs += [TO(gb.GetLeft()), TO(gb.GetRight())]; ys += [TO(gb.GetTop()), TO(gb.GetBottom())]
    if xs:
        court.append((min(xs), min(ys), max(xs), max(ys)))


def dist_seg(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L = dx * dx + dy * dy
    u = 0 if L == 0 else max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / L))
    return math.hypot(px - x1 - u * dx, py - y1 - u * dy)


def free(x, y, r=0.3, clr=0.35, in_court_ok=False, net=None, holes_ok=False, layer=None):
    if not in_court_ok and any(a - 0.2 < x < c + 0.2 and bb - 0.2 < y < d + 0.2 for a, bb, c, d in court):
        return False
    if x < 9.5 and 32.5 < y < 51.5:   # antenna keep-out
        return False
    if not holes_ok and any(math.hypot(x - hx, y - hy) < hr + 0.15 + 0.55 for hx, hy, hr in holes):
        return False   # JLC hole-to-hole spacing 0.5 mm, with margin
    for k, g, n, *L in obst:
        if net is not None and n == net:
            continue
        if layer is not None and L and L[0] is not None and layer not in L[0]:
            continue   # a track only has to clear copper on its own layer
        if k == "c":
            d = math.hypot(x - g[0], y - g[1]) - g[2]
        elif k == "s":
            d = dist_seg(x, y, *g[:4]) - g[4]
        else:
            dx = max(g[0] - x, 0, x - g[2]); dy = max(g[1] - y, 0, y - g[3]); d = math.hypot(dx, dy)
        if d < r + clr:
            return False
    return True


def via(x, y):
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
    v.SetWidth(MM(0.6)); v.SetDrill(MM(0.3)); v.SetNet(gnd)
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    b.Add(v)
    obst.append(("c", (x, y, 0.3), "GND"))
    holes.append((x, y, 0.15))


def track(x1, y1, x2, y2, layer, w=0.3):
    t = pcbnew.PCB_TRACK(b)
    t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1))); t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
    t.SetWidth(MM(w)); t.SetLayer(layer); t.SetNet(gnd); b.Add(t)
    obst.append(("s", (x1, y1, x2, y2, w / 2), "GND", {layer}))


# ground fan-out: every SMD ground pad gets its own via to the In1 ground plane, next to the pad
n_fan = n_miss = 0
missed = []
for f in b.GetFootprints():
    for p in f.Pads():
        if p.GetNetname() != "GND" or p.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
            continue
        layer = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
        cx, cy = TO(p.GetPosition().x), TO(p.GetPosition().y)
        bb = p.GetBoundingBox()
        hw, hh = TO(bb.GetWidth()) / 2, TO(bb.GetHeight()) / 2
        if any(math.hypot(cx - g[0], cy - g[1]) < max(hw, hh) + 0.2 for k, g, n, *L in obst if k == "c" and n == "GND"):
            n_fan += 1; continue   # a ground via already touches this pad
        done = False
        for clr in (0.2,):
            for d in (0.55, 0.75, 1.0, 1.3, 1.7, 2.2, 2.8):
                best = None
                for a in range(0, 360, 45):   # 45-degree steps keep the stubs neat
                    ux, uy = math.cos(math.radians(a)), math.sin(math.radians(a))
                    edge = min(hw / abs(ux) if abs(ux) > 1e-6 else 1e9, hh / abs(uy) if abs(uy) > 1e-6 else 1e9)
                    vx, vy = cx + ux * (edge + d), cy + uy * (edge + d)
                    if not free(vx, vy, 0.3, clr, in_court_ok=True, net="GND"):
                        continue
                    if not all(free(cx + (vx - cx) * u, cy + (vy - cy) * u, 0.125, clr, in_court_ok=True, net="GND", holes_ok=True, layer=layer)
                               for u in [k2 / 20 for k2 in range(2, 20)]):
                        continue
                    # point the via into open board, away from other nets' pins, so it does not box in a neighbour
                    room = min([math.hypot(max(g[0] - vx, 0, vx - g[2]), max(g[1] - vy, 0, vy - g[3]))
                                for k, g, n, *L in obst if k == "r" and n != "GND" and abs(g[0] - vx) < 4 and abs(g[1] - vy) < 4] or [9])
                    if best is None or room > best[0] + 1e-6:
                        best = (room, vx, vy)
                if best:
                    _, vx, vy = best
                    track(cx, cy, vx, vy, layer, 0.25); via(vx, vy); n_fan += 1; done = True
                if done:
                    break
            if done:
                break
        if not done:
            n_miss += 1; missed.append(f"{f.GetReference()}.{p.GetNumber()}")
print(f"ground fan-out: {n_fan} pads tied, {n_miss} left to the outer pour: {missed}")

# pads the fan-out could not serve: run a short straight or L-shaped track to the nearest ground via or ground pin
def seg_free(x1, y1, x2, y2, layer, w=0.25, last=True):
    n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.1))
    return all(free(x1 + (x2 - x1) * k / n, y1 + (y2 - y1) * k / n, w / 2, 0.2, in_court_ok=True, net="GND", holes_ok=True, layer=layer)
               for k in range(1, n if last else n + 1))


n_tie = 0
for ref in list(missed):
    r, num = ref.split(".")
    p = [q for q in b.FindFootprintByReference(r).Pads() if q.GetNumber() == num][0]
    layer = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
    cx, cy = TO(p.GetPosition().x), TO(p.GetPosition().y)
    targets = [(g[0], g[1]) for k, g, n, *L in obst if k == "c" and n == "GND"]
    targets += [(TO(q.GetPosition().x), TO(q.GetPosition().y)) for f in b.GetFootprints() for q in f.Pads()
                if q.GetNetname() == "GND" and q is not p and (q.GetDrillSize().x > 0 or q.IsOnLayer(layer)) and
                f"{f.GetReference()}.{q.GetNumber()}" not in missed]
    targets.sort(key=lambda t: math.hypot(t[0] - cx, t[1] - cy))
    for tx, ty in targets:
        if math.hypot(tx - cx, ty - cy) > 6:
            break
        paths = [[(cx, cy), (tx, ty)], [(cx, cy), (tx, cy), (tx, ty)], [(cx, cy), (cx, ty), (tx, ty)]]
        ok = next((pt for pt in paths if all(seg_free(*pt[i], *pt[i + 1], layer, last=(i == len(pt) - 2)) for i in range(len(pt) - 1))), None)
        if ok:
            for i in range(len(ok) - 1):
                track(*ok[i], *ok[i + 1], layer, 0.25)
            missed.remove(ref); n_tie += 1
            break
print(f"ground ties: {n_tie} more pads joined by short tracks; still on the pour only: {missed}")


# whatever is still left: grid search (0.1 mm, 8 directions, so 45-degree track) out to a spot where a via fits
def maze(cx, cy, layer, reach=4.0, step=0.1):
    global obst
    keep = obst
    obst = [o for o in keep if (o[0] == "c" and abs(o[1][0] - cx) < reach + 1 and abs(o[1][1] - cy) < reach + 1) or
            (o[0] == "s" and min(o[1][0], o[1][2]) < cx + reach + 1 and max(o[1][0], o[1][2]) > cx - reach - 1 and
             min(o[1][1], o[1][3]) < cy + reach + 1 and max(o[1][1], o[1][3]) > cy - reach - 1) or
            (o[0] == "r" and o[1][0] < cx + reach + 1 and o[1][2] > cx - reach - 1 and o[1][1] < cy + reach + 1 and o[1][3] > cy - reach - 1)]
    try:
        from collections import deque
        n = int(reach / step)
        prev = {(0, 0): None}
        q = deque([(0, 0)])
        while q:
            i, j = q.popleft()
            x, y = cx + i * step, cy + j * step
            if math.hypot(i, j) * step > 0.5 and free(x, y, 0.3, 0.2, in_court_ok=True, net="GND"):
                path = [(i, j)]
                while prev[path[-1]] is not None:
                    path.append(prev[path[-1]])
                return [(cx + a * step, cy + c * step) for a, c in reversed(path)]
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                k = (i + di, j + dj)
                if k in prev or abs(k[0]) > n or abs(k[1]) > n:
                    continue
                if free(cx + k[0] * step, cy + k[1] * step, 0.125, 0.2, in_court_ok=True, net="GND", holes_ok=True, layer=layer):
                    prev[k] = (i, j); q.append(k)
                elif math.hypot(*k) * step < 0.6:   # still inside the pad itself
                    prev[k] = (i, j); q.append(k)
        return None
    finally:
        obst = keep


def maze_pad(ref):
    r, num = ref.split(".")
    p = [q for q in b.FindFootprintByReference(r).Pads() if q.GetNumber() == num][0]
    layer = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
    path = maze(TO(p.GetPosition().x), TO(p.GetPosition().y), layer)
    if not path:
        return False
    # merge straight runs into single segments
    pts = [path[0]]
    for a, c in zip(path[1:], path[2:] + [None]):
        if c is None or abs((a[0] - pts[-1][0]) * (c[1] - a[1]) - (a[1] - pts[-1][1]) * (c[0] - a[0])) > 1e-6:
            pts.append(a)
    for u, v in zip(pts, pts[1:]):
        track(*u, *v, layer, 0.25)
    via(*pts[-1])
    return True


n_maze = 0
for ref in list(missed):
    if maze_pad(ref):
        missed.remove(ref); n_maze += 1
ripped = set()
if os.environ.get("FANOUT_ONLY") == "1":
    # still boxed in: lift the other nets' tracks right around the pin, escape, and let the router put them back
    for ref in list(missed):
        r, num = ref.split(".")
        p = [q for q in b.FindFootprintByReference(r).Pads() if q.GetNumber() == num][0]
        layer = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
        cx, cy = TO(p.GetPosition().x), TO(p.GetPosition().y)
        for t in list(b.GetTracks()):
            if t.GetClass() != "PCB_TRACK" or t.GetNetname() == "GND" or t.GetLayer() != layer:
                continue
            g = (TO(t.GetStart().x), TO(t.GetStart().y), TO(t.GetEnd().x), TO(t.GetEnd().y))
            if dist_seg(cx, cy, *g) < 1.3:
                obst[:] = [o for o in obst if not (o[0] == "s" and all(abs(a - c) < 1e-6 for a, c in zip(o[1][:4], g)))]
                ripped.add(t.GetNetname()); b.Delete(t)
        if maze_pad(ref):
            missed.remove(ref); n_maze += 1
    print(f"lifted for re-routing: {sorted(ripped)}")
print(f"ground maze: {n_maze} more pads; still on the pour only: {missed}")
if os.environ.get("FANOUT_ONLY") == "1":
    # hand the router a board without the outer pours, so it sees which ground pins really are unconnected
    for z in list(b.Zones()):
        if not z.GetIsRuleArea() and z.GetLayer() in (pcbnew.F_Cu, pcbnew.B_Cu, pcbnew.In2_Cu):
            b.Delete(z)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(out, b)
    sys.exit(0)

n_edge = n_grid = 0
# edge ring: GND vias every 2.5 mm, 1.2 mm in from the edge (ESD return path, keeps the outer pours tied)
for i in range(int(100 / 2.5) + 1):
    for x, y in ((1.2 + i * 2.5, 1.2), (1.2 + i * 2.5, 78.8)):
        if x < 99 and free(x, y, in_court_ok=False):
            via(x, y); n_edge += 1
for i in range(int(80 / 2.5) + 1):
    for x, y in ((1.2, 1.2 + i * 2.5), (98.8, 1.2 + i * 2.5)):
        if 2 < y < 78 and free(x, y):
            via(x, y); n_edge += 1
# interior grid every 5 mm
for gx in range(2, 20):
    for gy in range(2, 16):
        x, y = gx * 5.0, gy * 5.0
        if free(x, y):
            via(x, y); n_grid += 1
print(f"stitching vias: {n_edge} edge, {n_grid} grid")

# ------------------------------------------------------------ silkscreen
SMALL = ("R", "C", "FB", "D", "TP", "H", "FID", "JP", "F", "L")
for f in b.GetFootprints():
    ref = f.Reference()
    prefix = "".join(ch for ch in f.GetReference() if ch.isalpha())
    big = f.GetReference() in ("D1", "D2", "D3", "D19", "F1", "L1", "C1") or prefix in ("U", "J", "SW")
    ref.SetVisible(big)
    ref.SetTextSize(pcbnew.VECTOR2I(MM(1.0), MM(1.0))); ref.SetTextThickness(MM(0.15))
    f.Value().SetVisible(False)


def silk(text, x, y, size=1.0, layer=pcbnew.F_SilkS, angle=0, bold=False):
    t = pcbnew.PCB_TEXT(b)
    t.SetText(text); t.SetLayer(layer)
    t.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
    t.SetTextSize(pcbnew.VECTOR2I(MM(size), MM(size))); t.SetTextThickness(MM(size * (0.2 if bold else 0.15)))
    t.SetTextAngleDegrees(angle)
    if layer == pcbnew.B_SilkS:
        t.SetMirrored(True)
    b.Add(t)


# connector names go on the back: on the front they would sit under the D-sub bodies
silk("AIRCRAFT DB-15", 77.3, 76.0, 1.2, pcbnew.B_SilkS, bold=True)
silk("CAN-FiX DE-9", 13.0 + 5.5, 76.0, 1.2, pcbnew.B_SilkS, bold=True)
for txt, x, y, s in (("USB", 45.0, 72.6, 0.9),
                     ("P-FWD", 35.0, 15.0, 0.9), ("P45", 12.0, 15.0, 0.9), ("STATIC", 22.5, 11.2, 0.7),
                     ("microSD", 84.0, 13.4, 0.9), ("I2C EXP", 52.0, 8.6, 0.8), ("RST", 36.6, 46.0, 0.8), ("BOOT", 36.9, 38.6, 0.8),
                     ("PWR", 26.3, 26.6, 0.7), ("STAT", 26.6, 29.4, 0.7), ("CAN TERM", 22.4, 66.2, 0.6),
                     ("IMU  X>", 50.0, 34.2, 0.7)):
    silk(txt, x, y, max(s, 0.8))
silk("OnSpeed S3  rev A  2026-10", 50.0, 76.0 - 52.0, 1.2, bold=True)
silk("ONSPEED S3 rev A", 50.0, 20.0, 1.5, pcbnew.B_SilkS, bold=True)
silk("Static-sensitive. Conformal coat after test, keep sensor ports and connectors masked.", 50.0, 23.0, 0.8, pcbnew.B_SilkS)


# ------------------------------------------------------------ silkscreen tidy
# Nothing printed on a pad, past the board edge or on top of other print: move it, or drop a reference.
def bbm(item):
    r = item.GetBoundingBox()
    return TO(r.GetLeft()), TO(r.GetTop()), TO(r.GetRight()), TO(r.GetBottom())


def hit(a, c, m=0.15):
    return a[0] < c[2] + m and c[0] < a[2] + m and a[1] < c[3] + m and c[1] < a[3] + m


for side, silk_l, cu in ((0, pcbnew.F_SilkS, pcbnew.F_Cu), (1, pcbnew.B_SilkS, pcbnew.B_Cu)):
    for f in b.GetFootprints():
        for g in list(f.GraphicalItems()):
            if g.GetLayer() == silk_l and g.GetClass() != "FP_TEXT":
                x0, y0, x1, y1 = bbm(g)
                if x0 < 0.3 or y0 < 0.3 or x1 > 99.7 or y1 > 79.7:
                    f.Remove(g)
    pads = [bbm(p) for f in b.GetFootprints() for p in f.Pads() if p.IsOnLayer(cu)]
    lines = [(f.GetReference(), bbm(g)) for f in b.GetFootprints() for g in f.GraphicalItems()
             if g.GetLayer() == silk_l and g.GetClass() != "FP_TEXT"]
    placed = []

    def clean(box, owner=None):
        return (box[0] > 0.4 and box[1] > 0.4 and box[2] < 99.6 and box[3] < 79.6 and
                not any(hit(box, p, 0.1) for p in pads) and not any(hit(box, q) for q in placed) and
                not any(hit(box, l, 0.05) for o, l in lines if o != owner or True))

    labels = [t for t in b.GetDrawings() if t.GetClass() == "PCB_TEXT" and t.GetLayer() == silk_l]
    for t in labels:
        x, y = TO(t.GetPosition().x), TO(t.GetPosition().y)
        for dx, dy in [(0, 0)] + [(dx, dy) for d in (0.6, 1.2, 1.8, 2.4) for dx, dy in ((0, -d), (0, d), (-d, 0), (d, 0))]:
            t.SetPosition(pcbnew.VECTOR2I(MM(x + dx), MM(y + dy)))
            if clean(bbm(t)):
                break
        else:
            t.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        placed.append(bbm(t))
    n_hidden = 0
    for f in b.GetFootprints():
        ref = f.Reference()
        if not ref.IsVisible() or ref.GetLayer() != silk_l:
            continue
        cx0, cy0, cx1, cy1 = bbm(f)
        mx, my = (cx0 + cx1) / 2, (cy0 + cy1) / 2
        h = 0.8
        cands = [(mx, cy0 - h), (mx, cy1 + h), (cx0 - 2.0, my), (cx1 + 2.0, my), (mx, my)]
        for x, y in cands:
            ref.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
            if clean(bbm(ref), f.GetReference()):
                placed.append(bbm(ref))
                break
        else:
            ref.SetVisible(False); n_hidden += 1
    print(f"silk tidy side {side}: {len(labels)} labels, {n_hidden} references hidden")

# every piece of the outer ground pours gets at least two vias to In1, so no copper island floats
filler = pcbnew.ZONE_FILLER(b)
n_frag = 0
for rnd in range(3):
    filler.Fill(b.Zones())
    added = 0
    for z in b.Zones():
        if z.GetIsRuleArea() or z.GetNetname() != "GND" or z.GetLayer() not in (pcbnew.F_Cu, pcbnew.B_Cu):
            continue
        polys = z.GetFilledPolysList(z.GetLayer())
        for i in range(polys.OutlineCount()):
            ch = polys.Outline(i)
            bb = ch.BBox()
            inside = lambda x, y: ch.PointInside(pcbnew.VECTOR2I(MM(x), MM(y)))
            have = sum(1 for hx, hy, hr in holes if inside(hx, hy) and any(
                n == "GND" and k == "c" and abs(g[0] - hx) < 1e-3 and abs(g[1] - hy) < 1e-3 for k, g, n, *L in obst))
            if have >= 2:
                continue
            x0, y0, x1, y1 = TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom())
            step = 0.2
            cands = [(x0 + i2 * step, y0 + j2 * step) for i2 in range(int((x1 - x0) / step) + 1) for j2 in range(int((y1 - y0) / step) + 1)]
            for x, y in cands:
                if have >= 2:
                    break
                if all(inside(x + dx, y + dy) for dx, dy in ((0, 0), (0.36, 0), (-0.36, 0), (0, 0.36), (0, -0.36))) \
                        and free(x, y, 0.3, 0.2, in_court_ok=True, net="GND"):
                    via(x, y); have += 1; added += 1
    n_frag += added
    if not added:
        break
filler.Fill(b.Zones())
print(f"pour-island vias: {n_frag}")
pcbnew.SaveBoard(out, b)

# ------------------------------------------------------------ project file with net classes (for KiCad users)
CL = {"Power": (0.5, 0.2, 0.8, 0.4, ["VIN_RAW", "VIN_F", "VIN_P", "VIN", "+5V", "BUCK_SW", "VBUS", "3V3_OUT_F", "X_3V3"]),
      "Rail": (0.3, 0.2, 0.6, 0.3, ["+3V3_S", "+3V3_A", "+5V_S", "+3V3", "DAC_*", "RS_*", "AUDIO_GND"])}
pro = {"board": {"design_settings": {"rules": {"min_clearance": 0.15, "min_track_width": 0.15, "min_via_diameter": 0.45,
                                              "min_through_hole_diameter": 0.2, "min_hole_to_hole": 0.5,
                                              "min_copper_edge_clearance": 0.3, "min_hole_clearance": 0.25,
                                              "min_text_height": 0.8, "min_text_thickness": 0.12, "min_silk_clearance": 0.0}}},
       "meta": {"filename": out.split("/")[-1].replace(".kicad_pcb", ".kicad_pro"), "version": 1},
       "net_settings": {"classes": [{"name": "Default", "track_width": 0.2, "clearance": 0.2, "via_diameter": 0.6, "via_drill": 0.3,
                                     "diff_pair_width": 0.2, "diff_pair_gap": 0.25, "microvia_diameter": 0.3, "microvia_drill": 0.1,
                                     "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
                                     "schematic_color": "rgba(0, 0, 0, 0.000)"}] +
                        [{"name": k, "track_width": v[0], "clearance": v[1], "via_diameter": v[2], "via_drill": v[3],
                          "diff_pair_width": 0.2, "diff_pair_gap": 0.25, "microvia_diameter": 0.3, "microvia_drill": 0.1,
                          "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
                          "schematic_color": "rgba(0, 0, 0, 0.000)"} for k, v in CL.items()],
                        "meta": {"version": 3},
                        "netclass_patterns": [{"netclass": k, "pattern": n} for k, v in CL.items() for n in v[4]]}}
json.dump(pro, open(out.replace(".kicad_pcb", ".kicad_pro"), "w"), indent=2)

# ------------------------------------------------------------ DRC
import shutil
shutil.copy(os.path.join(os.path.dirname(os.path.abspath(__file__)), "dru_rules.kicad_dru"), out.replace(".kicad_pcb", ".kicad_dru"))
b2 = pcbnew.LoadBoard(out)
b2.GetDesignSettings().m_MinThroughDrill = MM(0.2)
rpt = out.replace(".kicad_pcb", "_drc.rpt")
pcbnew.WriteDRCReport(b2, rpt, pcbnew.EDA_UNITS_MILLIMETRES, True)
txt = open(rpt).read()
print(txt[:200])
import re
for k in ("violations", "unconnected", "footprint errors"):
    m = re.search(rf"\*\* Found (\d+) (DRC )?{k}", txt)
    print(k, m.group(1) if m else "?")
