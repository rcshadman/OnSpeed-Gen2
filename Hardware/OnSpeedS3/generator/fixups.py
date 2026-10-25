"""Track clean-up on the routed board: drops zero-length segments and merges collinear runs (and jogs
under 10 um), so no micro-jogs reach the factory. remove()/add()/via() are kept for hand fixes."""
import math, sys, pcbnew

MM, TO = pcbnew.FromMM, pcbnew.ToMM
src, out = sys.argv[1], sys.argv[2]
b = pcbnew.LoadBoard(src)


def near(p, x, y, tol=0.03):
    return abs(TO(p.x) - x) < tol and abs(TO(p.y) - y) < tol


def remove(net, pts):
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        hit = [t for t in b.GetTracks() if t.GetClass() == "PCB_TRACK" and t.GetNetname() == net and
               ((near(t.GetStart(), x1, y1) and near(t.GetEnd(), x2, y2)) or (near(t.GetStart(), x2, y2) and near(t.GetEnd(), x1, y1)))]
        assert hit, f"{net}: no segment {x1},{y1} - {x2},{y2}"
        for t in hit:
            b.Delete(t)


def add(net, pts, w=0.2, layer=pcbnew.F_Cu):
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        t = pcbnew.PCB_TRACK(b)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1))); t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w)); t.SetLayer(layer); t.SetNet(b.FindNet(net)); b.Add(t)


def via(net, x, y):
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y))); v.SetWidth(MM(0.6)); v.SetDrill(MM(0.3))
    v.SetNet(b.FindNet(net)); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); b.Add(v)


def dseg(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L = dx * dx + dy * dy
    u = 0 if L == 0 else max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / L))
    return math.hypot(px - x1 - u * dx, py - y1 - u * dy)


def connect(ref, num, width=0.3, layer=pcbnew.F_Cu, reach=6.0, step=0.05, clr=0.2):
    """Finish a connection the router left open: 8-direction grid search on one layer from the pad to the
    nearest copper of the same net, keeping class clearance to everything else."""
    p = [q for q in b.FindFootprintByReference(ref).Pads() if q.GetNumber() == num][0]
    net = p.GetNetname()
    cx, cy = TO(p.GetPosition().x), TO(p.GetPosition().y)
    box = lambda x, y: abs(x - cx) < reach + 2 and abs(y - cy) < reach + 2
    obs, tgt = [], []
    for f in b.GetFootprints():
        for q in f.Pads():
            if not q.IsOnLayer(layer):
                continue
            r = q.GetBoundingBox()
            g = ("r", TO(r.GetLeft()), TO(r.GetTop()), TO(r.GetRight()), TO(r.GetBottom()))
            if not box(TO(q.GetPosition().x), TO(q.GetPosition().y)):
                continue
            if q.GetNetname() == net:
                if not (f.GetReference() == ref and q.GetNumber() == num):
                    tgt.append(g)
            else:
                obs.append(g)
    for t in b.GetTracks():
        if t.GetClass() == "PCB_VIA":
            g = ("c", TO(t.GetPosition().x), TO(t.GetPosition().y), TO(t.GetWidth()) / 2)
        elif t.GetLayer() == layer:
            g = ("s", TO(t.GetStart().x), TO(t.GetStart().y), TO(t.GetEnd().x), TO(t.GetEnd().y), TO(t.GetWidth()) / 2)
        else:
            continue
        if not box(g[1], g[2]):
            continue
        (tgt if t.GetNetname() == net else obs).append(g)

    def dist(g, x, y):
        if g[0] == "r":
            return math.hypot(max(g[1] - x, 0, x - g[3]), max(g[2] - y, 0, y - g[4]))
        if g[0] == "c":
            return math.hypot(x - g[1], y - g[2]) - g[3]
        return dseg(x, y, *g[1:5]) - g[5]

    from collections import deque
    n = int(reach / step)
    prev = {(0, 0): None}
    q = deque([(0, 0)])
    pr = TO(max(p.GetSize().x, p.GetSize().y)) / 2
    while q:
        i, j = q.popleft()
        x, y = cx + i * step, cy + j * step
        if math.hypot(i, j) * step > pr and any(dist(g, x, y) <= 0 for g in tgt):
            path = [(i, j)]
            while prev[path[-1]] is not None:
                path.append(prev[path[-1]])
            pts = [(cx + a * step, cy + c * step) for a, c in reversed(path)]
            keep = [pts[0]]
            for a, c in zip(pts[1:], pts[2:] + [None]):
                if c is None or abs((a[0] - keep[-1][0]) * (c[1] - a[1]) - (a[1] - keep[-1][1]) * (c[0] - a[0])) > 1e-9:
                    keep.append(a)
            add(net, keep, width, layer)
            return len(keep)
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            k2 = (i + di, j + dj)
            if k2 in prev or abs(k2[0]) > n or abs(k2[1]) > n:
                continue
            x2, y2 = cx + k2[0] * step, cy + k2[1] * step
            own = abs(x2 - cx) < TO(p.GetBoundingBox().GetWidth()) / 2 and abs(y2 - cy) < TO(p.GetBoundingBox().GetHeight()) / 2
            if own or all(dist(g, x2, y2) >= width / 2 + clr for g in obs):
                prev[k2] = (i, j); q.append(k2)
    raise RuntimeError(f"no path for {ref}.{num} ({net})")


# U11 (PCM5102A) pin 8 AVDD: the router left it open; join it to the +3V3_A decoupling track beside it
def prune_dangling():
    """Drop track stubs that end in nothing (router leftovers), repeating until none are left."""
    n = 0
    while True:
        tr = [t for t in b.GetTracks() if t.GetClass() == "PCB_TRACK"]
        ends = {}
        for t in tr:
            for p in (t.GetStart(), t.GetEnd()):
                ends.setdefault((t.GetNetCode(), t.GetLayer(), p.x // 2000, p.y // 2000), 0)
                ends[(t.GetNetCode(), t.GetLayer(), p.x // 2000, p.y // 2000)] += 1
        dead = []
        for t in tr:
            for p in (t.GetStart(), t.GetEnd()):
                if ends[(t.GetNetCode(), t.GetLayer(), p.x // 2000, p.y // 2000)] > 1:
                    continue
                on_pad = any(q.GetNetCode() == t.GetNetCode() and q.IsOnLayer(t.GetLayer()) and q.HitTest(p)
                             for f in b.GetFootprints() for q in f.Pads())
                on_via = any(v.GetClass() == "PCB_VIA" and v.GetNetCode() == t.GetNetCode() and v.HitTest(p) for v in b.GetTracks())
                if not on_pad and not on_via:
                    dead.append(t); break
        if not dead:
            return n
        for t in dead:
            b.Remove(t)
        n += len(dead)


def connect2(ref, num, width, reach=8.0, step=0.1, clr=0.2, via_cost=3.0):
    """Two-layer (F.Cu/B.Cu) grid search with vias, for a connection boxed in on the top layer."""
    import heapq
    p = [q for q in b.FindFootprintByReference(ref).Pads() if q.GetNumber() == num][0]
    net = p.GetNetname()
    cx, cy = TO(p.GetPosition().x), TO(p.GetPosition().y)
    inb = lambda x, y: abs(x - cx) < reach + 2 and abs(y - cy) < reach + 2
    LAY = (pcbnew.F_Cu, pcbnew.B_Cu)
    obs = {L: [] for L in LAY}; tgt = {L: [] for L in LAY}; holes = []
    for f in b.GetFootprints():
        for q in f.Pads():
            if not inb(TO(q.GetPosition().x), TO(q.GetPosition().y)):
                continue
            r = q.GetBoundingBox()
            g = ("r", TO(r.GetLeft()), TO(r.GetTop()), TO(r.GetRight()), TO(r.GetBottom()))
            if q.GetDrillSize().x > 0:
                holes.append((TO(q.GetPosition().x), TO(q.GetPosition().y), TO(q.GetDrillSize().x) / 2))
            for L in LAY:
                if not q.IsOnLayer(L):
                    continue
                if q.GetNetname() == net:
                    if not (f.GetReference() == ref and q.GetNumber() == num):
                        tgt[L].append(g)
                else:
                    obs[L].append(g)
    for t in b.GetTracks():
        if t.GetClass() == "PCB_VIA":
            g = ("c", TO(t.GetPosition().x), TO(t.GetPosition().y), TO(t.GetWidth()) / 2)
            if not inb(g[1], g[2]):
                continue
            holes.append((g[1], g[2], TO(t.GetDrillValue()) / 2))
            for L in LAY:
                (tgt if t.GetNetname() == net else obs)[L].append(g)
        elif t.GetLayer() in LAY:
            g = ("s", TO(t.GetStart().x), TO(t.GetStart().y), TO(t.GetEnd().x), TO(t.GetEnd().y), TO(t.GetWidth()) / 2)
            if inb(g[1], g[2]) or inb(g[3], g[4]):
                (tgt if t.GetNetname() == net else obs)[t.GetLayer()].append(g)

    def dist(g, x, y):
        if g[0] == "r":
            return math.hypot(max(g[1] - x, 0, x - g[3]), max(g[2] - y, 0, y - g[4]))
        if g[0] == "c":
            return math.hypot(x - g[1], y - g[2]) - g[3]
        return dseg(x, y, *g[1:5]) - g[5]

    pw, ph = TO(p.GetBoundingBox().GetWidth()) / 2, TO(p.GetBoundingBox().GetHeight()) / 2
    own = lambda x, y: abs(x - cx) < pw and abs(y - cy) < ph
    ok = {}
    def free(i, j, L, r):
        key = (i, j, L, r)
        if key not in ok:
            x, y = cx + i * step, cy + j * step
            ok[key] = own(x, y) and L == pcbnew.F_Cu or all(dist(g, x, y) >= r + clr for g in obs[L])
        return ok[key]
    def via_ok(i, j):
        x, y = cx + i * step, cy + j * step
        return (all(free(i, j, L, 0.3) for L in LAY) and not own(x, y) and
                all(math.hypot(x - hx, y - hy) >= hr + 0.15 + 0.5 for hx, hy, hr in holes) and
                not (x < 9.5 and 32.5 < y < 51.5))
    start = (0, 0, pcbnew.F_Cu)
    dist_ = {start: 0}; prev = {start: None}
    pq = [(0, start)]
    n = int(reach / step)
    while pq:
        d, s = heapq.heappop(pq)
        if d > dist_[s]:
            continue
        i, j, L = s
        x, y = cx + i * step, cy + j * step
        if not own(x, y) and any(dist(g, x, y) <= 0 for g in tgt[L]):
            path = [s]
            while prev[path[-1]] is not None:
                path.append(prev[path[-1]])
            path.reverse()
            # emit: straight-run merge per layer, a via at each layer change
            run = [path[0]]
            def flush(run):
                pts = [(cx + a * step, cy + c * step) for a, c, _ in run]
                keep = [pts[0]]
                for a, c in zip(pts[1:], pts[2:] + [None]):
                    if c is None or abs((a[0] - keep[-1][0]) * (c[1] - a[1]) - (a[1] - keep[-1][1]) * (c[0] - a[0])) > 1e-9:
                        keep.append(a)
                if len(keep) > 1:
                    add(net, keep, width, run[0][2])
            nv = 0
            for st in path[1:]:
                if st[2] != run[-1][2]:
                    flush(run); via(net, cx + st[0] * step, cy + st[1] * step); nv += 1
                    run = [st]
                else:
                    run.append(st)
            flush(run)
            return nv
        moves = [((i + di, j + dj, L), math.hypot(di, dj) * step) for di, dj in
                 ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))]
        if via_ok(i, j):
            moves.append(((i, j, LAY[1] if L == LAY[0] else LAY[0]), via_cost))
        for s2, c in moves:
            if abs(s2[0]) > n or abs(s2[1]) > n:
                continue
            if s2[2] == L and not free(s2[0], s2[1], L, width / 2):
                continue
            nd = d + c
            if nd < dist_.get(s2, 1e9):
                dist_[s2] = nd; prev[s2] = s; heapq.heappush(pq, (nd, s2))
    raise RuntimeError(f"no two-layer path for {ref}.{num} ({net})")


print("dangling stubs removed:", prune_dangling())
# USB-C VBUS: the router joined only one of the two VBUS pad pairs. The data lines sit between them on the
# top layer, so the second pair crosses on the bottom layer through two vias.
for w in (0.5, 0.4):
    try:
        print("J1 VBUS A4 joined at", w, "mm with", connect2("J1", "A4", w), "vias")
        break
    except RuntimeError:
        pass
else:
    raise SystemExit("J1 VBUS still open")

# clean-up, done on plain tuples and written back in one pass (editing tracks in place crashes KiCad 7's bindings)
segs = [(t.GetNetCode(), t.GetLayer(), t.GetWidth(), (t.GetStart().x, t.GetStart().y), (t.GetEnd().x, t.GetEnd().y))
        for t in b.GetTracks() if t.GetClass() == "PCB_TRACK"]
n0 = len(segs)
segs = [s_ for s_ in segs if math.dist(s_[3], s_[4]) >= MM(0.005)]
fixed = {(p.GetNetCode(), p.GetPosition().x // 1000, p.GetPosition().y // 1000) for f in b.GetFootprints() for p in f.Pads()}
fixed |= {(v.GetNetCode(), v.GetPosition().x // 1000, v.GetPosition().y // 1000) for v in b.GetTracks() if v.GetClass() == "PCB_VIA"}
k = lambda p: (p[0] // 1000, p[1] // 1000)
changed = True
while changed:
    changed = False
    ends = {}
    for idx, (net, lay, w, a, c) in enumerate(segs):
        for p in (a, c):
            ends.setdefault((net, lay, k(p)), []).append(idx)
    for (net, lay, pk), ids in ends.items():
        if len(ids) != 2 or ids[0] == ids[1] or (net,) + pk in fixed:
            continue
        s1, s2 = segs[ids[0]], segs[ids[1]]
        if s1[2] != s2[2]:
            continue
        far1 = s1[4] if k(s1[3]) == pk else s1[3]
        mid = s1[3] if k(s1[3]) == pk else s1[4]
        far2 = s2[4] if k(s2[3]) == pk else s2[3]
        L = math.dist(far1, far2) or 1
        cross = (mid[0] - far1[0]) * (far2[1] - mid[1]) - (mid[1] - far1[1]) * (far2[0] - mid[0])
        if abs(cross) / L < MM(0.01):   # collinear, or a jog under 10 um
            segs[ids[0]] = (net, lay, s1[2], far1, far2)
            del segs[ids[1]]
            changed = True
            break
old = [t for t in b.GetTracks() if t.GetClass() == "PCB_TRACK"]
for t in old:
    b.Remove(t)
nets = {n.GetNetCode(): n for n in b.GetNetsByNetcode().values()}
for net, lay, w, a, c in segs:
    t = pcbnew.PCB_TRACK(b)
    t.SetStart(pcbnew.VECTOR2I(*a)); t.SetEnd(pcbnew.VECTOR2I(*c)); t.SetWidth(w); t.SetLayer(lay); t.SetNet(nets[net])
    b.Add(t)
print(f"fixups applied; segments {n0} -> {len(segs)}")
pcbnew.SaveBoard(out, b)
