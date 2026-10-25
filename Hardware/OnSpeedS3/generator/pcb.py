"""Build the KiCad 7 board from the same component list as the schematic.

Footprints are loaded from the standard libraries (or the project library), placed at the given
positions, and every pad gets the net from the component's pin map. Footprint paths are set to the
schematic symbol UUIDs, so KiCad treats board and schematic as linked.
"""
import pcbnew, os

FPLIB = "/usr/share/kicad/footprints"
PROJLIB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "onspeed.pretty")
MM = pcbnew.FromMM


def load_fp(fpid):
    lib, name = fpid.split(":")
    path = PROJLIB if lib == "OnSpeed" else os.path.join(FPLIB, lib + ".pretty")
    fp = pcbnew.FootprintLoad(path, name)
    if fp is None:
        raise KeyError(fpid)
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def build(parts, place, sch_uuid, out, outline=(100, 80), stackup_layers=4, holes=()):
    b = pcbnew.BOARD()
    b.SetCopperLayerCount(stackup_layers)
    ds = b.GetDesignSettings()
    # JLCPCB 4-layer capability with margin: 0.127/0.127 mm trace/space min, we design to 0.15/0.15
    ds.m_TrackMinWidth = MM(0.15)
    ds.m_MinClearance = MM(0.15)
    ds.m_ViasMinSize = MM(0.45)
    ds.m_MinThroughDrill = MM(0.2)   # LM5164 thermal-pad vias are 0.2 mm; JLC 4-layer allows 0.15
    ds.m_CopperEdgeClearance = MM(0.3)
    ds.m_HoleToHoleMin = MM(0.5)
    # nets
    nets = {}
    for s in parts:
        for pin, n in s.nets.items():
            if n and n not in nets:
                ni = pcbnew.NETINFO_ITEM(b, n)
                b.Add(ni)
                nets[n] = ni
    # outline with 2 mm corner radius on a 100 x 80 mm board
    W, H = outline
    r = 2.0
    def seg(x1, y1, x2, y2):
        l = pcbnew.PCB_SHAPE(b)
        l.SetShape(pcbnew.SHAPE_T_SEGMENT)
        l.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1))); l.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        l.SetLayer(pcbnew.Edge_Cuts); l.SetWidth(MM(0.1)); b.Add(l)
    def arc(cx, cy, sx, sy, ang):
        a = pcbnew.PCB_SHAPE(b)
        a.SetShape(pcbnew.SHAPE_T_ARC)
        a.SetCenter(pcbnew.VECTOR2I(MM(cx), MM(cy))); a.SetStart(pcbnew.VECTOR2I(MM(sx), MM(sy)))
        a.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(ang, pcbnew.DEGREES_T), True)
        a.SetLayer(pcbnew.Edge_Cuts); a.SetWidth(MM(0.1)); b.Add(a)
    seg(r, 0, W - r, 0); seg(W, r, W, H - r); seg(W - r, H, r, H); seg(0, H - r, 0, r)
    arc(W - r, r, W - r, 0, 90); arc(W - r, H - r, W, H - r, 90); arc(r, H - r, r, H, 90); arc(r, r, 0, r, 90)
    missing = []
    for s in parts:
        if s.ref.startswith("#") or not s.footprint:
            continue
        fp = load_fp(s.footprint)
        fp.SetReference(s.ref)
        fp.SetValue(s.value)
        fp.SetPath(pcbnew.KIID_PATH(f"/{sch_uuid}/{s.uuid}"))
        if s.ref not in place:
            missing.append(s.ref)
            x, y, rot, side = 120, 10 + len(missing) * 3, 0, "F"
        else:
            x, y, rot, side = (list(place[s.ref]) + ["F"])[:4]
        b.Add(fp)
        if side == "B":
            fp.Flip(fp.GetPosition(), False)
        fp.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        fp.SetOrientationDegrees(rot)
        for f in ("LCSC", "Rating", "MPN"):
            if f in s.fields:
                fp.SetProperty(f, s.fields[f])
        for pad in fp.Pads():
            num = pad.GetNumber()
            n = s.nets.get(num)
            if n is None and num in ("", "MP"):
                continue
            if n:
                pad.SetNet(nets[n])
        fp.Reference().SetTextSize(pcbnew.VECTOR2I(MM(0.8), MM(0.8)))
        fp.Reference().SetTextThickness(MM(0.12))
    pcbnew.SaveBoard(out, b)
    return b, missing
