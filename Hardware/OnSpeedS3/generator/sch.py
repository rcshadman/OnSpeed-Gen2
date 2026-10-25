"""Schematic writer for KiCad 7.

Builds a .kicad_sch from a component list. Every pin gets a short wire stub ending in a net label.
That label-per-pin style keeps the generated drawing readable and makes the connectivity explicit
for review (every pin shows its net name).
"""
import uuid, math, copy, os
from symlib import lib, unq, find_symbol, pins as lib_pins, parse

GRID = 1.27
NS = uuid.UUID('6f1c2a7e-0b5d-4f0e-9a51-0e5ee0d0a5ed')


def U():
    return str(uuid.uuid4())


def q(s):
    return '"' + str(s).replace('\\', '\\\\').replace('"', '\\"') + '"'


def ser(x, ind=0):
    """Serialise a parsed s-expression list back to text, keeping element order."""
    if not isinstance(x, list):
        return x
    if all(not isinstance(e, list) for e in x):
        return "(" + " ".join(x) + ")"
    out = "(" + x[0]
    for e in x[1:]:
        out += ("\n" + "  " * (ind + 1) + ser(e, ind + 1)) if isinstance(e, list) else " " + e
    return out + ")"


def snap(v):
    return round(round(v / GRID) * GRID, 4)


class Symbol:
    """One schematic symbol instance."""

    def __init__(self, ref, libname, sym, value, footprint="", fields=None, nets=None, at=(0, 0), rot=0,
                 mirror=None, dnp=False, custom=None, in_bom=True):
        self.ref, self.lib, self.sym, self.value, self.footprint = ref, libname, sym, value, footprint
        self.fields = fields or {}
        self.nets = nets or {}  # pin number -> net name ("" or None = no-connect)
        self.at, self.rot, self.mirror, self.dnp = at, rot, mirror, dnp
        self.custom = custom  # parsed symbol definition for custom library symbols
        self.uuid = str(uuid.uuid5(NS, ref))  # stable, so the board links to the schematic across rebuilds
        self.in_bom = in_bom

    @property
    def lib_id(self):
        return f"{self.lib}:{self.sym}"

    def definition(self):
        if self.custom is not None:
            return self.custom
        return find_symbol(self.lib, self.sym)

    def pin_list(self):
        if self.custom is not None:
            from symlib import pins_of
            return pins_of(self.custom)
        return lib_pins(self.lib, self.sym)

    def pin_pos(self, p):
        """Absolute schematic position of a pin and the outward direction (unit vector)."""
        x, y = p["x"], -p["y"]  # library y up -> schematic y down
        ang = p["rot"]
        # direction from the connection point towards the body (in schematic coords)
        dx, dy = math.cos(math.radians(ang)), -math.sin(math.radians(ang))
        if self.mirror == "y":
            x, dx = -x, -dx
        r = math.radians(-self.rot)  # schematic rotation is counter-clockwise in the UI
        cx = x * math.cos(r) - y * math.sin(r)
        cy = x * math.sin(r) + y * math.cos(r)
        ddx = dx * math.cos(r) - dy * math.sin(r)
        ddy = dx * math.sin(r) + dy * math.cos(r)
        return (self.at[0] + cx, self.at[1] + cy), (-ddx, -ddy)


def embed_symbol(s, name):
    """Copy a library symbol into lib_symbols form, resolving 'extends'."""
    libname, sym = name.split(":")
    d = copy.deepcopy(s.definition())
    ext = [e for e in d if isinstance(e, list) and e[0] == "extends"]
    if ext:
        parent = copy.deepcopy(find_symbol(libname, unq(ext[0][1])))
        pname = unq(parent[1])
        child_props = {unq(e[1]): e for e in d if isinstance(e, list) and e[0] == "property"}
        new = []
        for e in parent:
            if isinstance(e, list) and e[0] == "property" and unq(e[1]) in child_props:
                new.append(child_props[unq(e[1])])
            elif isinstance(e, list) and e[0] == "symbol":
                e = copy.deepcopy(e)
                e[1] = q(unq(e[1]).replace(pname, sym, 1))
                new.append(e)
            else:
                new.append(e)
        d = new
    d[1] = q(name)
    return d


class Schematic:
    def __init__(self, title, paper="A1"):
        self.title, self.paper = title, paper
        self.symbols, self.wires, self.labels, self.texts, self.noconn, self.rects = [], [], [], [], [], []
        self.uuid = str(uuid.uuid5(NS, 'sheet:' + title))
        self.pwr_n = 0

    def add(self, s):
        self.symbols.append(s)
        return s

    def text(self, txt, x, y, size=2.0, bold=False):
        self.texts.append((txt, x, y, size, bold))

    def rect(self, x1, y1, x2, y2):
        self.rects.append((x1, y1, x2, y2))

    def connect_all(self, stub=2.54):
        """Wire stub + label on every pin that has a net; no-connect flag on pins mapped to None."""
        for s in self.symbols:
            for p in s.pin_list():
                if p["hidden"] and p["number"] not in s.nets:
                    continue
                if p["number"] not in s.nets:
                    raise KeyError(f"{s.ref} pin {p['number']} ({p['name']}) has no net assignment")
                net = s.nets[p["number"]]
                (x, y), (dx, dy) = s.pin_pos(p)
                x, y = snap(x), snap(y)
                if net is None:
                    self.noconn.append((x, y))
                    continue
                if p["hidden"]:
                    continue  # stacked hidden pins share the visible pin's position
                ex, ey = snap(x + dx * stub), snap(y + dy * stub)
                self.wires.append(((x, y), (ex, ey)))
                if abs(dx) > abs(dy):
                    ang = 0 if dx > 0 else 180
                else:
                    ang = 270 if dy > 0 else 90
                self.labels.append((net, ex, ey, ang))

    def write(self, path, project):
        L = []
        L.append(f'(kicad_sch (version 20230121) (generator eeschema)\n  (uuid {self.uuid})\n  (paper "{self.paper}")')
        L.append(f'  (title_block (title {q(self.title)}) (date "2026-10-25") (rev "A") (company "OnSpeed ESP32-S3 port"))')
        L.append("  (lib_symbols")
        seen = set()
        for s in self.symbols:
            if s.lib_id in seen:
                continue
            seen.add(s.lib_id)
            L.append("    " + ser(embed_symbol(s, s.lib_id), 2))
        L.append("  )")
        for (x1, y1), (x2, y2) in self.wires:
            L.append(f"  (wire (pts (xy {x1} {y1}) (xy {x2} {y2})) (stroke (width 0) (type default)) (uuid {U()}))")
        for (x, y) in self.noconn:
            L.append(f"  (no_connect (at {x} {y}) (uuid {U()}))")
        for net, x, y, ang in self.labels:
            just = "left" if ang in (0, 90) else "right"
            L.append(f"  (label {q(net)} (at {x} {y} {ang}) (fields_autoplaced) (effects (font (size 1.27 1.27)) (justify {just} bottom)) (uuid {U()}))")
        for txt, x, y, size, bold in self.texts:
            b = " bold" if bold else ""
            L.append(f"  (text {q(txt)} (at {x} {y} 0) (effects (font (size {size} {size}){b}) (justify left bottom)) (uuid {U()}))")
        for x1, y1, x2, y2 in self.rects:
            for a, b in (((x1, y1), (x2, y1)), ((x2, y1), (x2, y2)), ((x2, y2), (x1, y2)), ((x1, y2), (x1, y1))):
                L.append(f"  (polyline (pts (xy {a[0]} {a[1]}) (xy {b[0]} {b[1]})) (stroke (width 0.3) (type dash)) (uuid {U()}))")
        for s in self.symbols:
            x, y = snap(s.at[0]), snap(s.at[1])
            mir = f" (mirror {s.mirror})" if s.mirror else ""
            L.append(f'  (symbol (lib_id {q(s.lib_id)}) (at {x} {y} {s.rot}){mir} (unit 1)')
            L.append(f'    (in_bom {"yes" if s.in_bom else "no"}) (on_board yes) (dnp {"yes" if s.dnp else "no"}) (uuid {s.uuid})')
            hide_val = s.ref.startswith("#")
            ys = [s.pin_pos(p)[0][1] for p in s.pin_list()] or [y]
            big = len(s.pin_list()) > 3
            top = min(ys + [y - 2.54]) - (3.81 if big else 2.54)
            bot = max(ys + [y + 2.54]) + (3.81 if big else 2.54)
            if s.rot == 90 and not big:  # horizontal two-pin part: text above and below the body
                top, bot = y - 2.54, y + 2.54
            props = [("Reference", s.ref, (x, round(top, 2)), hide_val), ("Value", s.value, (x, round(bot, 2)), hide_val),
                     ("Footprint", s.footprint, (x, y), True), ("Datasheet", s.fields.get("Datasheet", "~"), (x, y), True)]
            for k, v in s.fields.items():
                if k != "Datasheet":
                    props.append((k, v, (x, y), True))
            for k, v, (px, py), hid in props:
                h = " hide" if hid else ""
                L.append(f'    (property {q(k)} {q(v)} (at {px} {py} 0) (effects (font (size 1.27 1.27)){h}))')
            for p in s.pin_list():
                L.append(f'    (pin {q(p["number"])} (uuid {U()}))')
            L.append(f'    (instances (project {q(project)} (path "/{self.uuid}" (reference {q(s.ref)}) (unit 1))))')
            L.append("  )")
        L.append('  (sheet_instances (path "/" (page "1")))')
        L.append(")")
        open(path, "w").write("\n".join(L) + "\n")
