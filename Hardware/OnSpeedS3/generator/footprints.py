"""Project footprints for parts with no KiCad library footprint. Coordinates come from the Gen2
Eagle board (Hardware/Eagle Files/onspeed.brd), converted to KiCad's y-down frame."""
import os
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "onspeed.pretty")


def fp(name, descr, pads, body, court, ref_y, val_y, extra=""):
    bx1, by1, bx2, by2 = body
    cx1, cy1, cx2, cy2 = court
    L = [f'(footprint "{name}" (version 20221018) (generator onspeed_footprints) (layer "F.Cu")',
         f'  (descr "{descr}") (attr through_hole)',
         f'  (fp_text reference "REF**" (at 0 {ref_y}) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))',
         f'  (fp_text value "{name}" (at 0 {val_y}) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))']
    for (x1, y1, x2, y2) in ((bx1, by1, bx2, by1), (bx2, by1, bx2, by2), (bx2, by2, bx1, by2), (bx1, by2, bx1, by1)):
        L.append(f'  (fp_line (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.1) (type solid)) (layer "F.Fab"))')
        L.append(f'  (fp_line (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))')
    for (x1, y1, x2, y2) in ((cx1, cy1, cx2, cy1), (cx2, cy1, cx2, cy2), (cx2, cy2, cx1, cy2), (cx1, cy2, cx1, cy1)):
        L.append(f'  (fp_line (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.05) (type solid)) (layer "F.CrtYd"))')
    for num, x, y, size, drill in pads:
        shape = "rect" if num == "1" else "circle"
        L.append(f'  (pad "{num}" thru_hole {shape} (at {x} {y}) (size {size} {size}) (drill {drill}) (layers "*.Cu" "*.Mask"))')
    L.append(extra + ")")
    open(os.path.join(OUT, name + ".kicad_mod"), "w").write("\n".join(L) + "\n")


# MS4525DO DIP-8, 0.5 in rows (Gen2 DIL08-500): 1-4 along one row, 8-5 opposite. Body about
# 10.2 x 8.9 mm (Gen2 silkscreen); the two 1/8 in barbs leave the pin-1/8 end and stand out about 7 mm.
# The courtyard covers the barbs so nothing is placed in the tube path.
P = []
for i, x in enumerate((-3.81, -1.27, 1.27, 3.81)):
    P.append((str(i + 1), x, 6.35, 1.8, 0.85))
    P.append((str(8 - i), x, -6.35, 1.8, 0.85))
barbs = "".join(f'\n  (fp_line (start -5.1 {y}) (end -12.1 {y}) (stroke (width 0.1) (type solid)) (layer "F.Fab"))' for y in (-2.05, 2.05))
fp("MS4525DO_DIP8_500mil", "TE MS4525DO dual-port DIP-8, 12.7 mm row spacing, Gen2 land pattern, barbs on the pin 1/8 end",
   P, (-5.1, -7.4, 5.1, 7.4), (-12.6, -7.9, 5.6, 7.9), -9.0, 9.0, barbs)

# Honeywell SSC SIP, 4 pins at 2.54 mm (Gen2 SIP-RN), body about 10 x 6 mm.
P = [(str(i + 1), -3.81 + i * 2.54, 0, 1.4, 0.7) for i in range(4)]
fp("Honeywell_SSC_SIP4_RN", "Honeywell TruStability SSC SIP single radial port, 4 pins 2.54 mm",
   P, (-5.0, -2.0, 5.0, 4.0), (-5.5, -2.5, 5.5, 4.5), -3.5, 5.4)


def dsub_with_holes(src, name, spacing, descr):
    """KiCad's EdgePinOffset9.40mm D-sub (matches the CONNFLY DS1037 *KT74 parts) has no board-lock
    holes; add the two 3.2 mm flange holes at the connector's B dimension, midway between the rows."""
    import re
    t = open(f"/usr/share/kicad/footprints/Connector_Dsub.pretty/{src}.kicad_mod").read()
    xs = [float(m) for m in re.findall(r'\(pad "\d+" thru_hole \w+ \(at (-?[\d.]+) ', t)]
    cx = (min(xs) + max(xs)) / 2
    holes = "".join(f'\n  (pad "0" thru_hole circle (at {cx + sx * spacing / 2:.3f} 1.42) (size 4 4) (drill 3.2) (layers "*.Cu" "*.Mask"))'
                    for sx in (-1, 1))
    t = t.replace(f'(footprint "{src}"', f'(footprint "{name}"', 1)
    t = re.sub(r'\(descr "[^"]*"\)', f'(descr "{descr}")', t, count=1)
    k = t.rindex("(model")
    t = t[:k] + holes.lstrip("\n") + "\n  " + t[k:]
    open(os.path.join(OUT, name + ".kicad_mod"), "w").write(t)


dsub_with_holes("DSUB-15_Female_Horizontal_P2.77x2.84mm_EdgePinOffset9.40mm", "DSUB-15_Female_Horizontal_P2.77x2.84mm_EdgePinOffset9.40mm_Holes33.32mm",
                33.32, "CONNFLY DS1037-15FNAKT74-0CC: KiCad EdgePinOffset9.40mm pattern plus 3.2 mm board-lock holes 33.32 mm apart")
dsub_with_holes("DSUB-9_Male_Horizontal_P2.77x2.84mm_EdgePinOffset9.40mm", "DSUB-9_Male_Horizontal_P2.77x2.84mm_EdgePinOffset9.40mm_Holes24.99mm",
                24.99, "CONNFLY DS1037-09MNAKT74-0CC: KiCad EdgePinOffset9.40mm pattern plus 3.2 mm board-lock holes 24.99 mm apart")


def esp32_body_courtyard():
    """ESP32-S3-WROOM-1 with a body-sized courtyard and no built-in antenna keep-out zone. The library
    version's 48 x 21 mm keep-out assumes the antenna overhangs the board edge; on this board the keep-out
    is a board-level rule area under the antenna (Wi-Fi is not used in flight)."""
    from symlib import parse
    from sch import ser
    src = "/usr/share/kicad/footprints/RF_Module.pretty/ESP32-S3-WROOM-1.kicad_mod"
    t = parse(open(src).read())
    keep = []
    for e in t:
        if isinstance(e, list) and e[0] == "zone":
            continue
        if isinstance(e, list) and any(isinstance(x, list) and x[0] == "layer" and x[1] == '"F.CrtYd"' for x in e):
            continue
        keep.append(e)
    keep[1] = '"ESP32-S3-WROOM-1_BodyCourtyard"'
    txt = ser(keep)
    x1, y1, x2, y2 = -9.3, -13.05, 9.3, 13.05
    lines = "".join(f'\n  (fp_line (start {a} {b}) (end {c} {d}) (stroke (width 0.05) (type solid)) (layer "F.CrtYd"))'
                    for a, b, c, d in ((x1, y1, x2, y1), (x2, y1, x2, y2), (x2, y2, x1, y2), (x1, y2, x1, y1)))
    txt = txt[:-1] + lines + ")\n"
    open(os.path.join(OUT, "ESP32-S3-WROOM-1_BodyCourtyard.kicad_mod"), "w").write(txt)


esp32_body_courtyard()
