"""Generate simple rectangular KiCad symbols for parts with no standard-library symbol."""
from symlib import parse

def box_symbol(name, left=(), right=(), top=(), bottom=(), ref="U", width=None, value=None):
    """Each side is a list of (number, name, type) or None for a gap. Returns parsed s-expr."""
    P = 2.54
    n = max(len(left), len(right), 1)
    tb = max(len(top), len(bottom), 0)
    w = width or max(12.7, (tb + 1) * P, max([len(p[1]) for side in (left, right) for p in side if p] + [4]) * 1.27 * 2 + 5.08)
    w = round(w / P) * P
    h = (n + 1) * P
    hw, hh = w / 2, h / 2
    pins = []
    def add(side, i, p, count):
        if p is None: return
        num, pname, typ = p
        if side == 'L': x, y, a = -hw - P, hh - P * (i + 1), 0
        if side == 'R': x, y, a = hw + P, hh - P * (i + 1), 180
        off = (count - 1) * P / 2
        if side == 'T': x, y, a = -off + i * P, hh + P, 270
        if side == 'B': x, y, a = -off + i * P, -hh - P, 90
        pins.append(f'(pin {typ} line (at {x:.2f} {y:.2f} {a}) (length {P}) (name "{pname}" (effects (font (size 1.27 1.27)))) (number "{num}" (effects (font (size 1.27 1.27)))))')
    for i, p in enumerate(left): add('L', i, p, len(left))
    for i, p in enumerate(right): add('R', i, p, len(right))
    for i, p in enumerate(top): add('T', i, p, len(top))
    for i, p in enumerate(bottom): add('B', i, p, len(bottom))
    txt = f'''(symbol "{name}" (in_bom yes) (on_board yes)
  (property "Reference" "{ref}" (at 0 {hh + 3.81:.2f} 0) (effects (font (size 1.27 1.27))))
  (property "Value" "{value or name}" (at 0 {-hh - 3.81:.2f} 0) (effects (font (size 1.27 1.27))))
  (property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
  (property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
  (symbol "{name}_0_1" (rectangle (start {-hw:.2f} {hh:.2f}) (end {hw:.2f} {-hh:.2f}) (stroke (width 0.254) (type default)) (fill (type background))))
  (symbol "{name}_1_1" {" ".join(pins)}))'''
    return parse(txt)
