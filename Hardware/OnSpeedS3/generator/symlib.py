"""Minimal KiCad 7 .kicad_sym reader: s-expression parse, symbol lookup (with extends), pin list."""
import re, os
LIB = "/usr/share/kicad/symbols"
def parse(text):
    tok = re.findall(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()]+', text)
    stack = [[]]
    for t in tok:
        if t == '(':
            stack.append([])
        elif t == ')':
            x = stack.pop(); stack[-1].append(x)
        else:
            stack[-1].append(t)
    return stack[0][0]
_cache = {}
def lib(name):
    if name not in _cache:
        _cache[name] = parse(open(os.path.join(LIB, name + ".kicad_sym")).read())
    return _cache[name]
def unq(s): return s[1:-1] if isinstance(s, str) and s.startswith('"') else s
def find_symbol(libname, sym):
    L = lib(libname)
    for e in L:
        if isinstance(e, list) and e[0] == 'symbol' and unq(e[1]) == sym:
            return e
    raise KeyError(f"{libname}:{sym}")
def pins(libname, sym):
    s = find_symbol(libname, sym)
    ext = [e for e in s if isinstance(e, list) and e[0] == 'extends']
    if ext: return pins(libname, unq(ext[0][1]))
    return pins_of(s)
def pins_of(s):
    out = []
    def walk(n):
        for e in n:
            if isinstance(e, list):
                if e[0] == 'pin':
                    at = [x for x in e if isinstance(x, list) and x[0] == 'at'][0]
                    nm = [x for x in e if isinstance(x, list) and x[0] == 'name'][0]
                    nb = [x for x in e if isinstance(x, list) and x[0] == 'number'][0]
                    ln = [x for x in e if isinstance(x, list) and x[0] == 'length']
                    out.append(dict(type=e[1], number=unq(nb[1]), name=unq(nm[1]), x=float(at[1]), y=float(at[2]), rot=float(at[3]) if len(at) > 3 else 0.0,
                                    length=float(ln[0][1]) if ln else 2.54, hidden='hide' in e))
                else:
                    walk(e)
    walk(s)
    return out
if __name__ == "__main__":
    import sys
    for p in pins(sys.argv[1], sys.argv[2]):
        print(p['number'], p['name'], p['type'], p['x'], p['y'], p['rot'], 'H' if p['hidden'] else '')
