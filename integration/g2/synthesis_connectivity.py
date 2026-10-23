"""Read explicit Gowin structural Verilog, resolving hierarchy port aliases.

This is a connectivity reader, not a Verilog simulator or timing analyzer.
Reject assigns/unsupported expressions instead of silently guessing aliases.
"""
import re
from pathlib import Path
from audit_netlist_memories import MODULE

CELL = re.compile(r'^\s{2}(\w+)\s+(\\\S+|[^\s(]+)\s*\(\s*\n(.*?)^\);', re.M|re.S)
PORT = re.compile(r'\.([A-Za-z_][A-Za-z0-9_]*)\(([^()]*)\)')
DECL = re.compile(r'^\s*(input|output|inout|wire)\s+(?:\[(\d+):(\d+)\]\s+)?(\\\S+|\w+)\s*;', re.M)
TOKEN = re.compile(r"\s*(?:(\d+)'([bhd])([0-9a-fA-FxXzZ]+)|(\\[^\s,{}]+|[A-Za-z_][A-Za-z0-9_$]*)(?:\s*\[(\d+)(?::(\d+))?\])?|([{},]))")


def read(path, keep=lambda hierarchy: True):
    modules = dict(MODULE.findall(Path(path).read_text(encoding='utf-8')))
    cells = {}; cache = {}
    def definition(kind):
        if kind not in cache:
            body = modules[kind]
            assert not re.search(r'^\s*assign\s', body, re.M), 'assign requires explicit alias support'
            declarations = {n.lstrip('\\'): (d, int(hi) if hi else None, int(lo) if lo else None)
                            for d, hi, lo, n in DECL.findall(body)}
            cache[kind] = declarations, [(t, n.lstrip('\\'), dict(PORT.findall(p))) for t,n,p in CELL.findall(body)]
        return cache[kind]
    def walk(kind, hierarchy, bindings):
        declarations, instances = definition(kind)
        def local(name):
            if name in ('VCC','GND'): return name
            return bindings.get(name, (hierarchy+'/' if hierarchy else '')+name)
        def indices(hi,lo): return range(hi,lo+(1 if lo>=hi else -1),1 if lo>=hi else -1)
        def expand(expr):
            result=[]; position=0
            while position<len(expr):
                if not expr[position:].strip():break
                m=TOKEN.match(expr,position)
                assert m, ('unsupported net expression',kind,hierarchy,expr[position:])
                position=m.end(); width,base,value,name,hi,lo,punct=m.groups()
                if punct:continue
                if width:
                    assert not re.search('[xXzZ]',value), ('unknown constant',expr)
                    number=int(value,{'b':2,'h':16,'d':10}[base])
                    result += ['VCC' if number>>i&1 else 'GND' for i in range(int(width)-1,-1,-1)]
                else:
                    name=name.lstrip('\\')
                    if hi is not None: result += [local(f'{name}[{i}]') for i in indices(int(hi),int(lo or hi))]
                    elif name in declarations and declarations[name][1] is not None:
                        result += [local(f'{name}[{i}]') for i in indices(*declarations[name][1:])]
                    else:result.append(local(name))
            return result
        for child,instance,ports in instances:
            full=(hierarchy+'/'+instance).strip('/')
            if not keep(full):continue
            connections={p:expand(e) for p,e in ports.items()}
            if child in modules:
                child_declarations,_=definition(child); mapped={}
                for p,nets in connections.items():
                    d,hi,lo=child_declarations[p]
                    names=[p] if hi is None else [f'{p}[{i}]' for i in indices(hi,lo)]
                    assert len(names)==len(nets),('port width mismatch',full,p,names,nets)
                    mapped.update(zip(names,nets))
                walk(child,full,mapped)
            else:cells[full]=dict(name=full,kind=child,ports=connections)
    walk('g2_top','',{})
    return cells
