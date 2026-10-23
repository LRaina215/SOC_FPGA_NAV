"""Inventory frozen connectivity to choose bounded routing diagnostics.

This reports structural pressure, never a list of failed routes or measured
congestion. Primitive directions come from the installed device simulation
library, not a guessed list of input pin names.
"""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import re

from audit_netlist_memories import BSRAM
from audit_netlist_memories import MODULE
from synthesis_connectivity import read, DECL


def primitive_directions(library):
    text = Path(library).read_text(encoding='utf-8')
    text = re.sub(r'/\*.*?\*/|//[^\n]*', '', text, flags=re.S)
    result = {}
    for match in re.finditer(r'\bmodule\s+(\w+)\b(.*?)\bendmodule', text, re.S):
        name, body = match.groups()
        directions = {}
        # Conventional declarations, including multiple names per statement.
        for d, names in re.findall(
                r'\b(input|output|inout)\s+(?:(?:wire|reg|signed)\s+)*'
                r'(?:\[[^\]]+\]\s*)?([\w\s,]+);', body):
            for pin in names.split(','):
                pin = pin.strip()
                if re.fullmatch(r'\w+', pin):
                    directions[pin] = d
        # ANSI declarations in the GTR interface.
        header = body.split(');', 1)[0]
        for d, pin in re.findall(
                r'\b(input|output|inout)\s+(?:(?:wire|reg|signed)\s+)*'
                r'(?:\[[^\]]+\]\s*)?(\w+)', header):
            directions[pin] = d
        if directions:
            result[name] = directions
    return result


def group(name, depth=2):
    parts = name.split('/')
    return '/'.join(parts[:min(depth, len(parts)-1)]) or '(top)'


def audit(netlist, library):
    cells = read(netlist)
    top = dict(MODULE.findall(Path(netlist).read_text(encoding='utf-8')))['g2_top']
    external_inputs = set()
    for direction, hi, lo, name in DECL.findall(top):
        if direction not in ('input', 'inout'):
            continue
        name = name.lstrip('\\')
        external_inputs.update([name] if not hi else
                               (name+'['+str(i)+']' for i in range(int(lo), int(hi)+1)))
    directions = primitive_directions(library)
    unknown = Counter((c['kind'], pin) for c in cells.values()
                      for pin in c['ports']
                      if pin not in directions.get(c['kind'], {}))
    assert not unknown, ('unclassified primitive ports', unknown)
    drivers, sinks = defaultdict(list), defaultdict(list)
    modules = defaultdict(Counter)
    for name, cell in cells.items():
        modules[group(name)][cell['kind']] += 1
        for pin, nets in cell['ports'].items():
            d = directions[cell['kind']][pin]
            for bit, net in enumerate(nets):
                if net in ('VCC', 'GND'):
                    continue
                endpoint = dict(cell=name, kind=cell['kind'], pin=pin, bit=bit)
                if d in ('input', 'inout'):
                    sinks[net].append(endpoint)
                if d in ('output', 'inout'):
                    drivers[net].append(endpoint)
    rows, crossings = [], Counter()
    for net, loads in sinks.items():
        pin_counts = Counter(x['pin'] for x in loads)
        domains = Counter(group(x['cell']) for x in loads)
        sources = drivers.get(net, [])
        # Dedicated clocks are recorded separately; they dominate the list.
        clock = any(p in ('CLK', 'CLKA', 'CLKB', 'CLOCK') for p in pin_counts)
        row = dict(net=net, sink_bits=len(loads), pins=dict(pin_counts),
                   sink_modules=dict(domains), drivers=sources,
                   sink_examples=loads[:4], clock_pin_load=clock)
        rows.append(row)
        if not clock and len(sources) == 1:
            source_group = group(sources[0]['cell'])
            for target in domains:
                if target != source_group:
                    crossings[(source_group, target)] += 1
    rows.sort(key=lambda x: (-x['sink_bits'], x['net']))
    primitives = Counter(c['kind'] for c in cells.values())
    return dict(
        netlist=str(Path(netlist).resolve()), primitive_library=str(Path(library).resolve()),
        cells=len(cells), primitive_counts=dict(primitives),
        unclassified_ports=[],
        modules=[dict(path=p, cells=sum(v.values()), bsram=sum(v[t] for t in BSRAM),
                      ram16=sum(n for t, n in v.items() if t.startswith('RAM16')),
                      mux=sum(n for t, n in v.items() if t.startswith('MUX')),
                      ff=sum(n for t, n in v.items() if t.startswith('DFF')),
                      primitives=dict(v))
                 for p, v in sorted(modules.items(), key=lambda x: -sum(x[1].values()))],
        high_fanout_nonclock=[x for x in rows if not x['clock_pin_load']][:80],
        clocks=[x for x in rows if x['clock_pin_load']][:12],
        module_crossings=[dict(source=a, target=b, distinct_net_bits=n)
                          for (a, b), n in crossings.most_common(50)],
        unclassified_driver_nets=sum(not drivers.get(n) for n in sinks),
        external_input_driver_nets=sum(n in external_inputs and not drivers.get(n) for n in sinks),
        unresolved_internal_driver_nets=[dict(net=n, sink_example=v[0])
                                        for n, v in sinks.items()
                                        if n not in external_inputs and not drivers.get(n)],
        multiple_driver_nets=sum(len(v) > 1 for v in drivers.values()),
        failed_net_list_available=False, physical_locations_available=False,
        scope='expanded explicit SYN connectivity; protected macro internals remain opaque; '
              'module crossings and loads are not route length, congestion or failure attribution')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('netlist', type=Path)
    p.add_argument('--library', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    r = audit(a.netlist, a.library)
    a.out.write_text(json.dumps(r, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps(dict(cells=r['cells'], modules=r['modules'][:8],
                          top_loads=r['high_fanout_nonclock'][:8],
                          crossings=r['module_crossings'][:10])))


if __name__ == '__main__':
    main()
