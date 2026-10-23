"""Inventory direct cross-clock DFF Q->D edges in a flat physical netlist.

Reports, but does not exempt, first stages that branch into combinational logic.
Does not cover combinational source cones, memory buses, resets, IO or MTBF.
"""
import argparse,json
from collections import defaultdict
from pathlib import Path
from audit_pin_loads import CELLS,PORTS,INPUTS
from audit_cdc_first_stages import bits


def inventory(path):
    cells={n.lstrip('\\'):dict(kind=k,ports={p:bits(e)for p,e in PORTS.findall(b)})
           for k,n,b in CELLS.findall(path.read_text(encoding='utf-8'))}
    drivers=defaultdict(list);loads=defaultdict(list)
    for name,c in cells.items():
        for pin,nets in c['ports'].items():
            if c['kind'].startswith('DFF') and pin=='Q':
                for net in nets:drivers[net].append(name)
            if pin in INPUTS:
                for net in nets:loads[net].append([name,pin])
    rows=[]
    for name,c in cells.items():
        if not c['kind'].startswith('DFF') or 'CLK'not in c['ports']:continue
        for net in c['ports'].get('D',[]):
            for source in drivers[net]:
                src=cells[source]['ports'];dst=c['ports']
                if src['CLK']==dst['CLK']:continue
                fanout=loads[dst['Q'][0]]
                second=cells[fanout[0][0]] if len(fanout)==1 and fanout[0][1]=='D' else None
                single_second=bool(second and second['kind'].startswith('DFF') and second['ports']['CLK']==dst['CLK'])
                rows.append(dict(source=source,first=name,source_clock=src['CLK'],destination_clock=dst['CLK'],
                    first_q_loads=fanout,single_second_register=single_second))
    return dict(netlist=str(path.resolve()),direct_crossings=len(rows),paths=rows,
        scope='direct Q to D inventory only; does not establish data coherence, CDC completeness or approve exceptions')


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('netlist',type=Path);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();r=inventory(a.netlist);a.out.write_text(json.dumps(r,indent=2),encoding='utf-8')
    print(json.dumps(dict(direct_crossings=len(r['paths']),single_second_register=sum(p['single_second_register']for p in r['paths']))))
