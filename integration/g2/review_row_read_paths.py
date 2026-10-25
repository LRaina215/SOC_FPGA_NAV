"""Locate moved combinational logic in the two actual row-store netlists.

Cell counts are not nanosecond delays. An unconstrained input/output path is
not accepted just because the synthesis frequency table omits it.
"""
import argparse
from collections import Counter, defaultdict
from functools import lru_cache
import json
from pathlib import Path

from synthesis_connectivity import read
from audit_routing_topology import primitive_directions
from measure_slf_row_banks import LIBRARY


def analyze(path):
    cells=read(path);directions=primitive_directions(LIBRARY)
    drivers={};inputs={};counts=Counter()
    for name,c in cells.items():
        inputs[name]=[]
        for pin,nets in c['ports'].items():
            d=directions[c['kind']][pin]
            for net in nets:
                if net in ('VCC','GND'):continue
                counts[d]+=1
                if d=='output':
                    assert net not in drivers,(net,name,drivers.get(net))
                    drivers[net]=name
                if d=='input':inputs[name].append(net)

    @lru_cache(None)
    def trace(net,origin):
        if net in ('VCC','GND'):return None
        name=drivers.get(net)
        if name is None:return (0,[]) if origin=='external' else None
        c=cells[name];kind=c['kind']
        boundary='register' if kind.startswith('DFF') else 'memory' if kind.startswith('RAM16') else 'external' if kind=='IBUF' else None
        if boundary:return (0,[name]) if origin==boundary else None
        if kind in ('VCC','GND','GSR'):return None
        assert kind.startswith(('LUT','MUX')) or kind in ('INV','BUF','OBUF'),kind
        choices=[t for n in inputs[name] if (t:=trace(n,origin)) is not None]
        if not choices:return None
        best=max(choices,key=lambda x:x[0])
        return best[0]+1,best[1]+[name]

    results={}
    for destination in ('register_data','output'):
        for origin in ('memory','external','register'):
            candidates=[]
            for name,c in cells.items():
                selected=(c['kind'].startswith('DFF') and destination=='register_data') or (c['kind']=='OBUF' and destination=='output')
                if not selected:continue
                port='D' if destination=='register_data' else 'I'
                for net in c['ports'][port]:
                    t=trace(net,origin)
                    if t is not None:
                        candidates.append(dict(combinational_cell_levels=t[0],path=t[1]+[name],endpoint=name+'.'+port))
            results[origin+'_to_'+destination]=max(candidates,key=lambda x:x['combinational_cell_levels']) if candidates else None
    report=path.parent/'g2_top_syn.rpt.html'
    return dict(netlist=str(path),paths=results,explicit_pin_references=dict(counts),
                synthesis_report_no_timing_path='No timing path found' in report.read_text(encoding='utf-8'),
                scope='Actual combinational cells between explicit boundaries; not physical delay, geometry, route status, or a complete IO timing check')


def main():
    p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('candidate',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    result=dict(baseline=analyze(a.baseline),candidate=analyze(a.candidate),
                full_joint_acceptance=False,physical_delay_verified=False,
                decision='withhold single-register candidate: read selection moved before capture; no valid timing evidence for the new RAM-to-register path')
    a.out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
