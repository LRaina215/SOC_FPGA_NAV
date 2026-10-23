"""Measure actual synthesized SLF row-buffer address loads and source delta."""
import argparse
from collections import Counter
import json
from pathlib import Path
from audit_pin_loads import INPUTS
from synthesis_connectivity import read


def inventory(build):
    prefix='u_nav/u_match/u_ex'
    cells=read(Path(build)/'impl/gwsynthesis/g2_top.vg',
               lambda n: n in ('u_nav','u_nav/u_match',prefix) or n.startswith(prefix+'/'))
    loads=Counter(net for c in cells.values() for pin,nets in c['ports'].items()
                  if pin in INPUTS for net in nets if net not in ('VCC','GND'))
    address={net for c in cells.values() if c['kind'].startswith('RAM16')
             for net in c['ports'].get('RAD',[])}
    return dict(primitive_counts=dict(Counter(c['kind'] for c in cells.values())),
                ram_address_net_count=len(address),
                max_direct_address_load=max((loads[n] for n in address),default=0),
                address_loads=sorted((dict(net=n,pins=loads[n]) for n in address),key=lambda x:(-x['pins'],x['net'])))


def compare(before,after):
    before,after=Path(before),Path(after)
    original=(before/'nav/rtl/match/slf_extract.v').read_text(encoding='utf-8')
    changed=(after/'nav/rtl/match/slf_extract.v').read_text(encoding='utf-8')
    assert changed.replace(' /* synthesis syn_maxfan=64 */','')==original,'source delta exceeds a single attribute'
    a,b=inventory(before),inventory(after)
    return dict(before_build=str(before),after_build=str(after),before=a,after=b,
                attribute_only=True,measured_fanout_improved=b['max_direct_address_load']<a['max_direct_address_load'],
                limit64_met=b['max_direct_address_load']<=64,
                scope='synthesis direct connectivity only; physical resources, routing and timing remain unverified')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('before',type=Path);p.add_argument('after',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    result=compare(a.before,a.after);a.out.write_text(json.dumps(result,indent=2),encoding='utf-8',newline='\n')
    print(json.dumps({key:value for key,value in result.items() if key not in ('before','after')}))
    print(json.dumps({k:{n:v for n,v in result[k].items() if n!='address_loads'} for k in ('before','after')}))
