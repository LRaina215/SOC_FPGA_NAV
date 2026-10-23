"""Actual frozen address-LUT aliases, including rejecting altered behavior."""
from copy import deepcopy
from pathlib import Path
import json
import re

from audit_routing_topology import primitive_directions
from prove_row_address_alias import PREFIX,prove
from revalidate_lidar_cdc import definitions,module_at
from synthesis_connectivity import read


def main():
    base = Path('F:/nav_integration_g2_20261023')
    paths = [base/'sfp_clockpriority_pnr4/g2_top.vg',base/'sfp_lane2_rowbanks_syn7/impl/gwsynthesis/g2_top.vg']
    keep = lambda n:n in ('u_nav','u_nav/u_match',PREFIX.rstrip('/')) or n.startswith(PREFIX)
    before,after = [read(p,keep) for p in paths]
    old_defs,new_defs = [definitions(p) for p in paths]
    directions = primitive_directions(Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v'))
    def drivers(cells,net):
        return [c for c in cells.values() for pin,nets in c['ports'].items()
                if directions[c['kind']][pin] == 'output' and net in nets]
    rows=[]
    for bit in (0,1):
        net=PREFIX+f'rb_ra[{bit}]';old,new=drivers(before,net),drivers(after,net)
        proof=prove(net,old,new,before,after,old_defs,new_defs)
        rows.append(dict(case=f'actual_alias_bit{bit}',passed=True,proof=proof))
    net=PREFIX+'rb_ra[0]';old,new=drivers(before,net),drivers(after,net)
    def reject(name,data=after,defs=new_defs,target=net):
        try:prove(target,old,drivers(data,net),before,data,old_defs,defs)
        except AssertionError as error:rows.append(dict(case=name,rejected=True,reason=str(error)))
        else:raise AssertionError('negative case accepted: '+name)
    damaged=deepcopy(after);damaged[new[0]['name']]['ports']['I2']=['GND'];reject('wrong_lut_input',data=damaged)
    damaged=deepcopy(after);damaged[PREFIX+'u_0_s0']['ports']['CE']=['GND'];reject('changed_upstream_enable',data=damaged)
    damaged=deepcopy(new_defs);body=module_at(damaged,PREFIX.rstrip('/'))
    local=new[0]['name'].rsplit('/',1)[1]
    pattern=r'(defparam\s+'+re.escape(local)+r'\.INIT\s*=\s*)8\x27hCA(;)'
    changed,n=re.subn(pattern,lambda m:m[1]+"8'hCB"+m[2],body)
    assert n==1,'actual LUT INIT parameter not found'
    kind=next(k for k,v in damaged.items() if v == body);damaged[kind]=changed
    reject('changed_lut_init',defs=damaged)
    reject('unreviewed_alias_target',target=PREFIX+'rb_ra[2]')
    out=base/'lane2_rowbanks_one_shot/row_address_alias_tests.json'
    out.write_text(json.dumps(dict(passed=True,cases=rows),indent=2),encoding='utf-8',newline='\n')
    print(json.dumps(dict(passed=True,cases=len(rows),evidence=str(out))))


if __name__=='__main__':main()
