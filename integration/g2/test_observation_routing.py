"""Actual-driver positive and negative checks for the one reset alias."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import re

from revalidate_observation_routing import validate,OLD,NEW,ROOT
from revalidate_lidar_cdc import definitions,module_at
from synthesis_connectivity import read


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    a.out.mkdir(parents=True,exist_ok=False)
    plan=json.loads((ROOT/'integration/g2/clock_resource_plan_lane2_obsreuse.json').read_text())
    source=Path(plan['target_source_build']);original=Path(plan['structural_driver_reference_build'])
    old=read(original/'impl/gwsynthesis/g2_top.vg');new=read(source/'impl/gwsynthesis/g2_top.vg')
    da=definitions(original/'impl/gwsynthesis/g2_top.vg');db=definitions(source/'impl/gwsynthesis/g2_top.vg')
    result=validate(source,plan,old,new,da,db)
    rows=[dict(case='actual12_targets4_clocks',passed=True)]
    def reject(name,cells=new,defs=db,config=plan):
        try:validate(source,config,old,cells,da,defs)
        except AssertionError as error:rows.append(dict(case=name,rejected=True,reason=str(error)))
        else:raise AssertionError('negative mutation accepted: '+name)
    damaged=dict(new);n='u_control/model_thread/observation/n971_s2'
    damaged[n]=deepcopy(new[n]);damaged[n]['ports']['I1']=['GND'];reject('changed_reset_input',cells=damaged)
    damaged=dict(db);body=module_at(db,'u_control/model_thread/observation')
    init=result['proved_alias']['init'];replacement="16'h"+format(int(init[4:],16)^1,'X')
    text,count=re.subn(r'(defparam\s+n971_s2\.INIT\s*=\s*)'+re.escape(init)+r'(;)',lambda m:m[1]+replacement+m[2],body)
    assert count==1
    kind,=[k for k,v in db.items() if v==body];damaged[kind]=text
    reject('changed_reset_lut_init',defs=damaged)
    driver=result['proved_alias']['upstream'][1]['driver']
    assert new[driver]['kind'].startswith('DFF') and 'CE' in new[driver]['ports']
    damaged=dict(new);damaged[driver]=deepcopy(new[driver]);damaged[driver]['ports']['CE']=['GND']
    reject('changed_upstream_register_enable',cells=damaged)
    damaged=dict(new);name=result['targets'][0]['new_driver'];damaged[name]=deepcopy(new[name])
    damaged[name]['ports']['I0']=['GND'];reject('changed_other_routing_driver',cells=damaged)
    bad=deepcopy(plan);bad['reference_net_aliases'][NEW]='r2';reject('unreviewed_reference_alias',config=bad)
    bad=deepcopy(plan);bad['target_source_build']=str(original);reject('wrong_target_source',config=bad)
    bad=deepcopy(plan);bad['local'].append(NEW);reject('routing_choice_change',config=bad)
    (a.out/'result.json').write_text(json.dumps(dict(passed=True,cases=rows,actual_proof=result),indent=2)+'\n')
    print(json.dumps(dict(passed=True,cases=len(rows),init=init,evidence=str(a.out/'result.json'))))


if __name__=='__main__':main()
