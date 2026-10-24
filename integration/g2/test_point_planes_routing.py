"""Reject real driver/clock/INIT mutations before launching a point-store PNR."""
from copy import deepcopy
from revalidate_point_planes_routing import validate
from revalidate_lidar_cdc import module_at
from audit_netlist_memories import PARAM


def check(source,plan,old,new,da,db):
    actual=validate(source,plan,old,new,da,db)
    rows=[dict(case=f'actual{len(actual["targets"])}_drivers_and4_clocks',passed=True)]
    def reject(label,cells=new,defs=db,config=plan):
        try:validate(source,config,old,cells,da,defs)
        except AssertionError as error:rows.append(dict(case=label,rejected=True,reason=str(error)))
        else:raise AssertionError('negative driver mutation accepted: '+label)
    n=actual['targets'][0]['driver'];damaged=dict(new);damaged[n]=deepcopy(new[n])
    pin=next(p for p in new[n]['ports'] if p.startswith('I'))
    damaged[n]['ports'][pin]=['VCC' if new[n]['ports'][pin]==['GND'] else 'GND']
    reject('changed_control_driver_input',cells=damaged)
    clock=actual['true_clock_drivers'][2]['driver'];damaged=dict(new);damaged[clock]=deepcopy(new[clock])
    pin=next(p for p,v in new[clock]['ports'].items() if v==['h_rx_clk'])
    damaged[clock]['ports'][pin]=['GND'];reject('changed_lane2_receive_clock',cells=damaged)
    reset=actual['targets'][2]['driver'];hierarchy,instance=reset.rsplit('/',1)
    body=module_at(db,hierarchy);kind,=[k for k,v in db.items() if v==body]
    found=[(i,k,v) for i,k,v in PARAM.findall(body) if i.strip().lstrip('\\')==instance and k=='INIT']
    assert len(found)==1
    _,_,value=found[0];assert value.startswith("16'h")
    badvalue="16'h"+format(int(value[4:],16)^1,'X')
    import re
    changed,count=re.subn(r'(defparam\s+'+re.escape(instance)+r'\s*\.INIT\s*=\s*)'+re.escape(value)+r'(;)',
                         lambda m:m[1]+badvalue+m[2],body)
    assert count==1
    damaged=dict(db);damaged[kind]=changed;reject('changed_control_driver_init',defs=damaged)
    bad=deepcopy(plan);bad['reference_net_aliases']={'r2':'n2371_7'};reject('unreviewed_alias',config=bad)
    bad=deepcopy(plan);bad['promote'][0]['primary_index']=7;reject('changed_primary_assignment',config=bad)
    bad=deepcopy(plan);bad['target_source_build']=str(source.parent/'wrong_source');reject('wrong_frozen_source',config=bad)
    if plan.get('observation_lw_reallocation'):
        # The old twelve-target plan must still work; the new permission is
        # limited to the two observed nets and their exact actual drivers.
        inherited=deepcopy(plan);inherited.pop('observation_lw_reallocation');inherited.pop('observation_lw_reference_build')
        inherited['prefer_global'].pop();inherited['local'].pop()
        assert validate(source,inherited,old,new,da,db)['strict_targets']==12
        rows.append(dict(case='inherited12_still_strict',passed=True))
        from revalidate_point_planes_routing import OBSERVATION_UPDATE
        row,=[r for r in actual['targets'] if r['net']==OBSERVATION_UPDATE]
        n=row['driver'];damaged=dict(new);damaged[n]=deepcopy(new[n])
        pin=next(p for p in new[n]['ports'] if p.startswith('I'))
        damaged[n]['ports'][pin]=['VCC' if new[n]['ports'][pin]==['GND'] else 'GND']
        reject('changed_observation_update_driver',cells=damaged)
        bad=deepcopy(plan);bad['local'][-1]='u_nav/u_pf/n3967_6';reject('unreviewed_small_local_choice',config=bad)
        bad=deepcopy(plan);bad['observation_lw_reference_build']=str(source.parent/'sfp_lane2_pointplanes_pnr13')
        reject('wrong_observation_allocation_reference',config=bad)
    return dict(passed=True,cases=rows)
