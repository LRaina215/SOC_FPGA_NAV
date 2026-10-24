"""Bind the original twelve routing choices to the equivalent observation SYN.

Only the observed n331_7 -> n971_7 reset-LUT alias is permitted. The LUT
inputs, INIT and their unique upstream drivers must match; other targets
retain exact cells and parameters. This proves connectivity, not PNR timing.
"""
import json
from pathlib import Path

from prove_row_address_alias import parameters as nested_parameters
from audit_netlist_memories import PARAM
from revalidate_lidar_cdc import definitions
from synthesis_connectivity import read
from audit_routing_topology import primitive_directions

ROOT=Path(__file__).resolve().parents[2]
LIBRARY=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
OLD='u_control/model_thread/observation/n331_7'
NEW='u_control/model_thread/observation/n971_7'


def parameters(cell,modules):
    if '/' in cell['name']:return nested_parameters(cell,modules)
    pairs=[(k,v.strip()) for n,k,v in PARAM.findall(modules['g2_top'])
           if n.strip().lstrip('\\')==cell['name']]
    assert len(pairs)==len(dict(pairs)), 'duplicate top primitive parameter'
    return dict(pairs)


def driver_index(cells,directions):
    result={}
    for cell in cells.values():
        for pin,nets in cell['ports'].items():
            if directions[cell['kind']][pin]=='output':
                for net in nets:result.setdefault(net,[]).append(cell)
    return result


def prove_alias(before,after,a,b,old_defs,new_defs):
    old,new=a[OLD],b[NEW]
    assert len(old)==len(new)==1, 'observation reset driver not unique'
    old,new=old[0],new[0]
    assert old['kind']==new['kind']=='LUT4'
    assert old['name']=='u_control/model_thread/observation/n331_s2'
    assert new['name']=='u_control/model_thread/observation/n971_s2'
    inputs={'I0':['u_control/model_status_flags_1[6]'],'I1':['r2'],
            'I2':['u_control/n6_4'],'I3':['u_control/n6_5']}
    assert old['ports']==dict(F=[OLD],**inputs)
    assert new['ports']==dict(F=[NEW],**inputs)
    lhs,rhs=parameters(old,old_defs),parameters(new,new_defs)
    assert lhs==rhs and len(lhs)==1 and 'INIT' in lhs, 'reset LUT function changed'
    init=lhs['INIT']
    assert init.startswith("16'h")
    truth=[(int(init[4:],16)>>i)&1 for i in range(16)]
    upstream=[]
    for pin,nets in inputs.items():
        net,=nets
        assert len(a[net])==len(b[net])==1, 'observation reset upstream driver not unique'
        x,y=a[net][0],b[net][0]
        assert x==y, ('observation reset upstream inputs/clock/enable changed',net)
        assert parameters(x,old_defs)==parameters(y,new_defs), ('upstream INIT changed',net)
        upstream.append(dict(pin=pin,net=net,driver=x['name'],kind=x['kind'],
                             inputs_clock_enable_and_parameters_identical=True))
    return dict(old_net=OLD,new_net=NEW,old_driver=old['name'],new_driver=new['name'],
                kind='LUT4',init=init,truth_table_16=truth,upstream=upstream,
                outputs_only_renamed=True,scope='one named reset-LUT alias; no timing exception added')


def validate(source,plan,before=None,after=None,old_defs=None,new_defs=None):
    source=Path(source)
    assert Path(plan['target_source_build']).resolve()==source.resolve(), 'wrong target SYN'
    original=Path(plan['structural_driver_reference_build'])
    assert original.resolve()==Path(json.loads((source/'manifest.json').read_text())['frozen_source_build']).resolve()
    assert plan.get('reference_net_aliases')=={NEW:OLD}, 'unreviewed routing alias'
    template=json.loads((ROOT/'integration/g2/clock_resource_plan_lane2_telemetry.json').read_text())
    translate=lambda n:plan['reference_net_aliases'].get(n,n)
    for group in ('promote','prefer_global'):
        normalized=[dict(x,net=translate(x['net'])) for x in plan[group]]
        assert normalized==template[group], ('routing actions changed beyond the proved alias',group)
    assert plan['local']==template['local'], 'LOCAL choices changed'
    physical=Path(plan['reference_physical_build'])
    assert physical.name=='sfp_lane2_telemetryglobal_pnr10', 'wrong physical comparison'
    assert (physical/'g2_top.vg').read_bytes()==(original/'impl/gwsynthesis/g2_top.vg').read_bytes(), 'physical source differs from old SYN'
    before=read(original/'impl/gwsynthesis/g2_top.vg') if before is None else before
    after=read(source/'impl/gwsynthesis/g2_top.vg') if after is None else after
    old_defs=definitions(original/'impl/gwsynthesis/g2_top.vg') if old_defs is None else old_defs
    new_defs=definitions(source/'impl/gwsynthesis/g2_top.vg') if new_defs is None else new_defs
    directions=primitive_directions(LIBRARY)
    a,b=driver_index(before,directions),driver_index(after,directions)
    alias=prove_alias(before,after,a,b,old_defs,new_defs)
    names=[x['net'] for x in plan['promote']+plan['prefer_global']]+plan['local']
    assert len(names)==len(set(names))==12
    rows=[]
    for net in names:
        prior=translate(net)
        assert len(a.get(prior,[]))==len(b.get(net,[]))==1, ('routing source missing/nonunique',net)
        x,y=a[prior][0],b[net][0]
        if net!=NEW:
            assert x==y, ('routing driver inputs changed',net)
            assert parameters(x,old_defs)==parameters(y,new_defs), ('routing driver INIT changed',net)
        rows.append(dict(net=net,reference_net=prior,old_driver=x['name'],new_driver=y['name'],
                         parameters_identical=True,proved_alias=net==NEW))
    clocks=[]
    for net in ('clk50_d','rgmii_rxc_d','h_rx_clk','h_tx_clk'):
        assert len(a.get(net,[]))==len(b.get(net,[]))==1, ('clock driver missing/nonunique',net)
        x,y=a[net][0],b[net][0]
        assert x==y, ('true clock source changed',net)
        assert parameters(x,old_defs)==parameters(y,new_defs)
        clocks.append(dict(net=net,driver=y['name'],kind=y['kind'],inputs_and_parameters_identical=True))
    return dict(passed=True,reference_physical_build=str(physical),structural_reference=str(original),
                target_source=str(source),targets=rows,strict_targets=11,proved_alias=alias,
                true_clock_drivers=clocks,clock_periods_and_routing_choices_unchanged=True,
                scope='actual drivers, INIT and one proved alias; actual allocation/CDC delay/timing still require PNR')
