"""Verify the twelve inherited choices and one explicitly measured LW exchange."""
import json
from pathlib import Path

from audit_routing_topology import primitive_directions
from revalidate_lidar_cdc import definitions
from revalidate_observation_routing import LIBRARY, ROOT, driver_index, parameters
from synthesis_connectivity import read


OBSERVATION_UPDATE='u_control/model_thread/observation/n12472_10'
SMALL_PROJECT_CE='u_nav/u_pt/n5285_4'


def observation_lw_evidence(source,plan):
    """Bind this intervention to the completed 377-net run, not guessed names."""
    physical=Path(plan['observation_lw_reference_build'])
    assert physical.name=='sfp_lane2_private_snapshot_pnr14', 'wrong observation allocation reference'
    assert (physical/'g2_top.vg').read_bytes()==(source/'impl/gwsynthesis/g2_top.vg').read_bytes(), 'LW exchange must reuse the exact PNR14 SYN bytes'
    gate=json.loads((physical/'g2_gate.json').read_text())
    assert gate['unrouted_nets']==377 and gate['bsram']['used']==326
    globals=json.loads((physical/'global_loads.json').read_text())['global_nets']
    by_net={r['net']:r for r in globals}
    assert OBSERVATION_UPDATE not in by_net, 'observation update was not displaced'
    small=by_net[SMALL_PROJECT_CE]
    assert small['resource']=='LW' and small['direct_pin_counts']=={'CE':308}
    pins=json.loads((physical/'pin_loads.json').read_text())['top_direct_loads']
    observation,=[r for r in pins if r['net']==OBSERVATION_UPDATE]
    assert observation['pins']==3893 and observation['port_counts']=={'D':1,'CE':3892}
    return dict(reference_build=str(physical),exact_source_bytes=True,
                requested_global=observation,requested_local=small,
                failed_net_identities_available=False,
                scope='one measured LW allocation exchange; not a cause assignment for 377 failed nets')


def validate(source,plan,before=None,after=None,old_defs=None,new_defs=None):
    source=Path(source);manifest=json.loads((source/'manifest.json').read_text())
    original=Path(manifest['frozen_source_build']);physical=Path(plan['reference_physical_build'])
    assert manifest.get('slf_point_planes') and Path(plan['target_source_build']).resolve()==source.resolve()
    assert Path(plan['structural_driver_reference_build']).resolve()==original.resolve()
    assert not plan.get('reference_net_aliases'), 'new aliases require a separate structural proof'
    template=json.loads((ROOT/'integration/g2/clock_resource_plan_lane2_obsreuse.json').read_text())
    exchange=plan.get('observation_lw_reallocation',False)
    allocation_evidence=None
    if exchange:
        assert manifest.get('snapshot_private_reset'), 'LW exchange is bound to the measured private-snapshot source'
        template['prefer_global'].append(dict(net=OBSERVATION_UPDATE,signal_types='CE|LOGIC'))
        template['local'].append(SMALL_PROJECT_CE)
        allocation_evidence=observation_lw_evidence(source,plan)
    for group in ('promote','prefer_global','local'):
        assert plan[group]==template[group], ('routing choices changed',group)
    expected='sfp_lane2_pointplanes_pnr13' if manifest.get('snapshot_private_reset') else 'sfp_lane2_obsreuse_pnr12'
    assert physical.name==expected
    assert (physical/'g2_top.vg').read_bytes()==(original/'impl/gwsynthesis/g2_top.vg').read_bytes()
    before=read(original/'impl/gwsynthesis/g2_top.vg') if before is None else before
    after=read(source/'impl/gwsynthesis/g2_top.vg') if after is None else after
    old_defs=definitions(original/'impl/gwsynthesis/g2_top.vg') if old_defs is None else old_defs
    new_defs=definitions(source/'impl/gwsynthesis/g2_top.vg') if new_defs is None else new_defs
    directions=primitive_directions(LIBRARY)
    a,b=driver_index(before,directions),driver_index(after,directions)
    names=[x['net'] for x in plan['promote']+plan['prefer_global']]+plan['local']
    count=14 if exchange else 12
    assert len(names)==len(set(names))==count
    rows=[]
    for net in names+['clk50_d','rgmii_rxc_d','h_rx_clk','h_tx_clk']:
        assert len(a.get(net,[]))==len(b.get(net,[]))==1, ('driver missing or nonunique',net)
        x,y=a[net][0],b[net][0]
        assert x==y, ('driver input/clock/enable/reset changed',net,x,y)
        assert parameters(x,old_defs)==parameters(y,new_defs), ('driver parameters/INIT changed',net)
        rows.append(dict(net=net,driver=y['name'],kind=y['kind'],inputs_and_parameters_identical=True))
    return dict(passed=True,reference_physical_build=str(physical),structural_reference=str(original),
                targets=rows[:count],true_clock_drivers=rows[count:],strict_targets=count,aliases_allowed=0,
                observation_lw_exchange_evidence=allocation_evidence,
                scope='driver/INIT identity only; actual allocation, CDC delay and timing require PNR')
