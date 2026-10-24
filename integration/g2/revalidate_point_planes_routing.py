"""Keep all twelve measured routing choices when only point storage changes."""
import json
from pathlib import Path

from audit_routing_topology import primitive_directions
from revalidate_lidar_cdc import definitions
from revalidate_observation_routing import LIBRARY, ROOT, driver_index, parameters
from synthesis_connectivity import read


def validate(source,plan,before=None,after=None,old_defs=None,new_defs=None):
    source=Path(source);manifest=json.loads((source/'manifest.json').read_text())
    original=Path(manifest['frozen_source_build']);physical=Path(plan['reference_physical_build'])
    assert manifest.get('slf_point_planes') and Path(plan['target_source_build']).resolve()==source.resolve()
    assert Path(plan['structural_driver_reference_build']).resolve()==original.resolve()
    assert not plan.get('reference_net_aliases'), 'new aliases require a separate structural proof'
    template=json.loads((ROOT/'integration/g2/clock_resource_plan_lane2_obsreuse.json').read_text())
    for group in ('promote','prefer_global','local'):
        assert plan[group]==template[group], ('routing choices changed',group)
    assert physical.name=='sfp_lane2_obsreuse_pnr12'
    assert (physical/'g2_top.vg').read_bytes()==(original/'impl/gwsynthesis/g2_top.vg').read_bytes()
    before=read(original/'impl/gwsynthesis/g2_top.vg') if before is None else before
    after=read(source/'impl/gwsynthesis/g2_top.vg') if after is None else after
    old_defs=definitions(original/'impl/gwsynthesis/g2_top.vg') if old_defs is None else old_defs
    new_defs=definitions(source/'impl/gwsynthesis/g2_top.vg') if new_defs is None else new_defs
    directions=primitive_directions(LIBRARY)
    a,b=driver_index(before,directions),driver_index(after,directions)
    names=[x['net'] for x in plan['promote']+plan['prefer_global']]+plan['local']
    assert len(names)==len(set(names))==12
    rows=[]
    for net in names+['clk50_d','rgmii_rxc_d','h_rx_clk','h_tx_clk']:
        assert len(a.get(net,[]))==len(b.get(net,[]))==1, ('driver missing or nonunique',net)
        x,y=a[net][0],b[net][0]
        assert x==y, ('driver input/clock/enable/reset changed',net,x,y)
        assert parameters(x,old_defs)==parameters(y,new_defs), ('driver parameters/INIT changed',net)
        rows.append(dict(net=net,driver=y['name'],kind=y['kind'],inputs_and_parameters_identical=True))
    return dict(passed=True,reference_physical_build=str(physical),structural_reference=str(original),
                targets=rows[:12],true_clock_drivers=rows[12:],strict_targets=12,aliases_allowed=0,
                scope='driver/INIT identity only; actual allocation, CDC delay and timing require PNR')
