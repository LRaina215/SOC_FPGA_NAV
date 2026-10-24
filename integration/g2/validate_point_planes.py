"""Full-source, memory geometry and clock-domain checks for point-plane PNR."""
from collections import Counter
import json
from pathlib import Path

from audit_routing_topology import primitive_directions
from audit_sfp_cdc import audit as audit_sfp
from prepare_slf_point_planes import transform
from prepare_sfp_lane2 import verify_frozen,verify_synthesized_clock_pins
from revalidate_lidar_cdc import definitions,revalidate
from revalidate_point_planes_routing import validate as routing,LIBRARY
from revalidate_observation_routing import parameters
from synthesis_connectivity import read

POINT='g2_top/u_nav/u_match/u_pb/'


def validate(source,baseline,plan_file):
    from continue_bank_select_once import bank_load
    source,baseline=Path(source),Path(baseline)
    manifest=json.loads((source/'manifest.json').read_text());job=json.loads((source/'job.json').read_text())
    original=Path(manifest['frozen_source_build'])
    assert job['phase']=='synthesis_complete' and job['tool_exit']==0
    for flag in ('slf_point_planes','observation_reuse_prefetch','slf_row_banks','bank_select_data_fanout',
                 'real_sfp','full_control_telemetry'):
        assert manifest[flag],flag
    assert not manifest['control_weights_changed'] and not manifest.get('bias_block_rom')
    assert manifest['clock_core_mhz']==50
    ram='nav/rtl/common/nav_ram_sdp.v'
    assert (source/ram).read_text()==transform((original/ram).read_text())
    changed=[Path(n).relative_to(source).as_posix() for n in manifest['source_files']
             if Path(n).read_bytes()!=(original/Path(n).relative_to(source)).read_bytes()]
    assert changed==job['modified_rtl_inputs']==[ram]
    def assets(root):
        return {p.relative_to(root):p for p in root.rglob('*') if p.is_file() and p.suffix.lower() in
                ('.hex','.mem','.bin','.mif') and 'impl' not in p.relative_to(root).parts}
    a,b=assets(original),assets(source)
    assert a.keys()==b.keys() and len(a)==104 and all(p.read_bytes()==b[n].read_bytes() for n,p in a.items())
    for name in ('g2.cst','g2.sdc','ip/serdes.csr'):
        assert (source/name).read_bytes()==(original/name).read_bytes()
    probe=json.loads(Path(manifest['slf_point_planes_probe']).read_text())
    assert Path(probe['source']).resolve()==original.resolve() and probe['all_cycles_passed']
    assert probe['actual_module_bsram_saved']==8 and len(probe['checks'])==2
    assert [r['actual_physical_bsram'] for r in probe['runs']]==[16,8]
    assert all('all4096addresses=1 init_nonzero=1' in row['result'] for row in probe['checks'])
    old=json.loads((original/'memory_inventory.json').read_text());new=json.loads((source/'memory_inventory.json').read_text())
    outside=lambda inv:{r['path']:(r['primitive'],r['parameters']) for r in inv['leaves'] if not r['path'].startswith(POINT)}
    assert outside(old)==outside(new) and len(outside(new))==308, 'another BSRAM changed'
    old_point=[r for r in old['leaves'] if r['path'].startswith(POINT)]
    new_point=[r for r in new['leaves'] if r['path'].startswith(POINT)]
    assert len(old_point)==len(new_point)==8
    assert all(r['primitive']=='SDPX9B' and r['parameters']['BIT_WIDTH_0']=='36' and
               r['parameters']['BIT_WIDTH_1']=='36' for r in old_point)
    assert all(r['primitive']=='SDPX9B' and r['parameters']['BIT_WIDTH_0']=='18' and
               r['parameters']['BIT_WIDTH_1']=='18' and r['parameters']['READ_MODE']=="1'b0" for r in new_point)
    assert old['expanded_bsram_count']==new['expanded_bsram_count']==316
    assert old['wide32_36_count']==18 and new['wide32_36_count']==10
    assert old['predicted_split_bsram']==334 and new['predicted_split_bsram']==326<=336
    assert new['primitive_types']['GTR12_QUAD']==1
    summary=json.loads((source/'synthesis_summary.json').read_text())
    prior=json.loads((original/'synthesis_summary.json').read_text())
    assert not summary['errors'] and summary['actual_synthesis_global_freq_mhz']==50
    core,=[r for r in summary['synthesis_timing']['clocks'] if r['name']=='clk50']
    assert core['constraint_mhz']==50 and core['estimated_fmax_mhz']>=50
    get=lambda s,k:int(s['synthesis_hierarchy'][0].get(k,'0').split('(')[0])
    delta={k:get(summary,k)-get(prior,k) for k in ('T_Lut','T_Alu','T_Register','T_MULTALU27X18')}
    assert delta['T_MULTALU27X18']==0, 'control arithmetic unexpectedly changed'
    before=read(original/'impl/gwsynthesis/g2_top.vg');after=read(source/'impl/gwsynthesis/g2_top.vg')
    old_defs=definitions(original/'impl/gwsynthesis/g2_top.vg');new_defs=definitions(source/'impl/gwsynthesis/g2_top.vg')
    route=routing(source,json.loads(Path(plan_file).read_text()),before,after,old_defs,new_defs)
    from test_point_planes_routing import check as check_routing_rejections
    routing_regressions=check_routing_rejections(source,json.loads(Path(plan_file).read_text()),before,after,old_defs,new_defs)
    lane=verify_frozen(source,original,manifest['sfp_lane2_from_build'])
    lane['actual_synthesized_clock_pins']=verify_synthesized_clock_pins(after)
    directions=primitive_directions(LIBRARY)
    # All new storage clocks must still be the original core clock.
    point_cells={n:c for n,c in after.items() if n.startswith(POINT.removeprefix('g2_top/'))}
    ram_cells=[c for c in point_cells.values() if c['kind']=='SDPX9B']
    assert len(ram_cells)==8
    for cell in ram_cells:
        assert cell['ports']['CLKA']==cell['ports']['CLKB']==['clk50_d']
        assert cell['ports']['RESET']==['GND']
        init={k:v for k,v in parameters(cell,new_defs).items()
              if k.startswith('INIT_RAM_')}
        assert len(init)==64 and all(int(v.split("'h")[1],16)==0 for v in init.values()), 'full point RAM initialization changed'
    reference=json.loads((baseline/'cdc_reference_audit.json').read_text())
    lidar=revalidate(before,after,reference,directions,old_defs,new_defs)
    sfp=audit_sfp(source/'impl/gwsynthesis/g2_top.vg',True)
    assert sfp['passed'] and len(sfp['paths'])==51
    assert json.loads(json.dumps(sfp['paths']))==json.loads((baseline/'sfp_cdc_reference.json').read_text())['paths']
    selectors=bank_load(after);assert selectors['maximum']<=128
    row='u_nav/u_match/u_ex/u_rb/'
    def pressure(data):
        local={n:c for n,c in data.items() if n.startswith(row)};assert local
        loads=Counter(net for c in local.values() for pin,nets in c['ports'].items()
                      if directions[c['kind']][pin] in ('input','inout') and pin not in ('CLK','CLKA','CLKB')
                      for net in nets if net not in ('VCC','GND'))
        return dict(maximum_nonclock_load=max(loads.values()),distributed_words=sum(c['kind']=='RAM16SDP4' for c in local.values()),
                    bank_select_registers=sum(c['kind'].startswith('DFF') and 'row_selected' in n for n,c in local.items()))
    row_before,row_after=pressure(before),pressure(after)
    assert row_before==row_after and row_after['maximum_nonclock_load']==250 and row_after['distributed_words']==250
    return dict(passed=True,changed_rtl_inputs=changed,unchanged_firmware_storage_assets=104,
                source_bsram=316,wide_leaves=10,predicted_physical_bsram=326,physical_count_is_prediction=True,
                unchanged_other_bsram_leaves=308,point_store_bits=147456,point_source_leaves=new_point,
                point_module_actual_physical_before=16,point_module_actual_physical_after=8,point_module_checks=probe['checks'],
                full_joint_physical_reduction_not_yet_measured=True,core_synthesis_estimate=core,global_resource_delta=delta,
                control_weights_and125hz_unchanged=True,source_csr_and_clock_sdc_unchanged=True,
                routing_driver_revalidation=route,clock_plan_sources_revalidated=[r['net'] for r in route['targets']],
                routing_positive_negative_checks=routing_regressions,
                sfp_lane2_checks=lane,lidar_revalidation=lidar,lidar26_source_connections_match=True,
                host51_source_connections_match=True,host_cdc_audit=sfp,selector_after=selectors,row_store_checks=row_after,
                deployment_accepted=False,recurring_supervision_active=False)
