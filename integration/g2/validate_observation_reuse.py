"""Full frozen-source checks before the one observation-reuse PNR."""
import json
from pathlib import Path

from synthesis_connectivity import read
from prepare_observation_reuse import transform
from prepare_sfp_lane2 import verify_frozen,verify_synthesized_clock_pins
from revalidate_lidar_cdc import definitions,revalidate
from revalidate_observation_routing import validate as routing,LIBRARY
from audit_routing_topology import primitive_directions
from audit_sfp_cdc import audit as audit_sfp


def validate(source,baseline,plan_file):
    from continue_bank_select_once import bank_load,memories_match
    source,baseline=Path(source),Path(baseline)
    job=json.loads((source/'job.json').read_text())
    manifest=json.loads((source/'manifest.json').read_text())
    original=Path(manifest['frozen_source_build'])
    assert job['phase']=='synthesis_complete' and job['tool_exit']==0
    assert manifest['observation_reuse_prefetch'] and manifest['slf_row_banks'] and manifest['bank_select_data_fanout']
    assert manifest['real_sfp'] and manifest['full_control_telemetry'] and not manifest['control_weights_changed']
    assert manifest['clock_core_mhz']==50 and not manifest.get('bias_block_rom')
    observation='control/rl_observation_v5.v'
    assert (source/observation).read_text()==transform((original/observation).read_text(),prefetch=True)
    changed=[Path(n).relative_to(source).as_posix() for n in manifest['source_files']
             if Path(n).read_bytes()!=(original/Path(n).relative_to(source)).read_bytes()]
    assert changed==job['modified_rtl_inputs']==[observation]
    def assets(root):
        return {p.relative_to(root):p for p in root.rglob('*') if p.is_file() and p.suffix.lower() in
                ('.hex','.mem','.bin','.mif') and 'impl' not in p.relative_to(root).parts}
    a,b=assets(original),assets(source)
    assert a.keys()==b.keys() and len(a)==104 and all(p.read_bytes()==b[n].read_bytes() for n,p in a.items())
    probe=json.loads(Path(manifest['observation_module_probe']).read_text())
    assert Path(probe['source']).resolve()==original.resolve() and probe['operand_prefetch']
    assert 'TEST PASSED cycles=56000 frames=32' in probe['checks'][0]['result']
    assert probe['bsram_unchanged'] and probe['register_delta']==-77
    summary=json.loads((source/'synthesis_summary.json').read_text())
    prior=json.loads((original/'synthesis_summary.json').read_text())
    assert not summary['errors'] and summary['actual_synthesis_global_freq_mhz']==50
    get=lambda s,k:int(s['synthesis_hierarchy'][0].get(k,'0').split('(')[0])
    delta={k:get(summary,k)-get(prior,k) for k in ('T_Lut','T_Alu','T_Register','T_MULTALU27X18')}
    assert delta==dict(T_Lut=-505,T_Alu=94,T_Register=-77,T_MULTALU27X18=-12)
    core,=[x for x in summary['synthesis_timing']['clocks'] if x['name']=='clk50']
    assert core['constraint_mhz']==50 and core['estimated_fmax_mhz']>=50
    old_mem=json.loads((original/'memory_inventory.json').read_text())
    mem=json.loads((source/'memory_inventory.json').read_text())
    assert memories_match(old_mem,mem) and mem['expanded_bsram_count']==316 and mem['wide32_36_count']==18
    assert mem['predicted_split_bsram']==334<=336 and mem['primitive_types']['GTR12_QUAD']==1
    lane=verify_frozen(source,original,manifest['sfp_lane2_from_build'])
    for name in ('g2.cst','g2.sdc','ip/serdes.csr'):
        assert (source/name).read_bytes()==(original/name).read_bytes()
    old_cells=read(original/'impl/gwsynthesis/g2_top.vg')
    cells=read(source/'impl/gwsynthesis/g2_top.vg')
    old_defs=definitions(original/'impl/gwsynthesis/g2_top.vg')
    new_defs=definitions(source/'impl/gwsynthesis/g2_top.vg')
    route=routing(source,json.loads(Path(plan_file).read_text()),old_cells,cells,old_defs,new_defs)
    lane['actual_synthesized_clock_pins']=verify_synthesized_clock_pins(cells)
    directions=primitive_directions(LIBRARY)
    reference=json.loads((baseline/'cdc_reference_audit.json').read_text())
    lidar=revalidate(old_cells,cells,reference,directions,old_defs,new_defs)
    sfp=audit_sfp(source/'impl/gwsynthesis/g2_top.vg',True)
    assert sfp['passed'] and len(sfp['paths'])==51
    assert json.loads(json.dumps(sfp['paths']))==json.loads((baseline/'sfp_cdc_reference.json').read_text())['paths']
    selectors=bank_load(cells)
    assert selectors['maximum']<=128
    row='u_nav/u_match/u_ex/u_rb/'
    # This unchanged storage implementation must retain the same direct loads.
    from collections import Counter
    def pressure(data):
        local={n:c for n,c in data.items() if n.startswith(row)}
        assert local
        loads=Counter(net for c in local.values() for pin,nets in c['ports'].items()
                      if directions[c['kind']][pin] in ('input','inout') and pin not in ('CLK','CLKA','CLKB')
                      for net in nets if net not in ('VCC','GND'))
        return dict(maximum_nonclock_load=max(loads.values()),distributed_words=sum(c['kind']=='RAM16SDP4' for c in local.values()),
                    bank_select_registers=sum(c['kind'].startswith('DFF') and 'row_selected' in n for n,c in local.items()))
    before,after=pressure(old_cells),pressure(cells)
    assert before==after and after['maximum_nonclock_load']==250 and after['distributed_words']==250
    return dict(passed=True,source_bsram=316,wide_leaves=18,predicted_physical_bsram=334,physical_count_is_prediction=True,
        changed_rtl_inputs=changed,unchanged_firmware_storage_assets=104,all_existing_bsram_parameters_match=True,
        control_weights_and125hz_unchanged=True,source_csr_and_clock_sdc_unchanged=True,
        core_synthesis_estimate=core,global_resource_delta=delta,observation_rtl_probe=probe['checks'][0],
        native_netlist_simulation_available=probe['vendor_verification_available'],
        native_simulation_gap=probe.get('vendor_simulator_block'),full_candidate_for_physical_evaluation=True,
        routing_driver_revalidation=route,clock_plan_sources_revalidated=[r['net'] for r in route['targets']],
        sfp_lane2_checks=lane,lidar_revalidation=lidar,lidar26_source_connections_match=True,
        host51_source_connections_match=True,host_cdc_audit=sfp,selector_after=selectors,row_store_checks=after,
        deployment_accepted=False,recurring_supervision_active=False)
