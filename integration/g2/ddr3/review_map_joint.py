"""Review the completed full-map SYN and freeze exploratory PNR inputs.

DDR protection remains opaque. Vendor hierarchy totals account for its RAM;
the explicit reader must never silently claim the protected 16 blocks vanished.
No new CDC timing exception is introduced for the DDR domains.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from audit_netlist_memories import MODULE, PARAM
from audit_routing_topology import primitive_directions
from audit_sfp_cdc import audit as sfp_audit, sdc_fragment as sfp_sdc
from audit_cdc_first_stages import sdc_fragment as lidar_sdc
from synthesis_connectivity import read
from generate_phy import inspect as inspect_phy

LIBRARY=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')


def review(source, baseline, reference, phy, out):
    assert not out.exists()
    out.mkdir()
    job=json.loads((source/'job.json').read_text())
    assert job['phase']=='synthesis_complete' and job['tool_exit']==0
    manifest=json.loads((source/'manifest.json').read_text())
    assert manifest['ddr_map_candidate'] and manifest['CPU_remains_on_chip']
    assert not manifest['control_weights_changed'] and manifest['clock_core_mhz']==50
    before=json.loads((baseline/'manifest.json').read_text())
    tested=Path(manifest['map_RTL_proof'])
    for p in (source/'ddr3').glob('*.v'):
        if p.name in ('nav_ddr3_phy.v','nav_ddr3_pll.v','nav_ddr_board_reset.v'):continue
        expected=tested/p.name
        if p.name=='nav_ddr_row_stream.v' and manifest.get('row_explicit_mode_proof'):
            explicit=Path(manifest['row_explicit_mode_proof'])
            evidence=json.loads((explicit/'explicit_mode_result.json').read_text())
            assert evidence['actual_vendor_DPX9B_model'] and evidence['original_8_case_cycle_counts_unchanged']
            expected=explicit/'hardware_explicit_row_stream.v'
        elif p.name=='nav_ddr_row_stream.v' and manifest.get('row_normalmode_proof'):
            normal_proof=Path(manifest['row_normalmode_proof'])
            normal=json.loads((normal_proof/'functional_result.json').read_text())
            assert normal['passed'] and normal['metrics']['words']==524288
            expected=normal_proof/p.name
        assert p.read_bytes()==expected.read_bytes(),('unproved new RTL',p.name)
    changed=[]
    for p in before['source_files']:
        rel=Path(p).relative_to(baseline)
        if (source/rel).read_bytes()!=Path(p).read_bytes():changed.append(rel.as_posix())
    assert changed==['g2_top.v'],changed
    assets=[p for p in baseline.rglob('*') if p.is_file() and 'impl' not in p.relative_to(baseline).parts
            and p.suffix.lower() in ('.hex','.mem','.mi','.bin')]
    assert len(assets)==104
    assert all(p.read_bytes()==(source/p.relative_to(baseline)).read_bytes() for p in assets)
    assert (source/'ip/serdes.csr').read_bytes()==(baseline/'ip/serdes.csr').read_bytes()
    summary=json.loads((source/'synthesis_summary.json').read_text())
    assert not summary['errors'] and summary['actual_synthesis_global_freq_mhz']==50
    old=json.loads((baseline/'memory_inventory.json').read_text())
    new=json.loads((source/'memory_inventory.json').read_text())
    old_by={r['path']:r for r in old['leaves']}
    new_by={r['path']:r for r in new['leaves']}
    removed_prefixes=('g2_top/u_nav/u_map/u_em/u_ram/','g2_top/u_nav/u_match/u_sm/')
    moved_prefixes=('g2_top/u_nav/u_match/u_lf/','g2_top/u_nav/u_match/u_touch/')
    retained=[r for p,r in old_by.items() if not p.startswith(removed_prefixes+moved_prefixes)]
    for row in retained:
        other=new_by[row['path']]
        assert (row['primitive'],row['parameters'])==(other['primitive'],other['parameters']),row['path']
    assert len(retained)==196
    if manifest.get('row_normalmode_proof'):
        inputs=[r for r in new['leaves'] if r['primitive']=='DPX9B' and '/u_dma/' in r['path']]
        assert len(inputs)==2 and all(r['parameters']['WRITE_MODE0']=="2'b00" for r in inputs),inputs
    for p,row in old_by.items():
        if p.startswith(moved_prefixes[0]):
            target=p.replace('/u_match/u_lf/','/u_match/u_store/u_update/u_lf/')
            other=new_by[target]
            assert (row['primitive'],row['parameters'])==(other['primitive'],other['parameters'])
    hierarchy={r['path']:r for r in summary['synthesis_hierarchy']}
    vendor_total=int(re.match(r'\d+',hierarchy['/g2_top']['T_Bsram'])[0])
    protected=int(hierarchy['/g2_top/u_ddr_phy']['Bsram'])
    assert protected==16 and new['expanded_bsram_count']+protected==vendor_total==233
    # This is the same original C hardware IP whose isolated SYN exposed 16
    # 32-bit leaves; retain that conservative split assumption until PNR.
    phy_proof=inspect_phy(phy)
    assert phy_proof['readback']['DQ_WIDTH']=='32'
    assert phy_proof['source_bsram_leaves']==phy_proof['wide32_36_leaves']==16
    assert (source/'ddr3/nav_ddr3_phy.v').read_bytes()==(phy/'ip/nav_ddr3_phy/nav_ddr3_phy.v').read_bytes()
    predicted=new['expanded_bsram_count']+new['wide32_36_count']+2*protected
    assert predicted==259 and predicted<=336
    cells=read(source/'impl/gwsynthesis/g2_top.vg')
    dirs=primitive_directions(LIBRARY)
    unknown=Counter((c['kind'],p) for c in cells.values() for p in c['ports'] if p not in dirs.get(c['kind'],{}))
    assert not unknown,unknown
    loads=defaultdict(list);drivers=defaultdict(list)
    for name,c in cells.items():
        for pin,nets in c['ports'].items():
            for net in nets:
                if net in ('GND','VCC'):continue
                if dirs[c['kind']][pin] in ('input','inout'):loads[net].append((name,pin))
                if dirs[c['kind']][pin] in ('output','inout'):drivers[net].append((name,pin))
    lidar=json.loads((reference/'cdc_reference_audit.json').read_text())
    assert lidar['passed'] and len(lidar['paths'])==26
    for r in lidar['paths']:
        src,first,second=[cells[r[k]] for k in ('source_register','first_register','second_register')]
        assert src['ports']['Q']==first['ports']['D'] and first['ports']['Q']==second['ports']['D']
        assert drivers[first['ports']['D'][0]]==[(src['name'],'Q')]
        assert loads[first['ports']['Q'][0]]==[(second['name'],'D')]
        assert src['ports']['CLK']==[r['source_clock']]
        for c in (first,second):
            assert c['ports']['CLK']==[r['target_clock']] and c['ports'].get('CE',['VCC'])==['VCC']
    sfp=sfp_audit(source/'impl/gwsynthesis/g2_top.vg',synthesis=True)
    assert sfp['passed'] and len(sfp['paths'])==51,sfp['errors']
    for path in sfp['paths']:
        assert cells[path['first_register']]['ports'].get('RESET')==cells[path['second_register']]['ports'].get('RESET')
    # Bind the original four clocks to actual visible primitive outputs.
    original_clocks={
        'clk50_d':('clk50_ibuf','O'), 'rgmii_rxc_d':('rgmii_rxc_ibuf','O'),
        'h_rx_clk':('u_sfp/u_mac/u_serdes/gtr12_quad_inst0','LANE2_PCS_RX_O_FABRIC_CLK'),
        'h_tx_clk':('u_sfp/u_mac/u_serdes/gtr12_quad_inst0','LANE2_PCS_TX_O_FABRIC_CLK')}
    for net,endpoint in original_clocks.items():assert drivers[net]==[endpoint],(net,drivers[net])
    assert cells['u_ddr_pll/PLL_inst']['ports']['CLKOUT2']==['ddr_memory_clk']
    clocks=summary['synthesis_timing']['clocks']
    ui,=[r for r in clocks if r['constraint_mhz']==100 and r['name'].endswith('/fclkdiv/CLKOUT.default_gen_clk')]
    mem,=[r for r in clocks if r['constraint_mhz']==400 and r['name']=='u_ddr_pll/PLL_inst/CLKOUT2.default_gen_clk']
    # The protected clock divider is identified in the vendor timing report,
    # not fabricated by the explicit-cell reader (which cannot see it).
    ui_pin=ui['name'].removesuffix('.default_gen_clk')
    memory_pin=mem['name'].removesuffix('.default_gen_clk')
    qdrivers={c['ports']['Q'][0]:n for n,c in cells.items() if c['kind'].startswith('DFF')}
    direct=[]
    for name,c in cells.items():
        if not c['kind'].startswith('DFF'):continue
        source_reg=qdrivers.get(c['ports'].get('D',[''])[0])
        if source_reg and cells[source_reg]['ports']['CLK']!=c['ports']['CLK']:
            q=c['ports']['Q'][0];fanout=loads[q]
            direct.append(dict(source=source_reg,target=name,source_clock=cells[source_reg]['ports']['CLK'],
                target_clock=c['ports']['CLK'],first_q_loads=fanout))
    ddr_direct=[r for r in direct if r['source_clock']==['ddr_ui_clk'] or r['target_clock']==['ddr_ui_clk']]
    # Two largest measured reset nets receive PRIMARY6/7; six tracks are left
    # for original and DDR true clocks. Other reviewed controls request BUFG
    # without a track. Actual resource allocation remains a post-PNR gate.
    control_requests=[
        (cells['u_ddr_pll/PLL_inst']['ports']['RESET'][0],'SR',6),('n2371_7','SR|LOGIC',7),
        ('u_control/model_thread/observation/n971_7','SR|LOGIC',None),
        ('u_control/model_thread/policy/snapshot_update','CE|LOGIC',None),
        ('u_control/model_thread/observation/n12472_10','CE|LOGIC',None),
        ('u_control/telemetry_thread/n1315_4','LOGIC',None),
        ('u_control/telemetry_thread/snapshot_959_10','CE',None)]
    requests=[]
    for net,types,index in control_requests:
        assert len(drivers[net])==1 and len(loads[net])>=800,(net,drivers[net],len(loads[net]))
        assert not any(p in ('CLK','CLKA','CLKB','CLOCK') for n,p in loads[net])
        requests.append(dict(net=net,signal_types=types,primary_index=index,driver=drivers[net][0],
                             measured_SYN_pin_counts=dict(Counter(p for n,p in loads[net]))))
    cst=(source/'g2.cst').read_text()
    assert 'CLOCK_LOC' not in cst
    for r in requests:
        index='' if r['primary_index'] is None else '['+str(r['primary_index'])+']'
        cst+=f'\nCLOCK_LOC "{r["net"]}" BUFG{index} = {r["signal_types"]};'
    sdc=(baseline/'g2.sdc').read_text()
    assert len(re.findall(r'^create_clock',sdc,re.M))==4 and 'set_false_path' not in sdc
    sdc+=f'\ncreate_clock -name ddr_ui_clk -period 10.000 [get_pins {{{ui_pin}}}]\n'
    sdc+=f'create_clock -name ddr_memory_clk -period 2.500 [get_pins {{{memory_pin}}}]\n'
    sdc+='\n'+lidar_sdc(lidar)+'\n'+sfp_sdc(sfp)
    assert len(re.findall(r'^set_false_path .* -hold$',sdc,re.M))==77
    assert len(re.findall(r'^set_max_delay ',sdc,re.M))==77
    (out/'impl/gwsynthesis').mkdir(parents=True)
    for relative in ('impl/gwsynthesis/g2_top.vg','impl/gwsynthesis/g2_top.prj','effective_options.tcl'):
        shutil.copyfile(source/relative,out/relative)
    (out/'ip').mkdir();shutil.copyfile(source/'ip/serdes.csr',out/'ip/serdes.csr')
    (out/'g2.cst').write_text(cst+'\n');(out/'g2.sdc').write_text(sdc)
    (out/'manifest.json').write_text(json.dumps(dict(manifest,reviewed_joint_synthesis=str(source),
        exploratory_only=True,deployment_blocked=True,reviewed_constraints=str(out)),indent=2))
    (out/'cdc_reference_audit.json').write_text(json.dumps(lidar,indent=2))
    (out/'sfp_cdc_reference.json').write_text(json.dumps(sfp,indent=2))
    result=dict(reviewed_at=datetime.now().astimezone().isoformat(),source=str(source),baseline=str(baseline),
        passed=True,scope='exploratory PNR input review; not functional/5ms/CDC or timing acceptance',
        vendor_source_RAM=vendor_total,explicit_source_RAM=new['expanded_bsram_count'],protected_PHY_RAM=protected,
        known_explicit_wide_leaves=new['wide32_36_count'],protected_wide_leaves_predicted=16,
        physical_RAM_prediction=predicted,physical_RAM_measured=False,source_RAM_saved=316-vendor_total,
        physical_RAM_saving_prediction=326-predicted,retained_RAM_parameters_checked=len(retained),
        original_LF_four_leaf_parameters_preserved=True,CPU_RAM_source_leaves=64,control_RAM_source_leaves=43,
        all104assets_identical=True,original_other_RTL_unchanged=True,lidar_connections=26,host_connections=51,
        actual_DDR_UI_clock_pin=ui_pin,actual_DDR_memory_clock_pin=memory_pin,
        direct_crossings_visible=len(direct),DDR_direct_crossings=ddr_direct,protected_DDR_CDC_opaque=True,
        existing_hold_exceptions=77,existing_max_delay_constraints=77,new_DDR_timing_exceptions=0,
        all_new_DDR_paths_remain_normally_timed=True,global_requests=requests,
        SYN_first_path=summary['synthesis_timing']['first_path'],
        pending=['actual physical DDR expansion and allocation','DDR internal timing and all setup/hold',
                 'new CDC bundled payload/Gray skew and reset/PCS/IO full signoff',
                 'real DDR calibration/readback/service and complete UPDATE5ms','navigation performance/freshness'],
        programmed=False)
    (out/'pre_pnr_review.json').write_text(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for k in ('source','baseline','reference','phy','out'):p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args()
    r=review(*[v.resolve() for v in (a.source,a.baseline,a.reference,a.phy,a.out)])
    print(json.dumps({k:v for k,v in r.items() if k not in ('DDR_direct_crossings','global_requests')},indent=2))
