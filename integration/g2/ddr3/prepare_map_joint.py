"""Freeze the RTL-proved shared-map candidate in the entire original G2.

Parallel implementation is exploratory: DDR service and UPDATE WCET remain
deployment gates. CPU RAM, control sources and all 104 assets stay unchanged.
No behavioral DDR model or DQS simulation file enters this hardware build.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil

from prepare_joint import PORTS
from generate_phy import inspect as inspect_phy
from generate_pll import inspect as inspect_pll
from derive_ddr_matcher import HERE, derive, derive_store, derive_row_engine
from derive_ddr_elevation import derive as derive_em, derive_map, derive_traversability
from derive_ddr_shared import derive_store as share_store, derive_cache, derive_matcher
from derive_ddr_full_subsystem import subsystem, matcher, elevation, map_top
from interleave_struct_cache import derive as interleave


DDR_INSTANCE = '''    wire ddr_power_reset,ddr_protocol_reset,ddr_sticky_fault;
    wire ddr_ui_clk,ddr_memory_clk,ddr_pll_lock,ddr_pll_stop;
    wire ddr_calib_done,ddr_controller_reset,ddr_mem_init,ddr_mem_fault;
    wire [2:0]dd_cmd;wire[27:0]dd_addr;
    wire dd_en,dd_ready,dd_wen,dd_wend,dd_wr_ready,dd_rvalid,dd_rend;
    wire[255:0]dd_wdata,dd_rdata;wire[31:0]dd_mask;
    nav_ddr_board_reset #(.STARTUP_CYCLES(CLK_HZ/5000)) u_ddr_reset(
        .clk(clk),.board_reset(rst),.memory_fault(ddr_mem_fault),
        .power_reset(ddr_power_reset),.protocol_reset(ddr_protocol_reset),.sticky_fault(ddr_sticky_fault));
    Nav_DDR3_PLL u_ddr_pll(.clkin(clk50),.reset(ddr_protocol_reset),.lock(ddr_pll_lock),
        .clkout0(),.clkout1(),.clkout2(ddr_memory_clk),
        .enclk0(1'b1),.enclk1(1'b1),.enclk2(ddr_pll_stop));
    Nav_DDR3_PHY u_ddr_phy(.clk(clk50),.memory_clk(ddr_memory_clk),
        .pll_lock(ddr_pll_lock),.rst_n(~ddr_protocol_reset),.pll_stop(ddr_pll_stop),
        .clk_out(ddr_ui_clk),.ddr_rst(ddr_controller_reset),.init_calib_complete(ddr_calib_done),
        .cmd(dd_cmd),.cmd_en(dd_en),.addr({1'b0,dd_addr}),.cmd_ready(dd_ready),
        .wr_data(dd_wdata),.wr_data_mask(dd_mask),.wr_data_en(dd_wen),
        .wr_data_end(dd_wend),.wr_data_rdy(dd_wr_ready),.rd_data(dd_rdata),
        .rd_data_valid(dd_rvalid),.rd_data_end(dd_rend),
        .sr_req(1'b0),.ref_req(1'b0),.sr_ack(),.ref_ack(),.burst(1'b0),
        .O_ddr_addr(ddr_addr),.O_ddr_ba(ddr_bank),.O_ddr_cs_n(ddr_cs),
        .O_ddr_ras_n(ddr_ras),.O_ddr_cas_n(ddr_cas),.O_ddr_we_n(ddr_we),
        .O_ddr_clk(ddr_ck),.O_ddr_clk_n(ddr_ck_n),.O_ddr_cke(ddr_cke),
        .O_ddr_odt(ddr_odt),.O_ddr_reset_n(ddr_reset_n),.O_ddr_dqm(ddr_dm),
        .IO_ddr_dq(ddr_dq),.IO_ddr_dqs(ddr_dqs),.IO_ddr_dqs_n(ddr_dqs_n));
'''

NAV_PORTS = '''        .ddr_ui_clk(ddr_ui_clk),.ddr_cold_rst(ddr_protocol_reset),
        .ddr_calib_done(ddr_calib_done&&!ddr_sticky_fault),
        .ddr_pll_locked(ddr_pll_lock&&!ddr_sticky_fault),
        .ddr_memory_fault(ddr_mem_fault),.ddr_initialized(ddr_mem_init),
        .ddr_cmd(dd_cmd),.ddr_cmd_en(dd_en),.ddr_addr(dd_addr),.ddr_cmd_ready(dd_ready),
        .ddr_wr_data(dd_wdata),.ddr_wr_mask(dd_mask),.ddr_wr_en(dd_wen),.ddr_wr_end(dd_wend),
        .ddr_wr_ready(dd_wr_ready),.ddr_rd_data(dd_rdata),.ddr_rd_valid(dd_rvalid),.ddr_rd_end(dd_rend),
'''


def prepare(source, proof, phy, pll, audited_cpu_prepared, out):
    assert not out.exists(), 'preserve every existing build'
    result = json.loads((proof/'result.json').read_text())
    assert result['passed'] and result['interleaved_cache_banks']
    assert result['metrics']['checks']==75 and result['metrics']['max_polls']<=64
    assert result['CPU_HOLD'][0]['capacity_KiB']==128
    inspect_phy(phy); inspect_pll(pll)
    manifest = json.loads((source/'manifest.json').read_text())
    assert manifest['device_version']=='C' and manifest['real_sfp']
    assert manifest['slf_point_planes'] and manifest['snapshot_private_reset']
    nav = source/'nav'
    def original(rel): return (nav/rel).read_text(encoding='utf-8-sig')
    generated = {
        'nav_subsystem_ddr_full.v': subsystem(original('rtl/top/nav_subsystem.v')),
        'slf_match_ddr_full.v': matcher(derive_matcher(derive(original('rtl/match/slf_match.v')))),
        'elev_map_ddr.v': elevation(derive_em(original('rtl/map/elev_map.v'))),
        'map_subsys_ddr.v': map_top(derive_map(original('rtl/map/map_subsys.v'))),
        'trav_v3_ce.v': derive_traversability(original('rtl/map/trav_v3.v')),
        'nav_ddr_structure_store_full.v': share_store(derive_store((HERE/'nav_ddr_structure_store.v').read_text())),
        'nav_slf_row_update_full.v': derive_row_engine((HERE/'nav_slf_row_update.v').read_text()),
        'nav_ddr_struct_cache_fast.v': interleave((HERE/'nav_ddr_struct_cache.v').read_text()),
        'nav_ddr_em_cache.v': derive_cache((HERE/'nav_ddr_map_cache.v').read_text())}
    for name, text in generated.items():
        assert (proof/name).read_text()==text, 'tested RTL binding: '+name
    direct = ['nav_ddr_native.v','nav_ddr_row_stream.v','nav_ddr_dual_step.v',
              'nav_ddr_health.v','nav_ddr_admission.v']
    for name in direct:
        assert (proof/name).read_bytes()==(HERE/name).read_bytes(), name
    row_proof=source.parent/'ddr_map_row_update_probe7'
    assert json.loads((row_proof/'functional_result.json').read_text())['passed']
    assert (proof/'slf_lf_ce.v').read_bytes()==(row_proof/'slf_lf_ce.v').read_bytes()
    for item in manifest['source_files']:
        p=Path(item)
        if p.is_relative_to(nav):
            assert p.read_bytes()==(proof/p.name).read_bytes(), p.name
    shutil.copytree(source,out,ignore=shutil.ignore_patterns('impl','__pycache__','gw_sh.log',
        'preflight.log','job.json','tool_runtime.json','synthesis_summary.json','memory_inventory.json'))
    ddr=out/'ddr3';ddr.mkdir()
    for name, text in generated.items(): (ddr/name).write_text(text,encoding='utf-8')
    for name in direct: shutil.copyfile(proof/name,ddr/name)
    shutil.copyfile(proof/'slf_lf_ce.v',ddr/'slf_lf_ce.v')
    shutil.copyfile(HERE/'nav_ddr_board_reset.v',ddr/'nav_ddr_board_reset.v')
    for p in (phy/'ip/nav_ddr3_phy/nav_ddr3_phy.v',pll/'nav_ddr3_pll.v'):
        shutil.copyfile(p,ddr/p.name)
    top=(source/'g2_top.v').read_text(encoding='utf-8')
    def one(a,b):
        nonlocal top
        assert top.count(a)==1,a[:100]
        top=top.replace(a,b,1)
    one('    // 调试串口\n',PORTS+'    // 调试串口\n')
    one('    // ---------------- 总线：',DDR_INSTANCE+'\n    // ---------------- 总线：')
    one('    nav_subsystem #(','    nav_subsystem_ddr_full #(')
    one('    ) u_nav (\n        .clk(clk), .rst(rst),',
        '    ) u_nav (\n        .clk(clk), .rst(ddr_protocol_reset),\n'+NAV_PORTS.rstrip())
    # Extend the NAV-side protocol resets into each receiving clock domain.
    one('nav_rst_sync u_lrr (.clk(l_rx_clk), .arst(rst),',
        'nav_rst_sync u_lrr (.clk(l_rx_clk), .arst(ddr_protocol_reset),')
    marker='    // ---------------- 导航子系统 ----------------'
    one(marker,'''    wire ddr_nav_hr,ddr_nav_ht,ddr_nav_hc;
    nav_rst_sync u_ddr_hr(.clk(h_rx_clk),.arst(h_rx_rst|ddr_protocol_reset),.rst(ddr_nav_hr));
    nav_rst_sync u_ddr_ht(.clk(h_tx_clk),.arst(h_tx_rst|ddr_protocol_reset),.rst(ddr_nav_ht));
    nav_rst_sync u_ddr_hc(.clk(clk),.arst(h_core_rst|ddr_protocol_reset),.rst(ddr_nav_hc));
'''+marker)
    one('.h_core_rst(h_core_rst),.control_tap_valid', '.h_core_rst(ddr_nav_hc),.control_tap_valid')
    one('.h_rx_clk(h_rx_clk), .h_rx_rst(h_rx_rst),', '.h_rx_clk(h_rx_clk), .h_rx_rst(ddr_nav_hr),')
    one('.h_tx_clk(h_tx_clk), .h_tx_rst(h_tx_rst),', '.h_tx_clk(h_tx_clk), .h_tx_rst(ddr_nav_ht),')
    (out/'g2_top.v').write_text(top,encoding='utf-8')
    # Public hardware IP port sets must match the generated templates exactly.
    for module, instance, template in (
        ('Nav_DDR3_PHY','u_ddr_phy',phy/'ip/nav_ddr3_phy/nav_ddr3_phy_tmp.v'),
        ('Nav_DDR3_PLL','u_ddr_pll',pll/'nav_ddr3_pll_tmp.v')):
        body=re.search(module+r'\s+'+instance+r'\s*\((.*?)\);',top,re.S)[1]
        supplied=set(re.findall(r'\.(\w+)\s*\(',body))
        expected=set(re.findall(r'(?m)^\s*\.([A-Za-z_]\w*)\s*\(',template.read_text()))
        assert supplied==expected,(module,supplied^expected)
    old_cst=(source/'g2.cst').read_text()
    extra='\n'.join(x for x in (audited_cpu_prepared/'g2.cst').read_text().splitlines()
                    if re.match(r'IO_(LOC|PORT) "ddr_',x))+'\n'
    def pins(s):
        return {p for group in re.findall(r'(?m)^IO_LOC[^\n]*?\s([A-Z]+\d+(?:,[A-Z]+\d+)?)\s*;',s)
                for p in group.split(',')}
    assert len(pins(extra))==71 and not pins(extra)&pins(old_cst)
    assert 'IO_LOC "ddr_addr[12]" AA3;' in extra and 'IO_LOC "ddr_addr[1]" AB3;' in extra
    (out/'g2.cst').write_text(old_cst+'\n// Audited DDR-only pins. New global-net choices await actual SYN.\n'+extra)
    sdc=(source/'g2.sdc').read_text()
    sdc+='\ncreate_clock -name ddr_ui_clk -period 10 [get_pins {u_ddr_phy/clk_out}]\n'
    sdc+='create_clock -name ddr_memory_clk -period 2.5 [get_pins {u_ddr_pll/pll_inst/CLKOUT2}]\n'
    (out/'g2.sdc').write_text(sdc)
    modified=[]
    for item in manifest['source_files']:
        rel=Path(item).relative_to(source)
        if (source/rel).read_bytes()!=(out/rel).read_bytes():modified.append(rel.as_posix())
    assert modified==['g2_top.v'],modified
    assets=[]
    for p in source.rglob('*'):
        rel=p.relative_to(source)
        if p.is_file() and 'impl' not in rel.parts and p.suffix.lower() in ('.hex','.mem','.mi','.bin'):
            assert p.read_bytes()==(out/rel).read_bytes(),rel
            assets.append(rel.as_posix())
    assert len(assets)==104,len(assets)
    def relocate(v):
        if isinstance(v,str):return v.replace(str(source),str(out)).replace(source.as_posix(),out.as_posix())
        if isinstance(v,list):return [relocate(x) for x in v]
        if isinstance(v,dict):return {k:relocate(x) for k,x in v.items()}
        return v
    old_nav=out/'nav/rtl/top/nav_subsystem.v'
    m=relocate(manifest)
    m['source_files']=[p for p in m['source_files'] if Path(p)!=old_nav]
    new_sources=sorted(ddr.glob('*.v'))
    m['source_files'] += [str(p) for p in new_sources]
    m.update(frozen_source_build=str(source),ddr_map_candidate=True,CPU_remains_on_chip=True,
        cpu_capacity_KiB=128,map_RTL_proof=str(proof),automatic_pnr=False,
        exploratory_parallel_implementation_authorized=True,deployment_blocked=True,
        real_DDR_service_verified=False,complete_UPDATE_5ms_verified=False,
        new_timing_exceptions=0,scope='full shared-map DDR G2 exploratory implementation; not downloadable')
    (out/'manifest.json').write_text(json.dumps(m,indent=2))
    tcl=relocate((source/'run.tcl').read_text())
    old_line=f'add_file -type verilog {{{old_nav.as_posix()}}}\n'
    assert tcl.count(old_line)==1;tcl=tcl.replace(old_line,'')
    add='\n'.join(f'add_file -type verilog {{{p.as_posix()}}}' for p in new_sources)+'\n'
    assert tcl.count('set_option -top_module g2_top')==1
    tcl=tcl.replace('set_option -top_module g2_top',add+'set_option -top_module g2_top')
    (out/'run.tcl').write_text(tcl)
    report=dict(prepared_at=datetime.now().astimezone().isoformat(),source=str(source),proof=str(proof),out=str(out),
        original_modified_inputs=modified,original_NAV_top_not_compiled_to_preserve_header_guard=True,
        source_inputs=len(m['source_files']),new_sources=[p.name for p in new_sources],
        unchanged_assets=assets,all_control_sources_unchanged=True,CPU_original_bytes_unchanged=True,
        original_four_clock_constraints_preserved=True,DDR_unique_physical_pins=71,
        net_RAM_saving_prediction=67,whole_device_RAM_prediction=259,
        actual_joint_resources_verified=False,complete_UPDATE_5ms_verified=False,
        deployment_blocked=True,automatic_pnr=False,
        remaining_before_exploratory_PNR=['actual new RAM/resource inventory','actual clock sources and global choices',
            'original and new CDC connection/constraint mapping','DDR IO and hardware reset wiring review'])
    (out/'ddr_map_preparation.json').write_text(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('source','proof','phy','pll','audited-cpu-prepared','out'):
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    r=prepare(*[x.resolve() for x in (a.source,a.proof,a.phy,a.pll,a.audited_cpu_prepared,a.out)])
    print(json.dumps({k:v for k,v in r.items() if k!='unchanged_assets'},indent=2))
