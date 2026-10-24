"""Freeze a complete DDR-CPU candidate for serial SYN ONLY, not automatic PNR.

All existing source/firmware bytes except the SoC memory backend and top
connections must remain equal. No timing exceptions or control changes.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
from prepare_candidate import transform, once
from generate_phy import inspect as inspect_phy
from generate_pll import inspect as inspect_pll

PORTS='''    output wire [14:0] ddr_addr,
    output wire [2:0] ddr_bank,
    output wire ddr_cs,ddr_ras,ddr_cas,ddr_we,ddr_ck,ddr_ck_n,
    output wire ddr_cke,ddr_odt,ddr_reset_n,
    output wire [3:0] ddr_dm,
    inout wire [31:0] ddr_dq,
    inout wire [3:0] ddr_dqs,ddr_dqs_n,
'''

INSTANCE='''    // DDR clock is separate: original core/control remains clk50.
    // Only FPGA power-on restarts DDR and copies initial RAM contents.
    reg [5:0] ddr_power_on=0;
    always @(posedge clk50) ddr_power_on<={ddr_power_on[4:0],1'b1};
    wire ddr_cold_rst=~ddr_power_on[5];
    wire ddr_ui_clk,ddr_memory_clk,ddr_pll_lock,ddr_pll_stop;
    wire ddr_calib_done,ddr_controller_reset,ddr_mem_init,ddr_mem_fault;
    wire [2:0] dd_cmd;
    wire dd_en,dd_ready,dd_wen,dd_wend,dd_wr_ready,dd_rvalid,dd_rend;
    wire [27:0] dd_addr;
    wire [255:0] dd_wdata,dd_rdata;
    wire [31:0] dd_mask;
    Nav_DDR3_PLL u_ddr_pll(.clkin(clk50),.reset(ddr_cold_rst),.lock(ddr_pll_lock),
        .clkout0(),.clkout1(),.clkout2(ddr_memory_clk),
        .enclk0(1'b1),.enclk1(1'b1),.enclk2(ddr_pll_stop));
    Nav_DDR3_PHY u_ddr_phy(.clk(clk50),.memory_clk(ddr_memory_clk),
        .pll_lock(ddr_pll_lock),.rst_n(~ddr_cold_rst),.pll_stop(ddr_pll_stop),
        .clk_out(ddr_ui_clk),.ddr_rst(ddr_controller_reset),.init_calib_complete(ddr_calib_done),
        .cmd(dd_cmd),.cmd_en(dd_en),.addr(dd_addr),.cmd_ready(dd_ready),
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

SOC_PORTS='''        .ddr_clk(ddr_ui_clk),.ddr_cold_rst(ddr_cold_rst),
        .ddr_calib_done(ddr_calib_done),.ddr_pll_locked(ddr_pll_lock),
        .ddr_cmd(dd_cmd),.ddr_cmd_en(dd_en),.ddr_addr(dd_addr),.ddr_cmd_ready(dd_ready),
        .ddr_wr_data(dd_wdata),.ddr_wr_mask(dd_mask),.ddr_wr_en(dd_wen),.ddr_wr_end(dd_wend),
        .ddr_wr_ready(dd_wr_ready),.ddr_rd_data(dd_rdata),.ddr_rd_valid(dd_rvalid),.ddr_rd_end(dd_rend),
        .memory_initialized(ddr_mem_init),.memory_fault(ddr_mem_fault),
'''


def prepare(source,candidate,phy,pll,out,factory_cst):
    assert not out.exists()
    checks=json.loads((candidate/'functional_result.json').read_text())['checks']
    assert len(checks)==2 and all('TEST PASSED' in c for c in checks)
    assert 'warm_reset_retained=1 clock_loss_latched=1' in checks[1]
    inspect_phy(phy);inspect_pll(pll)
    image=json.loads((candidate/'candidate.json').read_text())
    assert Path(image['source']).resolve()==source.resolve()
    shutil.copytree(source,out,ignore=shutil.ignore_patterns('impl','__pycache__','gw_sh.log',
        'preflight.log','job.json','synthesis_summary.json','memory_inventory.json','tool_runtime.json'))
    baseline=(source/'nav/rtl/soc/nav_soc.v').read_text(encoding='utf-8')
    expected=transform(baseline,image['boot_image_bytes']//4)
    assert (candidate/'nav_soc_ddr.v').read_text(encoding='utf-8')==expected
    (out/'nav/rtl/soc/nav_soc.v').write_text(expected,encoding='utf-8')
    top=(source/'g2_top.v').read_text(encoding='utf-8')
    top=once(top,'    // 调试串口\n',PORTS+'    // 调试串口\n')
    top=once(top,'    // ---------------- 总线：',INSTANCE+'\n    // ---------------- 总线：')
    top=once(top,'    nav_soc #(','    nav_soc_ddr #(')
    marker=')) u_soc (\n        .clk(clk), .rst(rst),'
    top=once(top,marker,marker+'\n'+SOC_PORTS.rstrip())
    for module,instance,template in (
        ('Nav_DDR3_PHY','u_ddr_phy',phy/'ip/nav_ddr3_phy/nav_ddr3_phy_tmp.v'),
        ('Nav_DDR3_PLL','u_ddr_pll',pll/'nav_ddr3_pll_tmp.v')):
        body=re.search(module+r'\s+'+instance+r'\s*\((.*?)\);',top,re.S)[1]
        provided=set(re.findall(r'\.(\w+)\s*\(',body))
        actual=set(re.findall(r'(?m)^\s*\.([A-Za-z_]\w*)\s*\(',template.read_text()))
        assert provided==actual,(module,provided-actual,actual-provided)
    (out/'g2_top.v').write_text(top,encoding='utf-8')
    ddr=out/'ddr3';ddr.mkdir()
    new=[]
    for name in ('nav_ddr_boot_rom.v','nav_ddr_cache.v','nav_ddr_native.v'):
        shutil.copyfile(candidate/name,ddr/name);new.append(ddr/name)
    for p in candidate.glob('boot_b*.hex'):shutil.copyfile(p,out/p.name)
    for p in (phy/'ip/nav_ddr3_phy/nav_ddr3_phy.v',pll/'nav_ddr3_pll.v'):
        shutil.copyfile(p,ddr/p.name);new.append(ddr/p.name)
    old_cst=(source/'g2.cst').read_text(encoding='utf-8')
    extra='\n'.join(l for l in factory_cst.read_text().splitlines()
        if re.match(r'IO_(LOC|PORT) "ddr_',l))+'\n'
    old_pins=set(re.findall(r'(?m)^IO_LOC[^\n]*?\s([A-Z]+\d+)\s*;',old_cst))
    ddr_pins=set(re.findall(r'(?m)^IO_LOC[^\n]*?\s([A-Z]+\d+(?:,[A-Z]+\d+)?)\s*;',extra))
    ddr_pins={p for pair in ddr_pins for p in pair.split(',')}
    assert not old_pins & ddr_pins, old_pins & ddr_pins
    assert 'IO_LOC "ddr_addr[12]" AA3;' in extra and 'IO_LOC "ddr_addr[1]" AB3;' in extra
    (out/'g2.cst').write_text(old_cst+'\n// DDR-only pins from audited factory table; placement remains gated.\n'+extra,encoding='utf-8')
    old_sdc=(source/'g2.sdc').read_text(encoding='utf-8')
    # No new false-path exceptions. Actual generated clock pins/bundled CDC
    # timing must be derived from the full SYN before any PNR can be launched.
    (out/'g2.sdc').write_text(old_sdc+'\ncreate_clock -name ddr_ui_clk -period 10 [get_pins {u_ddr_phy/clk_out}]\n',encoding='utf-8')
    m=json.loads((source/'manifest.json').read_text())
    differences=[]
    for path in m['source_files']:
        rel=Path(path).relative_to(source)
        if (source/rel).read_bytes()!=(out/rel).read_bytes():differences.append(rel.as_posix())
    assert sorted(differences)==['g2_top.v','nav/rtl/soc/nav_soc.v'],differences
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
    m=relocate(m);m['source_files'] += [str(p) for p in new]
    m.update(frozen_source_build=str(source),ddr_cpu_candidate=True,
        ddr_cpu_functional_candidate=str(candidate),ddr_phy_source=str(phy),ddr_pll_source=str(pll),
        automatic_pnr=False,ddr_physical_acceptance=False,
        scope='DDR CPU migration full-source SYN only; physical clocks, bundled CDC and navigation latency unaccepted')
    (out/'manifest.json').write_text(json.dumps(m,indent=2),encoding='utf-8')
    tcl=relocate((source/'run.tcl').read_text(encoding='utf-8'))
    add='\n'.join(f'add_file -type verilog {{{p.as_posix()}}}' for p in new)+'\n'
    tcl=once(tcl,'set_option -top_module g2_top',add+'set_option -top_module g2_top')
    (out/'run.tcl').write_text(tcl,encoding='utf-8')
    r=dict(prepared_at=datetime.now().astimezone().isoformat(),source=str(source),candidate=str(candidate),
        prepared=str(out),modified_rtl_inputs=differences,unchanged_original_assets=assets,
        original_source_inputs=len(m['source_files'])-len(new),new_source_inputs=len(new),
        all_control_sources_unchanged=True,original_four_clock_lines_preserved=True,
        ddr_unique_physical_pins=len(ddr_pins),pin_overlap=sorted(old_pins&ddr_pins),
        new_timing_exceptions=0,automatic_pnr=False,programmed=False,
        pending_before_pnr=['actual RAM/resource effect','original and new clock drivers',
            'available PRIMARY/LW/HCLK allocation','native bundled-data CDC and reset constraints',
            'original 26+51 and extra 43 CDC/PCS/IO','navigation latency/deadlines'])
    (out/'ddr_preparation.json').write_text(json.dumps(r,indent=2),encoding='utf-8');return r


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--phy',type=Path,required=True);p.add_argument('--pll',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--factory-cst',type=Path,required=True);a=p.parse_args()
    r=prepare(*[x.resolve() for x in (a.source,a.candidate,a.phy,a.pll,a.out,a.factory_cst)])
    print(json.dumps({k:v for k,v in r.items() if k!='unchanged_original_assets'},indent=2))
