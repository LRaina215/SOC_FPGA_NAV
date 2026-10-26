"""Future ROM/CPU overlay, kept as Python strings until PNR54 natural exit.

--plan writes JSON only. --materialize is separately explicit and refuses any
unfinished/cancelled PNR54. No EDA, firmware build or source-list editing occurs.
"""
from pathlib import Path
from datetime import datetime
import argparse,difflib,json,psutil,shutil
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
HW=BASE/'SLF128_storage_safeFW_SYN43_prepared1'
GATE=BASE/'SLF128_storage_place2_pnr54/tool_runtime.json'
FW=BASE/'P11_quiesce_FW_sin_MMIO_prepared1'

ROM_MODULE='''// Independent firmware Q14 table. No matcher/traj port is shared.
module nav_fw_sin_rom #(parameter INIT_FILE="fw_sin_lut.hex") (
    input wire clk, input wire re, input wire [9:0] addr,
    output reg [15:0] q
);
    (* syn_ramstyle="block_ram",syn_romstyle="block_rom" *)
    reg [15:0] mem[0:1023];
    initial begin
        $readmemh(INIT_FILE,mem);
        q=16'd0;
    end
    always @(posedge clk) if(re) q<=mem[addr];
endmodule
'''

def patch_soc(old):
    t=old
    before='    parameter INIT_B3 = ""\n'
    after='    parameter INIT_B3 = "",\n    parameter FW_SIN_INIT = "fw_sin_lut.hex"\n'
    assert t.count(before)==1;t=t.replace(before,after)
    before="    wire sel_per = !sel_ram && !sel_nav;           // 外设与未映射地址（读 0、写忽略）\n"
    after='''    // Dedicated CPU-local data-only query window: signed16 at each32bit word.
    // PicoRV32 word-aligns its external address even for LH, so stride is4.
    wire sel_fw_sin = (mem_addr[31:12] == 20'h90000);
    wire sel_per = !sel_ram && !sel_nav && !sel_fw_sin;
    reg fw_sin_pending;
    wire [15:0] fw_sin_q;
'''
    assert t.count(before)==1;t=t.replace(before,after)
    before='    wire req     = mem_valid && !mem_ready;\n'
    after=before+'''    wire fw_sin_read = req && sel_fw_sin && !fw_sin_pending &&
                       !(|mem_wstrb) && !cpu_rst;
    nav_fw_sin_rom #(.INIT_FILE(FW_SIN_INIT)) u_fw_sin (
        .clk(clk), .re(fw_sin_read), .addr(mem_addr[11:2]), .q(fw_sin_q)
    );
'''
    assert t.count(before)==1;t=t.replace(before,after)
    before="            m_req <= 1'b0; nav_busy <= 1'b0;\n"
    after=before+"            fw_sin_pending <= 1'b0;\n"
    assert t.count(before)==1;t=t.replace(before,after)
    before="            if (req && sel_ram) ram_ready <= 1'b1;\n"
    after=before+'''            // E0 demand reads the ROM. E1 publishes Q and ACK exactly once.
            // Old pending1 prevents reacceptance while mem_valid stays high.
            if (fw_sin_pending) begin
                fw_sin_pending <= 1'b0;
                per_q <= {{16{fw_sin_q[15]}},fw_sin_q};
                per_ready <= 1'b1;
            end
            if (req && sel_fw_sin && !fw_sin_pending) begin
                if (|mem_wstrb) begin
                    per_q <= 32'd0; per_ready <= 1'b1;
                end else fw_sin_pending <= 1'b1;
            end
'''
    assert t.count(before)==1;t=t.replace(before,after)
    # Keep original RAM and console sections exactly intact.
    start='    // ================================================================ RAM（4 个字节通道）'
    end='    // ================================================================ 访问应答'
    assert t[t.index(start):t.index(end)]==old[old.index(start):old.index(end)]
    start='    // ================================================================ 调试口'
    assert t[t.index(start):]==old[old.index(start):]
    return t

def gate54(required):
    r=json.loads(GATE.read_text());assert r['pid']==48212
    assert r.get('max_runtime_s')==0 and r.get('max_no_observable_progress_s')==0 and not r.get('cancelled')
    running=False;reused=False
    try:
        p=psutil.Process(r['pid'])
        # Fixed creation time is from the actual immutable PNR54 start record.
        running=abs(p.create_time()-1793024323.6441314)<0.02
        reused=not running
    except psutil.NoSuchProcess:pass
    complete=r.get('tool_exit') is not None and bool(r.get('ended_at')) and not running
    info=dict(runtime=metadata(GATE),pid=r['pid'],original_created_epoch=1793024323.6441314,
              tool_exit=r.get('tool_exit'),ended_at=r.get('ended_at'),original_process_still_running=running,
              pid_reused_by_other_process=reused,cancelled=False,total_seconds=0,silent_seconds=0,natural_completion_gate=complete)
    if required:assert complete,'PNR54 is unfinished: no RTL output is permitted'
    return info

def run(source,out,materialize):
    assert not out.exists()
    gating=gate54(materialize)
    soc=source/'nav/rtl/soc/nav_soc.v';old=soc.read_text();new=patch_soc(old)
    table=(FW/'fw/include/nav_sin_lut.h').read_bytes()
    assert table==(source/'fw/include/nav_sin_lut.h').read_bytes()
    exact=(source/'sin_lut.hex').read_bytes()
    assert exact==(HW/'sin_lut.hex').read_bytes()
    out.mkdir()
    result=dict(at=datetime.now().astimezone().isoformat(),phase='materialized overlay needs actual proofs'if materialize else'JSON plan only; RTL remains Python strings',
        source=str(source),baseline=[metadata(soc),metadata(source/'sin_lut.hex'),metadata(FW/'fw/include/nav_fw_sin_lookup.h')],
        gate54=gating,materialized=materialize,EDA_started=False,full_FW_or_PNR_input_modified=False,
        files_if_later_materialized=['nav/rtl/soc/nav_soc.v','nav/rtl/soc/nav_fw_sin_rom.v','fw_sin_lut.hex'],
        exact_decoding='0x90000000..0x90000fff; word index mem_addr[11:2]; signed16 extended32; writes ignoredACK0',
        acceptance='E0 demand at req, E1 per_q+per_ready; pending suppresses duplicate acceptance, cpu_rst cancels pending',
        unchanged_sections=['entire original CPU RAM and console posted-write block','entire original debug FSM','all four original trigROMs, ports and INIT'],
        content='all1024 original signed16 Q14 words, byte-identical existing sin_lut.hex; no floating point regeneration',
        hardware_resource_expected='one16bit1024-word BSRAM, no new clock, actual mapping/reset/STA not proved',
        integration_remaining=['root must explicitly add new module to future source list and copy/init-path binding; this overlay does not silently update run.tcl or select a FW image',
             'selected new FW still additionally requires real P11 parser/CSR and RX-only quiesce implementation; do not run it on current selected hardware',
             'actual ROM/CPU miter: all1024/both sign/reset-beforeACK/heldreq/backtoback/writeignore/oldRAM-per-nav-console arbitration',
             'small actual mapping/INIT and final complete CPUINIT/resource/clock/CDC/reset/STA',
             'busyCPU350mszero and physical500ms stop'],deployment_ready=False)
    if materialize:
        dest=out/'nav/rtl/soc';dest.mkdir(parents=True)
        (dest/'nav_soc.v').write_text(new,encoding='utf8')
        (dest/'nav_fw_sin_rom.v').write_text(ROM_MODULE,encoding='utf8')
        shutil.copy2(source/'sin_lut.hex',out/'fw_sin_lut.hex')
        result['outputs']=[metadata(p)for p in sorted(out.rglob('*'))if p.is_file()]
    else:assert not list(out.rglob('*.v'))
    save(out/'result.json',result)
    if not materialize:save(REPO/'docs/4_design/evidence/g2_20261023/FW_sin_ROM_RTL_overlay_plan_20261026.json',result)
    print(json.dumps(dict(output=str(out),materialized=materialize,pnr54_natural_end=gating['natural_completion_gate'],EDA=False)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=HW);p.add_argument('--out',type=Path)
    p.add_argument('--materialize',action='store_true');a=p.parse_args()
    if a.materialize:assert a.out is not None,'explicit new output directory required'
    run(a.source,a.out or BASE/'FW_sin_ROM_RTL_overlay_plan1',a.materialize)
