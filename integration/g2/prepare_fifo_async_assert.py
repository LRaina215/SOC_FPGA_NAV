"""Isolated common-reset FIFO repair candidate and finite RTL comparison.

No joint synthesis/PNR or running input is modified.  The candidate assumes
both FIFO reset inputs are asynchronous-assert/synchronous-release derivatives
of one common reset, as in the actual frozen top.  It is not unilateral-reset
or epoch support.
"""
import argparse
from datetime import datetime
import difflib
import json
from pathlib import Path
import re
import shutil
import subprocess

from prepare_baseline_fusion_retime import metadata, same_bytes, save

ROOT=Path(__file__).resolve().parents[2]
BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_only_syn23'
COUNTEREXAMPLE=BASE/'fifo_clock_stop_reset_counterexample1/result.json'
OUT=BASE/'fifo_async_assert_candidate3'
VIV=Path('E:/2026.1/Vivado/bin')


def load(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def transform_fifo(source):
    substitutions=[
        ('always @(posedge wclk) rp_b_w <=', 'always @(posedge wclk or posedge wrst) rp_b_w <='),
        ('wire        wr     = s_valid & ~wfull;', 'wire        wr     = ~wrst & s_valid & ~wfull;'),
        ('assign s_ready = ~wfull;', 'assign s_ready = ~wrst & ~wfull;'),
        ('always @(posedge wclk) begin\n        if (wrst)', 'always @(posedge wclk or posedge wrst) begin\n        if (wrst)'),
        ('always @(posedge rclk) wp_b_r <=', 'always @(posedge rclk or posedge rrst) wp_b_r <='),
        ('always @(posedge rclk) begin\n        if (rrst)', 'always @(posedge rclk or posedge rrst) begin\n        if (rrst)')]
    result=source
    for before,after in substitutions:
        assert result.count(before)==1,before
        result=result.replace(before,after,1)
    # Keep both clocked RAM processes literally unchanged, apart from the
    # explicitly reviewed reset gating of the existing wr control signal.
    for memory_process in ('always @(posedge wclk) if (wr) mem[wp_b[AW-1:0]] <= s_data;',
                           'always @(posedge rclk) if (rd) q <= mem[rp_b[AW-1:0]];'):
        assert source.count(memory_process)==result.count(memory_process)==1
    assert result.count('or posedge wrst')==result.count('or posedge rrst')==2
    return result


def transform_ob(source):
    result=source.replace("assign ram_rd  = ram_avail & (occ < 3'd2);",
                          "assign ram_rd  = ~rst & ram_avail & (occ < 3'd2);")
    result=result.replace("assign m_valid = (cnt != 2'd0);", "assign m_valid = ~rst & (cnt != 2'd0);")
    assert result!=source
    start=result.index('    always @(posedge clk) begin')
    assert result.count('    always @(posedge clk) begin')==1
    original_process=result[start:result.rindex('endmodule')]
    assert "pend <= ram_rd;" in original_process and "case ({pend, pop})" in original_process
    new_process='''    // Only validity/occupancy state needs asynchronous assertion.  Payload
    // storage is not reset: keep it clocked so RAM/FF inference is unchanged.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pend <= 1'b0;
            cnt <= 2'd0;
        end else begin
            pend <= ram_rd;
            case ({pend, pop})
                2'b10: cnt <= cnt + 2'd1;
                2'b01: cnt <= cnt - 2'd1;
                default: ;
            endcase
        end
    end
    always @(posedge clk) begin
        if (!rst) begin
            case ({pend, pop})
                2'b10: begin
                    if (cnt == 2'd0) b0 <= ram_q; else b1 <= ram_q;
                end
                2'b01: b0 <= b1;
                2'b11: begin
                    if (cnt == 2'd1) b0 <= ram_q;
                    else begin b0 <= b1; b1 <= ram_q; end
                end
                default: ;
            endcase
        end
    end
'''
    return result[:start]+new_process+result[result.rindex('endmodule'):]


def static_review():
    top=(SOURCE/'g2_top.v').read_text(encoding='utf-8')
    port=(SOURCE/'nav/rtl/net/net_port.v').read_text(encoding='utf-8')
    sub=(SOURCE/'nav/rtl/top/nav_subsystem.v').read_text(encoding='utf-8')
    mac=(SOURCE/'nav/rtl/net/rgmii_mac100.v').read_text(encoding='utf-8')
    cdc=(SOURCE/'nav/rtl/common/nav_cdc.v').read_text(encoding='utf-8')
    regfile=(SOURCE/'nav/rtl/regs/nav_regfile.v').read_text(encoding='utf-8')
    assert 'nav_rst_sync u_rs (.clk(clk), .arst(~key_rst_n | ~pll_lock), .rst(rst));' in top
    for name,clock,rst in [('u_lrr','l_rx_clk','l_rx_rst'),('u_ltr','l_tx_clk','l_tx_rst'),
                           ('u_hrr','h_rx_clk','h_rx_rst'),('u_htr','h_tx_clk','h_tx_rst')]:
        assert f'nav_rst_sync {name} (.clk({clock}), .arst(rst), .rst({rst}));' in top
    assert 'assign l_tx_clk = clk;' in top
    assert 'assign h_rx_clk = clk;' in top and 'assign h_tx_clk = clk;' in top
    assert not re.search(r'^\s*`define\s+NAV_USE_(?:SFP|PLL)\b',top,re.M)
    assert 'always @(posedge clk or posedge arst)' in cdc and "r2 <= r1;" in cdc
    assert port.count('nav_async_fifo #(')==2 and sub.count('net_port #(')==2
    assert '.wclk(rx_mac_clk), .wrst(rx_mac_rst)' in port and '.rclk(clk), .rrst(rst)' in port
    assert '.wclk(clk), .wrst(rst)' in port and '.rclk(tx_mac_clk), .rrst(tx_mac_rst)' in port
    assert 'wire start_b = rx_mac_valid & ~in_frame;' in port
    assert 'wire admit   = start_b ? (rx_wfree > MAXF) : ~dropping;' in port
    assert "in_frame <= 1'b0; dropping <= 1'b0;" in port
    assert 'in_frame <= ~rx_mac_last;' in port and '.s_valid(rx_mac_valid & admit)' in port
    rx=mac.split('// ================================================================ 接收',1)[1].split('// ================================================================ 发送',1)[0]
    assert 'if (rst)' not in rx and 'rx_rst' not in rx
    assert "r_NETL_CTRL <= 32'h0000007D;" in regfile
    counter=load(COUNTEREXAMPLE)
    assert counter['counterexample_confirmed'] and counter['metrics']['replay_reads']==4
    rows=[]
    for relative in ('g2_top.v','nav/rtl/top/nav_subsystem.v','nav/rtl/net/net_port.v',
                     'nav/rtl/common/nav_async_fifo.v','nav/rtl/common/nav_fifo_ob.v','nav/rtl/common/nav_cdc.v',
                     'nav/rtl/net/rgmii_mac100.v','nav/rtl/net/udp_rx.v','nav/rtl/regs/nav_regfile.v','nav/rtl/soc/nav_soc.v'):
        text=(SOURCE/relative).read_text(encoding='utf-8')
        terms=('nav_rst_sync','nav_async_fifo','net_port #','wrst','rrst','in_frame','admit','cfg_enable',
               'r_NETL_CTRL <=','ts0 <= time_us','pkt_ts <=','cpu_rst =','posedge rgmii_rxc','initial begin rs')
        rows.append(dict(path=str(SOURCE/relative),metadata=metadata(SOURCE/relative),
                         evidence_lines=[dict(line=i,text=line) for i,line in enumerate(text.splitlines(),1) if any(term in line for term in terms)]))
    return dict(common_reset_for_all4_FIFOs_confirmed=True,actual_asynchronous_FIFOs=['u_nav/u_netl/u_rxf'],
        same_clock_FIFOs=['u_nav/u_netl/u_txf','u_nav/u_neth/u_rxf','u_nav/u_neth/u_txf'],
        independent_one_sided_reset_found=False,CPU_HOLD_resets_only_CPU_not_FIFO=True,
        actual_counterexample=str(COUNTEREXAMPLE),old_empty_nonzero_pointer_replay_confirmed_by_existing_RTL_run=True,
        minimal_sufficient_conditions=['Both endpoints are asserted by the common reset before either endpoint can reopen',
            'All binary/Gray pointers, pointer synchronizers, registered converted remote pointers and output validity are cleared without requiring the stopped clock',
            'Reset deassertion is synchronized separately in each domain by the existing wrappers; no RAM clear',
            'No independent one-sided reset is added; a future independent PHY flush must reset both endpoints or use explicit epoch/ready coordination'],
        normal_no_reset_throughput_latency_or_addressing_intentionally_changed=False,
        MAC_frame_boundary_review=dict(no_explicit_SOF_input=True,start_b_is_local_first_valid_after_in_frame0=True,
            admit_tests_only_available_space_not_real_SOF=True,reset_sets_in_frame0_even_if_MAC_RX_in_progress=True,
            RX_FSM_has_no_reset_branch=True,partial_tail_can_enter_FIFO_after_reset=True,
            accepted_partial_tail_is_not_proof_of_accepted_UDP_or_application=True,
            UDP_header_destination_checksum_and_XT_frame_checks_can_reject_it=True,
            second_application_level_failure_proved=False,
            minimum_followup='Either establish actual idle/frame-end before admission, or reset/re-arm MAC RX using its own synchronized reset and a wait-for-idle state; do not assume start_b is true external SOF'),
        unverified_assumptions=['Real PHY RX clock stop/start and reset behavior, exact on-board occurrence of the demonstrated sequence',
            'RAM dual-clock primitive collision/electrical behavior and whole FIFO protocol formal proof',
            'Physical asynchronous-reset recovery/removal and reset-tree fanout after changing reset style',
            'Maximum input frame length<=1536 and no unbounded jumbo frames while RX s_ready is intentionally unused',
            'Complete end-to-end packet/freshness/vehicle outcome of a stale byte replay'],source_evidence=rows)


def prepare(out=OUT,run=True):
    out=Path(out).resolve();assert not out.exists();out.mkdir(parents=True)
    review=static_review();save(out/'static_reset_and_frame_boundary_review.json',review)
    sources={name:(SOURCE/'nav/rtl/common'/name).read_text(encoding='utf-8')
             for name in ('nav_async_fifo.v','nav_fifo_ob.v','nav_cdc.v')}
    # Ordinary synchronous FIFOs also use nav_fifo_ob.  Keep that public module
    # byte-identical; only async FIFO instances acquire the private reset style.
    private_ob=transform_ob(sources['nav_fifo_ob.v']).replace('module nav_fifo_ob #(',
                                                           'module nav_fifo_ob_async_assert #(',1)
    fifo=transform_fifo(sources['nav_async_fifo.v']).replace('nav_fifo_ob #(',
                                                          'nav_fifo_ob_async_assert #(',1)+'\n'+private_ob
    ob=sources['nav_fifo_ob.v']
    for name,text in [('nav_async_fifo.v',fifo),('nav_fifo_ob.v',ob)]:
        (out/name).write_text(text,encoding='utf-8',newline='\n')
        (out/(name+'.patch')).write_text(''.join(difflib.unified_diff(sources[name].splitlines(True),text.splitlines(True),
             fromfile='a/nav/rtl/common/'+name,tofile='b/nav/rtl/common/'+name)),encoding='utf-8',newline='\n')
    shutil.copyfile(SOURCE/'nav/rtl/common/nav_fifo_ob.v',out/'nav_fifo_ob.v')
    assert same_bytes(SOURCE/'nav/rtl/common/nav_fifo_ob.v',out/'nav_fifo_ob.v')
    for name,text in sources.items():
        shutil.copyfile(SOURCE/'nav/rtl/common'/name,out/('original_'+name))
        assert same_bytes(SOURCE/'nav/rtl/common'/name,out/('original_'+name))
    (out/'compiled_original_fifo.v').write_text(sources['nav_async_fifo.v'].replace('module nav_async_fifo #(',
        'module nav_async_fifo_original #(',1).replace('nav_fifo_ob #(','nav_fifo_ob_original #(',1),encoding='utf-8')
    (out/'compiled_original_ob.v').write_text(sources['nav_fifo_ob.v'].replace('module nav_fifo_ob #(',
        'module nav_fifo_ob_original #(',1),encoding='utf-8')
    shutil.copyfile(ROOT/'integration/g2/tb_fifo_async_assert.sv',out/'tb_fifo_async_assert.sv')
    plan=dict(prepared=True,source=str(SOURCE),candidate=str(out),modified_inputs=['nav/rtl/common/nav_async_fifo.v'],
              public_nav_fifo_ob_byte_identical=True,private_FWFT_helper_in_async_FIFO_file=True,
              common_reset_only_not_unilateral_reset_support=True,RAM_processes_and_capacity_preserved=True,
              RAM_not_reset=True,normal_data_and_handshake_cycles_intended_identical=True,
              joint_SYN_PNR_started=False,programmed=False,deployment_accepted=False)
    save(out/'prepared_candidate.json',plan)
    if not run:return plan
    commands=[('xvlog',[str(out/'compiled_original_fifo.v'),str(out/'compiled_original_ob.v'),
                        str(out/'nav_async_fifo.v'),str(out/'nav_fifo_ob.v'),str(out/'original_nav_cdc.v')]),
              ('xvlog',['-sv',str(out/'tb_fifo_async_assert.sv')]),
              ('xelab',['-debug','off','-timescale','1ns/1ps','tb_fifo_async_assert','-s','snap']),
              ('xsim',['snap','-runall'])]
    runs=[]
    for tool,args in commands:
        logfile=out/(str(len(runs))+'_'+tool+'_stdout.log')
        with logfile.open('w',encoding='utf-8') as log:
            result=subprocess.run(['cmd','/c',str(VIV/(tool+'.bat')),*args],cwd=out,stdout=log,
                                  stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        runs.append(dict(tool=tool,exit_code=result.returncode,log=str(logfile)))
        assert result.returncode==0,(tool,str(logfile))
    log=(out/'xsim.log').read_text(encoding='utf-8',errors='replace')
    assert 'FIFO ASYNC ASSERT TEST PASSED' in log and 'Fatal:' not in log
    summary=re.search(r'FIFO ASYNC ASSERT TEST PASSED[^\r\n]*',log)[0]
    metrics={key:int(value) for key,value in re.findall(r'(\w+)=(\d+)',summary)}
    metrics['pending_words_deliberately_discarded_at_reset']=metrics['normal_writes']-metrics['normal_reads']
    result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),**plan,
        finite_RTL_parameters=dict(DW=10,AW=11),
        metrics=metrics,old_counterexample_preserved=str(COUNTEREXAMPLE),runs=runs,
        static_review=str(out/'static_reset_and_frame_boundary_review.json'),
        source_bindings=[metadata(SOURCE/'nav/rtl/common'/name) for name in sources],
        candidate_bindings=[metadata(out/name) for name in ('nav_async_fifo.v','nav_fifo_ob.v','tb_fifo_async_assert.sv')],
        no_pointer_or_clock_force_used=True,proof_scope='Finite behavioral RTL boundary comparison and actual common-reset stopped-clock cases; not mapped RAM, CDC/MTBF, whole application or physical timing acceptance',
        synthesis_or_PNR_started=False,physical_timing_accepted=False,
        remaining=['Mapped reset/CE/INIT/RAM mode and resource/fanout changes are not measured',
                   'Other DW/AW configurations are not separately simulated by this finite test',
                   'Unilateral reset, full protocol formal proof, CDC MTBF/physical timing and board behavior are not signed off',
                   'MAC reset/frame-boundary resynchronization and end-to-end packet acceptance remain separate'])
    save(out/'result.json',result);return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,default=OUT)
    parser.add_argument('--prepare-only',action='store_true');args=parser.parse_args()
    import psutil
    proc=psutil.Process();proc.nice(psutil.IDLE_PRIORITY_CLASS);proc.cpu_affinity([18])
    result=prepare(args.out,not args.prepare_only)
    print(json.dumps({key:value for key,value in result.items() if key in ('passed','prepared','candidate','metrics','joint_SYN_PNR_started','deployment_accepted')},ensure_ascii=False))
