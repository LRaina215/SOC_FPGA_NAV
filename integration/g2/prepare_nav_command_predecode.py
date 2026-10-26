"""Frozen SYN43 six-command predecode preparer; no simulator/EDA execution.

Default --plan renders the one-file change and future bus miter in memory.
--materialize and --prepare-bus-miter are separate future actions, both gated
on natural completion of the original PNR53/54 processes. No .v/.sv is written
by --plan. The old POSE-only/live-repository generator is not used.
"""
import argparse
from datetime import datetime
import difflib
import json
from pathlib import Path
import re
import shutil

from prepare_baseline_fusion_retime import metadata, same_bytes
from prepare_ring_stride32_private_reader import (
    BASE, SOURCE, kept_files, natural_pair_complete, disabled_tcl,
    relocate_paths, write_json, paths_in_manifest,
)


NAV = 'nav/rtl/top/nav_subsystem.v'
PLAN = BASE/'PNR53_NAV_command_predecode_plan2/plan.json'
TOKEN = 'materialize-nav-predecode-after-natural-PNR53-PNR54-completion'
MCAST = 'prepare-bus-miter-only-no-tools'
PULSES = ('match_eval_predecoded_q', 'match_update_predecoded_q',
          'match_clear_predecoded_q', 'pose_commit_predecoded_q',
          'map_sweep_predecoded_q', 'map_clear_predecoded_q')
REPLACEMENTS = (
    ('p_MATCH_CMD & wd_q[0]', PULSES[0], 2),
    ('p_MATCH_CMD & wd_q[1]', PULSES[1], 1),
    ('p_MATCH_CMD & wd_q[2]', PULSES[2], 1),
    ('p_LIDAR_POSE_COMMIT', PULSES[3], 1),
    ('p_MAP_CMD & wd_q[0]', PULSES[4], 1),
    ('p_MAP_CMD & wd_q[1]', PULSES[5], 2),
)
DEFAULTS = '''        // Clear with rf_wr; consumers still execute/ACK at original E1.
        match_eval_predecoded_q <= 1'b0; match_update_predecoded_q <= 1'b0;
        match_clear_predecoded_q <= 1'b0; pose_commit_predecoded_q <= 1'b0;
        map_sweep_predecoded_q <= 1'b0; map_clear_predecoded_q <= 1'b0;
'''
ACCEPT = '''                    if (bus_we) begin
                        rf_wr <= 1'b1;
                        // Same E0 and aligned live operands as a_q/wd_q/rf_wr.
                        // Bitwise data-bit gating preserves original X/Z semantics;
                        // a procedural if(address_match) would incorrectly suppressX.
                        match_eval_predecoded_q <= ({bus_addr[15:2],2'b00} == RA_MATCH_CMD) & bus_wdata[0];
                        match_update_predecoded_q <= ({bus_addr[15:2],2'b00} == RA_MATCH_CMD) & bus_wdata[1];
                        match_clear_predecoded_q <= ({bus_addr[15:2],2'b00} == RA_MATCH_CMD) & bus_wdata[2];
                        pose_commit_predecoded_q <= ({bus_addr[15:2],2'b00} == RA_LIDAR_POSE_COMMIT);
                        map_sweep_predecoded_q <= ({bus_addr[15:2],2'b00} == RA_MAP_CMD) & bus_wdata[0];
                        map_clear_predecoded_q <= ({bus_addr[15:2],2'b00} == RA_MAP_CMD) & bus_wdata[1];
                    end else rf_rd <= 1'b1;'''


def once(text, before, after):
    assert text.count(before) == 1, ('nonunique frozen anchor', before, text.count(before))
    return text.replace(before, after, 1)


def derive(text):
    assert all(p not in text for p in PULSES)
    declarations = '\n    // E0 RF-write acceptance pulses, independently preserve multibit commands.\n'
    declarations += '\n'.join('    reg '+p+';' for p in PULSES)
    new = once(text, '    reg         rf_wr, rf_rd;', '    reg         rf_wr, rf_rd;'+declarations)
    consumer_inventory = []
    for old, new_name, count in REPLACEMENTS:
        assert new.count(old) == count, (old, new.count(old), count)
        consumer_inventory += [dict(old_expression=old, new_pulse=new_name, line=n, text=line.strip())
            for n, line in enumerate(text.splitlines(), 1) if old in line]
        new = new.replace(old, new_name)
    default = "        bus_ack <= 1'b0; rf_wr <= 1'b0; rf_rd <= 1'b0;"
    new = once(new, default, DEFAULTS + default)
    acceptance = "                    if (bus_we) rf_wr <= 1'b1; else rf_rd <= 1'b1;"
    new = once(new, acceptance, ACCEPT)
    # Undo ONLY the introduced source changes; the whole original FSM, pipeline
    # ownership, RAW frame trigger, STOP path and every other line must remain.
    undo = new.replace(ACCEPT, acceptance).replace(DEFAULTS, '')
    # Strip declarations/defaults/acceptance before reversing consumer tokens.
    undo = once(undo, '    reg         rf_wr, rf_rd;'+declarations, '    reg         rf_wr, rf_rd;')
    for old, new_name, count in REPLACEMENTS:
        assert undo.count(new_name) == count
        undo = undo.replace(new_name, old)
    assert undo == text, 'unexpected source difference outside six-pulse patch'
    assert '.frame_start(raw_frame_accept & !match_only)' in new
    assert '.frame_begin(raw_frame_accept && !fuse_only)' in new
    assert 'bs <= B_RF;' in new and 'B_END: bs <= B_IDLE;' in new
    return new, consumer_inventory


def source_data(source):
    manifest, inputs = paths_in_manifest(source)
    original = inputs[NAV].read_text(encoding='utf8')
    generated, consumers = derive(original)
    regfile = source/'nav/rtl/regs/nav_regfile.v'
    regs = regfile.read_text(encoding='utf8')
    header = source/'nav/rtl/regs/nav_regs_map.vh'
    h = header.read_text(encoding='utf8')
    for symbol, addr in (('LIDAR_POSE_COMMIT', '2070'), ('MATCH_CMD', '700C'), ('MAP_CMD', '3004')):
        assert f"assign p_{symbol} = wr && (addr == 16'h{addr});" in regs
        assert f"localparam [15:0] RA_{symbol} = 16'h{addr};" in h
    plan = json.loads(PLAN.read_text(encoding='utf8'))
    assert plan['proposed_structure']['new_logical_pulse_bits'] == 6
    assert plan['actual_PNR53']['address7_launch_endpoints'] == 2546
    for b in plan['bindings']:
        if Path(b['path']).resolve() in (inputs[NAV], regfile.resolve(), header.resolve()):
            assert metadata(Path(b['path'])) == b, ('frozen planned source changed', b['path'])
    files = kept_files(source)
    assets = [r for r in files if Path(r).suffix.lower() in {'.hex', '.mem', '.bin', '.mif'}]
    packed = [r for r in assets if r.startswith('src/mem/weight9_')]
    assert len(assets) == 104 and len(packed) == 35
    assert (source/'firmware/nav_fw.bin').stat().st_size == 53572
    diff = list(difflib.unified_diff(original.splitlines(), generated.splitlines()))
    change = dict(file=NAV, source_bytes=inputs[NAV].stat().st_size,
        generated_UTF8_LF_bytes=len(generated.encode('utf8')),
        old_lines=len(original.splitlines()), new_lines=len(generated.splitlines()),
        removed_lines=sum(x.startswith('-') and not x.startswith('---') for x in diff),
        added_lines=sum(x.startswith('+') and not x.startswith('+++') for x in diff))
    return manifest, inputs, files, assets, generated, consumers, change, plan


def materialize(a, result, data):
    assert a.authorization == TOKEN
    pair = natural_pair_complete()  # Before output creation or any Verilog write.
    out = a.candidate.resolve()
    assert out.parent == BASE.resolve() and not out.exists(), 'one new F child directory required'
    assert not out.is_relative_to(a.source) and not a.source.is_relative_to(out)
    manifest, inputs, files, assets, generated, consumers, change, plan = data
    before = {r: metadata(p) for r, p in files.items()}
    out.mkdir()
    for rel, p in files.items():
        q = out/rel
        q.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, q)
    (out/NAV).write_text(generated, encoding='utf8', newline='\n')
    tcl = disabled_tcl(a.source, out).replace('ring stride32 private reader', 'NAV six-command predecode')
    (out/'run.tcl').write_text(tcl, encoding='utf8', newline='\n')
    m = relocate_paths(manifest, a.source, out)
    m.update(candidate=str(out), frozen_source_build=str(a.source),
        stage='prepared_only_unvalidated', source_preparation_only=True,
        NAV_command_predecode=True, modified_rtl_inputs=[NAV], EDA_authorized=False,
        synthesis_complete=False, automatic_SYN=False, automatic_PNR=False,
        automatic_pnr=False, programmed=False, deployment_accepted=False,
        inherited_proof_scope='Ancestor only; new six-pulse source, bus miter and mapping still pending')
    write_json(out/'manifest.json', m)
    same, rebound = [], ['manifest.json', 'run.tcl']
    for rel, p in files.items():
        assert before[rel] == metadata(p), ('ancestor changed during copy', rel)
        if rel not in [NAV]+rebound:
            assert same_bytes(p, out/rel), rel
            same.append(rel)
    assert all(same_bytes(inputs[rel], out/rel) for rel in inputs if rel != NAV)
    assert all(same_bytes(a.source/rel, out/rel) for rel in assets)
    result.update(phase='source_materialized_unvalidated', RTL_generated=True,
        candidate=str(out), natural_PNR53_PNR54_completion=pair,
        final_byte_identical_hardware_assets_FW_and_ancestor_receipts=same,
        metadata_rebound_or_tool_disabled=rebound,
        inherited_receipts_are_ancestor_provenance_only=True,
        all62_other_RTL_byte_identical=True, all104_assets_byte_identical=True,
        source_frozen_input_bindings=list(before.values()),
        root_source_gate_pending=True)
    write_json(out/'NAV_command_predecode_preparation_review.json', result)
    return result


# Future validation text only. No .v/.sv is emitted by plan/materialize.
# The miter has real NAV and the original two-master arbiter on both sides.
# Its finite idle-network stimuli are deliberately not the full RAW/MBX/console
# deployment proof; run_plan.json names those still-required extensions.
WRAPPER = r'''module @NAME@(
 input clk,rst,q0,w0,q1,w1,input[15:0]a0,a1,input[31:0]d0,d1,
 output k0,k1,output[31:0]r0,r1);
 wire req,we,ack,irq;wire[15:0]addr;wire[31:0]data,rdata;
 wire signed[15:0]vx,vy,wz;wire[7:0]mode;wire stop_,wdog,valid;wire[15:0]seq;
 nav_bus_arb arb(.clk(clk),.rst(rst),.m0_req(q0),.m0_we(w0),.m0_addr(a0),.m0_wdata(d0),.m0_ack(k0),.m0_rdata(r0),
 .m1_req(q1),.m1_we(w1),.m1_addr(a1),.m1_wdata(d1),.m1_ack(k1),.m1_rdata(r1),
 .s_req(req),.s_we(we),.s_addr(addr),.s_wdata(data),.s_ack(ack),.s_rdata(rdata));
 @NAVMODULE@ #(.CLK_HZ(50000000)) n(.clk(clk),.rst(rst),.bus_req(req),.bus_we(we),.bus_addr(addr),.bus_wdata(data),.bus_ack(ack),.bus_rdata(rdata),.irq(irq),
 .l_rx_clk(clk),.l_rx_rst(rst),.l_rx_valid(1'b0),.l_rx_data(8'd0),.l_rx_last(1'b0),.l_rx_error(1'b0),
 .l_tx_clk(clk),.l_tx_rst(rst),.l_tx_valid(),.l_tx_data(),.l_tx_last(),.l_tx_ready(1'b1),.l_phy_stat(32'd0),.l_rx_neg(),
 .h_rx_clk(clk),.h_rx_rst(rst),.h_rx_valid(1'b0),.h_rx_data(8'd0),.h_rx_last(1'b0),.h_rx_error(1'b0),
 .h_tx_clk(clk),.h_tx_rst(rst),.h_tx_valid(),.h_tx_data(),.h_tx_last(),.h_tx_ready(1'b1),.imu_rx(1'b1),.imu_tx(),
 .ch_vx(vx),.ch_vy(vy),.ch_wz(wz),.ch_mode(mode),.ch_stop(stop_),.ch_wdog(wdog),.ch_valid(valid),.ch_seq(seq),
 .odo_valid(1'b0),.odo_vx(16'd0),.odo_vy(16'd0),.odo_wz(16'd0),.ext_stop(4'd0));
endmodule
'''

TB = r'''`timescale 1ns/1ps
module tb_NAV_command_predecode;
 reg clk=0,rst=1;always #10 clk=~clk;
 reg q0=0,w0=0,q1=0,w1=0;reg[15:0]a0=0,a1=0;reg[31:0]d0=0,d1=0;
 wire k00,k01,k10,k11;wire[31:0]r00,r01,r10,r11;
 nav_command_reference_bus ref0(clk,rst,q0,w0,q1,w1,a0,a1,d0,d1,k00,k01,r00,r01);
 nav_command_candidate_bus dut(clk,rst,q0,w0,q1,w1,a0,a1,d0,d1,k10,k11,r10,r11);
 wire[5:0]old_cmd={ref0.n.p_MAP_CMD&ref0.n.wd_q[1],ref0.n.p_MAP_CMD&ref0.n.wd_q[0],ref0.n.p_LIDAR_POSE_COMMIT,
 ref0.n.p_MATCH_CMD&ref0.n.wd_q[2],ref0.n.p_MATCH_CMD&ref0.n.wd_q[1],ref0.n.p_MATCH_CMD&ref0.n.wd_q[0]};
 wire[5:0]new_cmd={dut.n.map_clear_predecoded_q,dut.n.map_sweep_predecoded_q,dut.n.pose_commit_predecoded_q,
 dut.n.match_clear_predecoded_q,dut.n.match_update_predecoded_q,dut.n.match_eval_predecoded_q};
 integer checks=0,transactions=0,resets=0,arbitration=0,unknown_read_words=0,xz_cases=0;
 integer pulses[0:5];reg compare_enabled=0;
 always@(negedge clk)if(compare_enabled)begin
  checks++;
  if(old_cmd!==new_cmd)$fatal(1,"six pulse mismatch old=%b new=%b bs=%h addr=%h data=%h",old_cmd,new_cmd,ref0.n.bs,ref0.n.a_q,ref0.n.wd_q);
  for(integer b=0;b<6;b++)if(old_cmd[b]===1'b1)pulses[b]++;
  if({k00,k01}!=={k10,k11})$fatal(1,"arbitrated ACK mismatch");
  if({ref0.n.bs,ref0.n.rf_wr,ref0.n.rf_rd,ref0.n.a_q,ref0.n.wd_q}!=={dut.n.bs,dut.n.rf_wr,dut.n.rf_rd,dut.n.a_q,dut.n.wd_q})$fatal(1,"bus FSM/accepted operands mismatch");
  if(k00===1'b1)begin if(r00!==r10)$fatal(1,"master0 data mismatch");if($isunknown(r00))unknown_read_words++;end
  if(k01===1'b1)begin if(r01!==r11)$fatal(1,"master1 data mismatch");if($isunknown(r01))unknown_read_words++;end
  if({ref0.n.raw_frame_accept,ref0.n.point_frame_admit,ref0.n.pt_ready,ref0.n.pf_ready,ref0.n.mt_eval_st,ref0.n.pose_wait,ref0.n.mb_shadow_read}!==
     {dut.n.raw_frame_accept,dut.n.point_frame_admit,dut.n.pt_ready,dut.n.pf_ready,dut.n.mt_eval_st,dut.n.pose_wait,dut.n.mb_shadow_read})$fatal(1,"RAW/readiness/control cycle mismatch");
  if({ref0.valid,ref0.stop_,ref0.wdog,ref0.seq}!=={dut.valid,dut.stop_,dut.wdog,dut.seq})$fatal(1,"SAFE publication/control mismatch");
  if(ref0.valid===1'b1 && {ref0.vx,ref0.vy,ref0.wz,ref0.mode}!=={dut.vx,dut.vy,dut.wz,dut.mode})$fatal(1,"SAFE published data mismatch");
  if(q0&&q1)arbitration++;
 end
 task automatic bus0(input bit wr,input[15:0]a,input[31:0]d,input integer held);
  integer n;begin @(negedge clk);#1;q0=1;w0=wr;a0=a;d0=d;n=0;
   do begin @(negedge clk);n++;if(n>500)$fatal(1,"master0 timeout %h",a);end while(k00!==1'b1);
   repeat(held)@(negedge clk);#1;q0=0;repeat(4)@(negedge clk);transactions++;
  end
 endtask
 task automatic bus1(input bit wr,input[15:0]a,input[31:0]d);
  integer n;begin @(negedge clk);#1;q1=1;w1=wr;a1=a;d1=d;n=0;
   do begin @(negedge clk);n++;if(n>500)$fatal(1,"master1 timeout %h",a);end while(k01!==1'b1);
   #1;q1=0;repeat(4)@(negedge clk);transactions++;
  end
 endtask
 initial begin
  for(integer b=0;b<6;b++)pulses[b]=0;
  repeat(5)@(negedge clk);#1;compare_enabled=1;rst=0;
  for(integer alias_=0;alias_<4;alias_++)begin
   for(integer bits_=0;bits_<8;bits_++)bus0(1,16'h700c+alias_,bits_,0);
   for(integer bits_=0;bits_<4;bits_++)bus0(1,16'h3004+alias_,bits_,0);
   bus0(1,16'h2070+alias_,0,0);bus0(1,16'h2070+alias_,32'hffffffff,0);
   bus0(0,16'h700c+alias_,32'hffffffff,0);bus0(0,16'h3004+alias_,32'hffffffff,0);bus0(0,16'h2070+alias_,32'hffffffff,0);
  end
  for(integer b=0;b<16;b++)begin
   bus0(1,16'h700c^(16'h1<<b),32'hffffffff,0);
   bus0(1,16'h3004^(16'h1<<b),32'hffffffff,0);
   bus0(1,16'h2070^(16'h1<<b),32'hffffffff,0);
  end
  // Special windows must retain higher acceptance priority than ordinary RF.
  bus0(1,16'h8000,7,0);bus0(1,16'hbfff,7,0);bus0(1,16'h4100,7,0);bus0(1,16'h4400,7,0);
  bus0(1,16'hc000,7,0);bus0(1,16'hc800,7,0);bus0(1,16'hd000,7,0);bus0(1,16'hd100,7,0);
  for(integer j=0;j<32;j++)begin
   fork begin bus0(1,16'h700c,(j&7),j%3);end begin bus1(1,16'h3004,(j&3));end join
  end
  // Four-state decoder checks: unknown address/data must equal original logic;
  // these are not interpreted as accepted meaningful motion commands.
  bus0(1,16'h70xc,32'hffffffff,0);xz_cases++;
  bus0(1,16'h700c,32'h0000000x,0);xz_cases++;
  bus0(1,16'h3004,32'h0000000z,0);xz_cases++;
  // Reset with a command already visible in B_RF, then held request across reset.
  @(negedge clk);#1;q0=1;w0=1;a0=16'h2070;d0=0;
  wait(ref0.n.bs==1);@(negedge clk);#1;rst=1;q0=0;resets++;
  repeat(3)@(negedge clk);#1;rst=0;
  bus0(1,16'h700c,7,0);
  @(negedge clk);#1;rst=1;q1=1;w1=1;a1=16'h3004;d1=3;resets++;
  repeat(3)@(negedge clk);#1;rst=0;
  do @(negedge clk);while(k01!==1'b1);#1;q1=0;
  repeat(6)@(negedge clk);
  if(checks<1000||transactions<180||arbitration==0||xz_cases!=3||resets!=2)$fatal(1,"insufficient finite bus stimulus");
  for(integer b=0;b<6;b++)if(pulses[b]==0)$fatal(1,"uncovered command bit %d",b);
  $display("TEST PASSED NAV_COMMAND_BUS checks=%0d transactions=%0d resets=%0d arbitration=%0d xz=%0d unknown_read_words=%0d",checks,transactions,resets,arbitration,xz_cases,unknown_read_words);
  $finish;
 end
 initial begin #10000000;$fatal(1,"finite bus miter stimulus watchdog");end
endmodule
'''


def prepare_bus_miter(a, result, data):
    assert a.authorization == MCAST and a.candidate is not None and a.validation_out is not None
    pair = natural_pair_complete()
    candidate = a.candidate.resolve()
    out = a.validation_out.resolve()
    assert out.parent == BASE.resolve() and not out.exists() and out != candidate
    manifest, inputs, files, assets, generated, consumers, change, plan = data
    assert (candidate/NAV).read_text(encoding='utf8') == generated
    assert all(same_bytes(p, candidate/r) for r, p in inputs.items() if r != NAV)
    assert all(same_bytes(a.source/r, candidate/r) for r in assets)
    assert json.loads((candidate/'NAV_command_predecode_preparation_review.json').read_text(encoding='utf8'))['phase'] == 'source_materialized_unvalidated'
    out.mkdir()
    old = inputs[NAV].read_text(encoding='utf8')
    # The register map declares module-local localparams. Both copies in one
    # compiler invocation need those declarations; a shared include guard would
    # otherwise suppress the second module's constants. Only simulation copies
    # receive this preprocessor directive; the frozen hardware stays unchanged.
    def comparison_module(text, name):
        renamed = once(text, 'module nav_subsystem #(', f'module {name} #(')
        return once(renamed, '`include "nav_regs_map.vh"',
                    '`undef NAV_REGS_MAP_VH\n`include "nav_regs_map.vh"')
    (out/'nav_reference.v').write_text(comparison_module(old, 'nav_command_reference'), encoding='utf8', newline='\n')
    (out/'nav_candidate.v').write_text(comparison_module(generated, 'nav_command_candidate'), encoding='utf8', newline='\n')
    wrappers = WRAPPER.replace('@NAME@', 'nav_command_reference_bus').replace('@NAVMODULE@', 'nav_command_reference')
    wrappers += WRAPPER.replace('@NAME@', 'nav_command_candidate_bus').replace('@NAVMODULE@', 'nav_command_candidate')
    (out/'bus_wrappers.sv').write_text(wrappers, encoding='utf8', newline='\n')
    (out/'tb_NAV_command_predecode.sv').write_text(TB, encoding='utf8', newline='\n')
    # Initialize no RAM. Copy only the frozen lookup asset needed by real NAV.
    shutil.copy2(a.source/'sin_lut.hex', out/'sin_lut.hex')
    sources = [str(candidate/r) for r in inputs if r.startswith('nav/') and r != NAV]
    sources += [str(out/'nav_reference.v'), str(out/'nav_candidate.v')]
    commands = [
        dict(tool='xvlog', arguments=['-i', str(candidate/'nav/rtl/regs')]+sources),
        dict(tool='xvlog', arguments=['-sv', str(out/'bus_wrappers.sv'), str(out/'tb_NAV_command_predecode.sv')]),
        dict(tool='xelab', arguments=['-debug', 'off', '-timescale', '1ns/1ps', 'tb_NAV_command_predecode', '-s', 'NAV_command_bus']),
        dict(tool='xsim', arguments=['NAV_command_bus', '-runall']),
    ]
    validation = dict(phase='finite_bus_miter_inputs_prepared_not_run',
        natural_PNR53_PNR54_completion=pair, cwd=str(out), root_authorization_before_tool_execution_required=True,
        simulation_started=False, no_force_or_RAM_init=True,
        sources='Frozen old/new NAV renamed with module-local register-map include guards reset; common RTL and two real arbiters unchanged',
        commands_not_executed=commands, expected_log='TEST PASSED NAV_COMMAND_BUS',
        scope='Finite six-pulse plus bus/control cycle equality under idle network; unknown read words are reported, not treated as verified numeric memory payloads',
        full_RAW_FIFO_MBX_console_concurrent_sequences_still_required=True,
        unknown_read_words_must_be_reviewed=True, all_source_bindings=[metadata(inputs[NAV]),metadata(candidate/NAV)],
        generated_bindings=[metadata(out/n) for n in ('nav_reference.v','nav_candidate.v','bus_wrappers.sv','tb_NAV_command_predecode.sv','sin_lut.hex')])
    write_json(out/'run_plan.json', validation)
    result.update(phase='bus_miter_inputs_materialized_not_run', RTL_generated=False,
        validation_RTL_generated=True, validation_out=str(out), validation=validation)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--plan', action='store_true')
    mode.add_argument('--materialize', action='store_true')
    mode.add_argument('--prepare-bus-miter', action='store_true')
    ap.add_argument('--source', type=Path, default=SOURCE)
    ap.add_argument('--candidate', type=Path)
    ap.add_argument('--validation-out', type=Path)
    ap.add_argument('--authorization', default='')
    ap.add_argument('--receipt', type=Path, required=True)
    a = ap.parse_args()
    a.source = a.source.resolve()
    assert a.source == SOURCE.resolve()
    assert a.receipt.suffix.lower() == '.json' and not a.receipt.exists()
    assert not a.receipt.resolve().is_relative_to(a.source)
    data = source_data(a.source)
    manifest, inputs, files, assets, generated, consumers, change, plan = data
    result = dict(recorded_at=datetime.now().astimezone().isoformat(), phase='generator_prepared_only',
        source=str(a.source), generator=metadata(Path(__file__).resolve()),
        helper=metadata(Path(__file__).with_name('prepare_ring_stride32_private_reader.py')),
        plan_binding=metadata(PLAN), source_inputs=[metadata(p) for p in inputs.values()],
        expected_changed_RTL=[NAV], actual_3RTL_ring_candidate_adopted=False,
        source_diff_contract=change, affected_consumers=consumers,
        extra_source_pulse_bits=6, physical_FF_delta_unknown=True, extra_RAM_target=0,
        frame_and_bus_extra_cycles=0, inherited_assets=69, packed_weight_files=35, all_assets=104,
        firmware_BIN_bytes=53572, unchanged_source_RTL=62,
        old_pose_only_generator_used=False, RTL_generated=False, validation_RTL_generated=False,
        simulation_started=False, EDA_started=False, SYN_started=False, PNR_started=False,
        automatic_SYN=False, automatic_PNR=False, deployment_accepted=False,
        measured_basis=dict(address7_failing_endpoints=2546,
            representative_data_ns=plan['timing_limit']['path24_data_delay_ns'],
            listed_decode_prefix_ns=plan['timing_limit']['listed_first3_decode_gate_and_route_delay_ns'],
            illustrative_unchanged_suffix_ns=plan['timing_limit']['unchanged_suffix_arithmetic_ns'],
            interpretation='Removing the prefix alone leaves about24.4ns; no50MHz guarantee, route or physical closure claim'),
        exact_capture_contract=dict(
            pulse_clear='Same always block and unconditional every-edge0 default as rf_wr; reset and stalls emit no command',
            pulse_set='Only original B_IDLE final ordinary-RF write branch with aligned live bus_addr and bus_wdata; preserve all special-window priority',
            four_state='Use equality result bitwise AND payload bit; never procedural if(address_match). Bus write if retains original X behavior.',
            E0='a_q/wd_q/rf_wr plus six pulses registered; no consumer event/ACK moved earlier',
            E1='Existing consumers see old pulse/operands and RF ACK/rdata; clear vector together with rf_wr',
            E2='Original B_END bubble and arbiter hold retained',
            independent_command_bits=True,
            RAW_ownership_and_frame_start='Original RAW acceptance trigger and WAIT_POINTS/read_admit retained byte-for-byte',
            stop='No changes to motion/support/SAFE/lease/IR priorities or actuator clock paths'),
        materialization_gate='Both exact PNR53/54 natural ends and original tools exited; explicit token plus unique output; run.tcl disabled with immediateerror/no run command',
        validation_entry='Same generator --prepare-bus-miter after candidate creation; writes renamed exact old/new NAV, real arbiter wrappers, finite TB and run_plan.json only, never starts tools',
        validation_scope=dict(basic='All independent commandbit combinations, aliases, reads, nearby addresses, window writes, two masters/held requests/reset/XZ; every-edge original bus/command/RAW readiness/SAFE publication comparisons',
            still_required=plan['required_future_proofs'],
            limits='Basic miter holds both MAC RX idle; it does not exercise simultaneous real RAW/SOF/point drain, MBX variable wait traffic or CPU console. Those require the existing full fixtures extended before mapped/physical acceptance.'),
        all_original_code_outside_six_pulse_transform_restored_exactly=True,
        future_receipts='Copied historical JSON receipts remain ancestor provenance; root source/mapped gates must be rerun for new candidate')
    if a.materialize:
        assert a.candidate is not None and a.validation_out is None
        result = materialize(a, result, data)
    elif a.prepare_bus_miter:
        result = prepare_bus_miter(a, result, data)
    else:
        assert a.candidate is None and a.validation_out is None and not a.authorization
        # Check only Python template placeholders now; no Verilog file emitted.
        assert WRAPPER.count('@NAME@') == WRAPPER.count('@NAVMODULE@') == 1
        assert 'TEST PASSED NAV_COMMAND_BUS' in TB
    a.receipt.parent.mkdir(parents=True, exist_ok=True)
    write_json(a.receipt, result)
    print(json.dumps({k:result[k] for k in ('phase','RTL_generated','validation_RTL_generated',
        'expected_changed_RTL','all_assets','unchanged_source_RTL','simulation_started','EDA_started')},ensure_ascii=False))


if __name__ == '__main__':
    main()
