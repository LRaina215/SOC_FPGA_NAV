"""Stage the original signed64 action product through an existing publication edge.

This isolated candidate exposes the unchanged core action bus privately.  The
normal registered actions/valid interface and actuator publication edge remain.
No running build, firmware, clock constraint or public FPGA port is changed.
"""
import argparse
from datetime import datetime
import difflib
import json
from pathlib import Path
import re
import shutil
import subprocess

from prepare_baseline_fusion_retime import metadata, save

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_only_syn23'
ROOT=Path(__file__).resolve().parents[2]
VIV=Path('E:/2026.1/Vivado/bin')
PRODUCTS='''    // Speculative private data.  The unchanged scheduler publishes actions
    // from this same preview on the capture edge; action_valid is consumed one
    // edge later.  No output may consume these registers without that valid.
    // No private data reset is needed; the original output reset/stop remains.
    reg signed [63:0] product_lh,product_lk,product_lw;
    reg signed [63:0] product_rh,product_rk,product_rw;
    always @(posedge clk) begin
        product_lh <= $signed(clip(preview_actions[0+:24]))*19;
        product_lk <= $signed(clip(preview_actions[24+:24]))*(-19);
        product_lw <= $signed(clip(preview_actions[48+:24]))*(-600);
        product_rh <= $signed(clip(preview_actions[72+:24]))*19;
        product_rk <= $signed(clip(preview_actions[96+:24]))*19;
        product_rw <= $signed(clip(preview_actions[120+:24]))*600;
    end
    function signed [31:0] rounded_product;
        input signed [63:0] product;
        begin
            rounded_product=product<0 ? -((-product+2048)>>>12) : ((product+2048)>>>12);
        end
    endfunction
'''

def prepare(out):
    assert not out.exists();out.mkdir(parents=True)
    names=['control/policy_action_map.v','control/rl_policy_tick_v5.v','control/mem_policy.v']
    originals={n:(SOURCE/n).read_text()for n in names};changed=dict(originals)
    key=names[0];text=changed[key]
    assert text.count('    input wire [143:0] actions,')==1
    text=text.replace('    input wire [143:0] actions,','    input wire [143:0] actions,\n    input wire [143:0] preview_actions,',1)
    start=text.index('    function signed [31:0] scale;');end=text.index('    function [15:0] pulse;',start)
    old_scale=text[start:end];assert 'product=$signed(clip(a))*gain;'in old_scale
    text=text[:start]+PRODUCTS+text[end:]
    for expr,product in [('48','lw'),('120','rw'),('0','lh'),('24','lk'),('72','rh'),('96','rk')]:
        pattern=r'scale\(actions\['+expr+r'\+:24\],-?(?:600|19)\)'
        text,count=re.subn(pattern,'rounded_product(product_'+product+')',text);assert count==1
    changed[key]=text
    key=names[1];text=changed[key]
    assert text.count('    output reg [143:0] actions,')==1
    text=text.replace('    output reg [143:0] actions,','    output reg [143:0] actions,\n    output wire [143:0] preview_actions,',1)
    anchor='    wire signed [23:0] a0,a1,a2,a3,a4,a5;'
    assert text.count(anchor)==1
    text=text.replace(anchor,anchor+'\n    assign preview_actions={a5,a4,a3,a2,a1,a0};',1)
    changed[key]=text
    key=names[2];text=changed[key]
    text=text.replace('    wire [143:0] actions;','    wire [143:0] actions;\n    wire [143:0] action_preview;',1)
    assert text.count('.capture(capture),.action_valid(action_valid),.actions(actions),')==1
    text=text.replace('.capture(capture),.action_valid(action_valid),.actions(actions),','.capture(capture),.action_valid(action_valid),.actions(actions),.preview_actions(action_preview),',1)
    anchor='.run(run),.action_valid(action_valid),.actions(actions),'
    assert text.count(anchor)==1
    text=text.replace(anchor,anchor+'.preview_actions(action_preview),',1);changed[key]=text
    patches=[]
    for n in names:
        (out/n).parent.mkdir(exist_ok=True);(out/n).write_text(changed[n],newline='\n')
        patches.extend(difflib.unified_diff(originals[n].splitlines(True),changed[n].splitlines(True),fromfile='a/'+n,tofile='b/'+n))
    (out/'preview_product.patch').write_text(''.join(patches))
    # Reuse the complete existing arithmetic/stop/output comparison, adding
    # the one earlier preview edge required by the new private interface.
    tb=(ROOT/'integration/g2/tb_action_map_equivalence.sv').read_text()
    tb=tb.replace('input [143:0] actions','input [143:0] actions,input [143:0] preview')
    anchor='(clk,rst_n,run,valid,actions,ff_lh,ff_lk,ff_rh,ff_rk,new_out'
    assert tb.count(anchor)==1;tb=tb.replace(anchor,'(clk,rst_n,run,valid,actions,preview,ff_lh,ff_lk,ff_rh,ff_rk,new_out')
    tb=tb.replace('reg[143:0] actions=0;','reg[143:0] actions=0,preview=0;')
    tb=tb.replace('valid,actions,ff_lh,ff_lk,ff_rh,ff_rk);','valid,actions,preview,ff_lh,ff_lk,ff_rh,ff_rk);')
    # Preserve the new action while a non-public preview is captured.  Existing
    # old action_valid must be low on that extra setup edge in both modules.
    anchor='  rst_n=1;run=1;valid=1;frames=frames+1;'
    assert tb.count(anchor)==1
    tb=tb.replace(anchor,'  preview=actions;valid=0;rst_n=1;run=1;\n  @(negedge clk);#1;valid=1;frames=frames+1;',1)
    # Restore valid data after the final asynchronous reset at the same
    # protocol edge, without asking either side to consume stale inputs.
    tb=tb.replace('rst_n=0;run=1;valid=1;resets++','rst_n=0;run=1;valid=0;resets++')
    (out/'tb_arithmetic.sv').write_text(tb)
    (out/'map_original.v').write_text(originals[names[0]].replace('module policy_action_map #(','module action_map_original #(',1))
    result=dict(prepared=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),candidate=str(out),changed_RTL=names,
        source_bindings=[metadata(SOURCE/n)for n in names],candidate_bindings=[metadata(out/n)for n in names],
        algorithm_weights_RAM_clocks_unchanged=True,external_publication_extra_cycles=0,
        ff_still_sampled_on_original_actuator_edge=True,private_product_bits_before_mapping=384,
        private_products_have_no_reset=True,public_reset_stop_priority_unchanged=True,
        source_basis='Original real PNR32 action path: 16.231ns through multiplier output including C2Q, 13.835ns thereafter. New placements can differ; no50MHz success claimed.',
        requires_scheduler_relation='On each original action_valid consumption edge, actions equal preview sampled at preceding edge.',
        full_RL_inference_or_new_physical_acceptance=False,new_SYN_PNR_started=False)
    save(out/'prepared_review.json',result)
    return result

def simulate(out):
    commands=[('xvlog',[str(out/'map_original.v'),str(out/'control/policy_action_map.v')]),
        ('xvlog',['-sv',str(out/'tb_arithmetic.sv')]),('xelab',['tb_action_map_equivalence','-s','snap']),('xsim',['snap','-runall'])]
    for i,(tool,args)in enumerate(commands):
        with (out/f'{i}_{tool}.log').open('w')as log:
            r=subprocess.run(['cmd','/c',str(VIV/(tool+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        assert r.returncode==0,(tool,r.returncode)
    log=(out/'xsim.log').read_text();assert 'TEST PASSED'in log and 'Fatal:'not in log
    save(out/'arithmetic_result.json',dict(passed=True,summary=re.search(r'TEST PASSED[^\r\n]+',log)[0],scope='Finite isolated action map arithmetic/output-edge proof under explicit one-edge preview relation; scheduler integration and physical mapping still pending',candidate_bindings=[metadata(out/'control/policy_action_map.v'),metadata(out/'tb_arithmetic.sv')]))

def simulate_policy(out):
    proof=json.loads((out/'prepared_review.json').read_text())
    for row in proof['source_bindings']+proof['candidate_bindings']:
        assert metadata(Path(row['path']))==row
    tb=(ROOT/'integration/g2/tb_snapshot_fanout.sv').read_text()
    tb=tb.replace('wire[143:0]actions_a,actions_b;','wire[143:0]actions_a,actions_b;\n wire[143:0]preview_b;')
    anchor='capture_b,action_b,actions_b,\n';assert tb.count(anchor)==1
    tb=tb.replace(anchor,'capture_b,action_b,actions_b,preview_b,\n')
    pair=(out/'tb_arithmetic.sv').read_text().split('module tb_action_map_equivalence;',1)[0]
    assert 'input [143:0] preview'in pair
    extra='''
 reg signed[11:0] ff0=0,ff1=0,ff2=0,ff3=0;
 reg [143:0] sampled_preview;
 wire actuator_run=enable && sensor_ok && fault_a==0;
 action_map_pair p0(clk,rst_n,actuator_run,action_a,actions_a,preview_b,ff0,ff1,ff2,ff3);
 action_map_pair #(.LH(500),.LK(2500),.RH(1470),.RK(1530)) p1(clk,rst_n,actuator_run,action_a,actions_a,preview_b,ff0,ff1,ff2,ff3);
 always @(negedge clk)begin
  #2;ff0=random[11:0];ff1=random[23:12];ff2=~random[11:0];ff3=~random[23:12];
 end
 // Check the exact private ownership relation at the real unchanged
 // scheduler publication edge, including fault/deadline/retry paths.
 always @(posedge clk)begin
  sampled_preview=preview_b;
  #2;if(rst_n && action_b && actions_b!==sampled_preview)
       $fatal(1,"real scheduler published another preview/frame");
 end
'''
    anchor=' integer k;\n';assert tb.count(anchor)==1;tb=tb.replace(anchor,extra+anchor)
    (out/'tb_policy.sv').write_text(pair+tb)
    old=(SOURCE/'control/rl_policy_tick_v5.v').read_text()
    old=old.replace('RL_POLICY_TICK_V5_V_INCLUDED','PREVIEW_POLICY_ORIGINAL_V_INCLUDED').replace('module rl_policy_tick_v5 #(','module snapshot_original #(',1)
    (out/'policy_original.v').write_text(old)
    work=out/'policy_sim';work.mkdir(exist_ok=False)
    commands=[('xvlog',[str(out/'map_original.v'),str(out/'control/policy_action_map.v'),str(out/'policy_original.v'),str(out/'control/rl_policy_tick_v5.v')]),
        ('xvlog',['-sv',str(out/'tb_policy.sv')]),('xelab',['tb_snapshot_fanout','-s','policy_snap']),('xsim',['policy_snap','-runall'])]
    for i,(tool,args)in enumerate(commands):
        with (work/f'{i}_{tool}.log').open('w')as log:
            r=subprocess.run(['cmd','/c',str(VIV/(tool+'.bat')),*args],cwd=work,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        assert r.returncode==0,(tool,r.returncode)
    log=(work/'xsim.log').read_text();assert 'SNAPSHOT_FANOUT_PASS'in log and 'Fatal:'not in log
    save(out/'policy_result.json',dict(passed=True,summary=re.search(r'SNAPSHOT_FANOUT_PASS[^\r\n]+',log)[0],
        unchanged_actual_scheduler=True,source_frame_ownership_relation_checked=True,actuator_outputs_compared_each_cycle=True,
        ff_changes_every_cycle_and_sample_edge_preserved=True,parameter_sets=2,
        scope='Actual original scheduler and candidate with an explicit inference test double; transport/rejection/stop/reset and output timing only, not full original4500 inference',
        injected_cases=['core error','saturation','frame ID mismatch','deadline','enable','sensor','global reset','retry'],
        candidate_bindings=proof['candidate_bindings'],test_binding=metadata(out/'tb_policy.sv'),new_SYN_PNR_started=False))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True);parser.add_argument('--policy-only',action='store_true');args=parser.parse_args()
    import psutil
    me=psutil.Process();me.nice(psutil.IDLE_PRIORITY_CLASS);me.cpu_affinity([18])
    if args.policy_only:
        simulate_policy(args.out);print((args.out/'policy_result.json').read_text())
    else:
        prepare(args.out);simulate(args.out);print((args.out/'arithmetic_result.json').read_text())
