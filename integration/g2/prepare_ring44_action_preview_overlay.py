"""Rebase the proved private-product action edge onto frozen SYN44 ring RTL.

Only three independent overlay files are written. The actual scheduler state
logic, its ring LOAD interface and both sequential always blocks remain literal.
Source-level bit-vector edge proofs are run; no RTL simulator or EDA is invoked.
"""
from pathlib import Path
from datetime import datetime
import argparse,difflib,json,re
from prepare_baseline_fusion_retime import metadata,save
from prepare_action_preview_product import PRODUCTS
from prove_soc_console_remap import z3

BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
SOURCE=BASE/'SLF128_ring_stride32_SYN44_prepared1'
PROVED=BASE/'action_preview_product_candidate1'
NAMES=['control/policy_action_map.v','control/rl_policy_tick_v5.v','control/mem_policy.v']

def read(p):return p.read_text(encoding='utf-8-sig')
def once(t,a,b):
    assert t.count(a)==1,(a,t.count(a));return t.replace(a,b)

def rebase(source):
    proof=json.loads((PROVED/'prepared_review.json').read_text())
    arithmetic=json.loads((PROVED/'arithmetic_result.json').read_text())
    scheduler=json.loads((PROVED/'policy_result.json').read_text())
    assert arithmetic['passed']and scheduler['passed']and proof['changed_RTL']==NAMES
    assert scheduler['unchanged_actual_scheduler']and scheduler['source_frame_ownership_relation_checked']
    for row in proof['source_bindings']+proof['candidate_bindings']:
        assert metadata(Path(row['path']))==row,('originalproofbindingchanged',row)
    current={n:read(source/n)for n in NAMES};changed=dict(current)
    assert(source/NAMES[0]).read_bytes()==(Path(proof['source'])/NAMES[0]).read_bytes(), 'current map arithmetic differs from originally proved source'
    t=current[NAMES[0]];t=once(t,'    input wire [143:0] actions,','    input wire [143:0] actions,\n    input wire [143:0] preview_actions,')
    a=t.index('    function signed [31:0] scale;');b=t.index('    function [15:0] pulse;',a)
    oldscale=t[a:b];assert 'product=$signed(clip(a))*gain;'in oldscale
    t=t[:a]+PRODUCTS+t[b:]
    pairs=[('48','-600','lw'),('120','600','rw'),('0','19','lh'),('24','-19','lk'),('72','19','rh'),('96','19','rk')]
    for index,gain,name in pairs:t=once(t,'scale(actions['+index+'+:24],'+gain+')','rounded_product(product_'+name+')')
    assert t==read(PROVED/NAMES[0]),'rebase map must be the exact already-proved private64product source'
    changed[NAMES[0]]=t
    t=current[NAMES[1]]
    t=once(t,'    output reg [143:0] actions,','    output reg [143:0] actions,\n    output wire [143:0] preview_actions,')
    anchor='    wire signed [23:0] a0,a1,a2,a3,a4,a5;'
    t=once(t,anchor,anchor+'\n    assign preview_actions={a5,a4,a3,a2,a1,a0};')
    # The exact inverse establishes no accidental edits to the ring, counter,
    # deadline, fault, diagnostics, action publication, or read cursor logic.
    inverse=t.replace('    output wire [143:0] preview_actions,\n','').replace('\n    assign preview_actions={a5,a4,a3,a2,a1,a0};','')
    assert inverse==current[NAMES[1]]
    marker='    always @* begin';assert t[t.index(marker):]==current[NAMES[1]][current[NAMES[1]].index(marker):]
    assert t.count('always @(posedge clk or negedge rst_n)')==2
    for anchor in('frame_read_peek_first','frame_read_advance','frame_read_data','snapshot_word<=frame_read_data',
                   "if(load_index!=167) snapshot_word<=frame_read_data;",'OBS_RING_STORAGE'):
        assert t.count(anchor)==current[NAMES[1]].count(anchor),(anchor,'ring changed')
    changed[NAMES[1]]=t
    t=current[NAMES[2]];t=once(t,'    wire [143:0] actions;','    wire [143:0] actions;\n    wire [143:0] action_preview;')
    t=once(t,'.capture(capture),.action_valid(action_valid),.actions(actions),',
               '.capture(capture),.action_valid(action_valid),.actions(actions),.preview_actions(action_preview),')
    t=once(t,'.run(run),.action_valid(action_valid),.actions(actions),',
               '.run(run),.action_valid(action_valid),.actions(actions),.preview_actions(action_preview),')
    inverse=t.replace('    wire [143:0] action_preview;\n','').replace('.preview_actions(action_preview),','')
    assert inverse==current[NAMES[2]]
    changed[NAMES[2]]=t
    # Publication predicate used below is bound to the actual unchanged code.
    tick=current[NAMES[1]]
    for anchor in('wire deadline_missed = tick_count==PERIOD_CLKS-1 && state!=IDLE;',
      'capture<=0; action_valid<=0;','if (!enable) begin','end else if (!sensor_ok || fault!=0) begin',
      'if (!(tick_count==PERIOD_CLKS-1 && state!=IDLE)) begin','WAIT_CORE: if (core_done) begin',
      'if (core_errors!=0 || core_sat || result_id!=accepted_id) begin',
      'actions<={a5,a4,a3,a2,a1,a0}; action_valid<=1;'):
        assert anchor in tick,anchor
    return current,changed,dict(old_source_review=metadata(PROVED/'prepared_review.json'),
      old_arithmetic=metadata(PROVED/'arithmetic_result.json'),old_scheduler=metadata(PROVED/'policy_result.json'),
      arithmetic_summary=arithmetic['summary'],scheduler_summary=scheduler['summary'],
      old_proof_scope=scheduler['scope'],actual_original_map_bytes_equal=True,
      new_map_exactly_equal_to_old_proved_candidate=True,
      complete_scheduler_after_read_cursor_always_byte_identical=True,
      both_sequential_scheduler_blocks_byte_identical=True,
      complete_mem_policy_inverse_patch_is_original=True,
      ring_params_ports_LOAD_word_and_cursor_all_unchanged=True)

def edge_proof(out):
    tests=[]
    def prove(name,bad,assumptions=()):
        s=z3.Solver();s.add(*assumptions,bad);r=s.check()
        tests.append(dict(name=name,result=str(r)))
        assert r==z3.unsat,(name,str(s.model())if r==z3.sat else str(r))
    bus=z3.BitVec('arbitrary_core_bus_at_capture_E',144)
    action_old=z3.BitVec('previous_registered_actions',144)
    nonpublished_actions=z3.BitVec('arbitrary_nonpublished_next_actions',144)
    rst,enable,healthy,done,sat=z3.Bools('rst_n scheduler_enable sensor_ok core_done core_sat')
    state=z3.BitVec('state3',3);fault=z3.BitVec('fault3',3)
    tick=z3.BitVec('tick_count32',32);period=z3.BitVec('period32',32)
    errors=z3.BitVec('core_errors5',5);rid=z3.BitVec('result_id32',32);aid=z3.BitVec('accepted_id32',32)
    deadline=z3.And(tick==period-1,state!=0)
    pub=z3.And(rst,enable,healthy,fault==0,z3.Not(deadline),state==4,done,errors==0,z3.Not(sat),rid==aid)
    actions_next=z3.If(pub,bus,nonpublished_actions)
    valid_next=pub
    # No core-bus stability or E+1 relationship is assumed: the capture edge
    # samples exactly the same pre-edge coreFF tuple as the existing scheduler.
    gains=(19,-19,-600,19,19,600)
    def clip(a):return z3.If(a>4096,z3.BitVecVal(4096,24),z3.If(a< -4096,z3.BitVecVal(-4096,24),a))
    def product(a,g):return z3.SignExt(40,clip(a))*z3.BitVecVal(g,64)
    def rounded(p):return z3.Extract(31,0,z3.If(p<0,-((-p+2048)>>12),(p+2048)>>12))
    proves=[]
    for lane,gain in enumerate(gains):
        captured=product(z3.Extract(lane*24+23,lane*24,bus),gain)
        from_actions=product(z3.Extract(lane*24+23,lane*24,actions_next),gain)
        prove('valid_Eplus1_implies_captured_product_of_same_published_action_lane'+str(lane),
              z3.And(valid_next,captured!=from_actions))
        # Arbitrary stored private state is legal while valid0. This is the
        # inductive invariant, established by reset-valid0 then each capture.
        stored=z3.BitVec('private_product64_lane'+str(lane),64)
        old_action=z3.Extract(lane*24+23,lane*24,action_old)
        inv=z3.Implies(z3.Bool('registered_action_valid'),stored==product(old_action,gain))
        v=z3.Bool('registered_action_valid');run2,reset2=z3.Bools('actuator_run_Eplus1 reset_n_Eplus1')
        hold=z3.BitVec('equal_output_hold_lane'+str(lane),32)
        olddelta=z3.If(z3.And(reset2,run2,v),rounded(product(old_action,gain)),hold)
        newdelta=z3.If(z3.And(reset2,run2,v),rounded(stored),hold)
        prove('consume_Eplus1_exact_rounding_or_hold_lane'+str(lane),olddelta!=newdelta,(inv,))
        if lane in(0,1,3,4):
            ff=z3.BitVec('latest_ff12_lane'+str(lane),12);zero=z3.BitVec('zero_parameter32_lane'+str(lane),32)
            def pulse(delta):
                total=zero+z3.SignExt(20,ff)+delta
                return z3.If(total<500,z3.BitVecVal(500,32),z3.If(total>2500,z3.BitVecVal(2500,32),total))
            prove('latest_consume_edge_ff_and_pulse_identical_lane'+str(lane),pulse(olddelta)!=pulse(newdelta),(inv,))
    # Actual reset/default priority: action_valid is cleared before any case
    # branch and can become1 only by the gated publication above.
    scheduler_valid_after=z3.If(z3.Not(rst),z3.BoolVal(False),pub)
    prove('reset_valid0_needs_no_private_reset',z3.And(z3.Not(rst),scheduler_valid_after))
    prove('publication_rejected_on_deadline_even_with_done',z3.And(pub,deadline))
    prove('publication_rejected_on_fault_or_error_or_bad_id',z3.And(pub,z3.Or(fault!=0,errors!=0,sat,rid!=aid)))
    result=dict(passed=True,checks=tests,known_binary_core_actions_domain_bits=144,
      proof='inductive valid→private_product=P(registered_actions); captureE samples arbitrary same core bus in bothalways; consumeE+1 samples oldproduct/oldactions plus latestff on originaledge',
      no_assumption_core_bus_stable_at_Eplus1=True,no_inference_stub=True,tests_or_EDA_started=False,
      initial_state='actual scheduler reset action_valid0 makes invariant vacuous; private384bits may be arbitrary until firstclock. Globalasyncreset clearsvalid andpublicoutputs, not privateproducts',
      four_state_limit='Z3 is two-state; unchangedSV reset/priority and unusedprivateunknownsemantics are sourcebound, not claimed completeXZformalproof')
    save(out/'edge_relation_proof.json',result);return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,default=SOURCE)
    p.add_argument('--out',type=Path,required=True);a=p.parse_args();assert not a.out.exists()
    current,changed,binding=rebase(a.source);a.out.mkdir()
    patches=[]
    for rel in NAMES:
        dest=a.out/rel;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(changed[rel],encoding='utf8',newline='\n')
        if rel==NAMES[0]:assert dest.read_bytes()==(PROVED/rel).read_bytes()
        patches+=difflib.unified_diff(current[rel].splitlines(True),changed[rel].splitlines(True),fromfile='SYN44/'+rel,tofile='overlay/'+rel)
    (a.out/'action_preview_rebase.patch').write_text(''.join(patches),encoding='utf8')
    proof=edge_proof(a.out)
    old_mapping=BASE/'baseline_fusion_action_preview_one_shot/actual_preview_product_mapping.json'
    mapping=json.loads(old_mapping.read_text());assert mapping['passed']and mapping['actual_products_absorbed_in_DSP_output_registers']
    inputs=[metadata(a.source/n)for n in NAMES]+[metadata(PROVED/n)for n in NAMES]
    record=dict(at=datetime.now().astimezone().isoformat(),passed=True,phase='source_closed_independent_overlay_no_EDA',
      source=str(a.source),overlay=str(a.out),modified_RTL=NAMES,source_bindings=inputs,
      overlay_bindings=[metadata(a.out/n)for n in NAMES],source_semantics=binding,edge_proof=metadata(a.out/'edge_relation_proof.json'),
      new_source_bitvector_checks=len(proof['checks']),
      source_diff='same proven action_map384privateproductbits; scheduler only previewoutputport+directcorebusassign; mem_policy only privatewire+twoportconnections',
      unchanged=['ringstride32observer/RAM/LOADcursor/peek/freeze/advance','allpublicschedulerstate/period400000/fault/deadline/retry/diagnostics',
        'actualcore andall4500weights','allFFinputs/legpulseclamp/last_actions/publicreset-stop/hold-outputedge',
        'CPUFW/assets/capacities/CST/SDC/clock/CDC/IR/OLED; overlay containsnoneofthosefiles'],
      original_numeric_reused=dict(arithmetic=binding['arithmetic_summary'],actual_scheduler_with_explicit_stub=binding['scheduler_summary'],
        not_reexecuted=True,numeric_map_byte_identical=True,full4500_ring_existing_results_unchanged_but_not_rerun=True),
      SYN28_only_reference=dict(binding=metadata(old_mapping),private_product_source384bits=True,
        six_DSP_OREG_absorption_was_measured=True,not_current_mapping_or_timing=True),
      required_new_actual_netlist_checks=[
        'sixproducts originate from originalpolicy/core/action0..5, not registeredpolicy/actionsoneedge later',
        'sixoriginalsigned64clip×gain products: DSPoutputregister CLK0=actual50MHz/CEconstant1/RESETconstant0, otherinput/pipelinelevelsBYPASS; ifmappedotherwise readactualstageandprove',
        'no extraactuatorpublicationdelay; scheduleractions/valid boundaryandlatestff capturestilloriginal',
        'allring78RAM/pointers/readcursor/stoplogic fromSYN44unchanged exceptexpectedcontextmapping; allotherRAMINIT/CPU/2trueclocks/original26CDC',
        'actualresourcesandcompleteSYN/PNRnegativepaths; 22.610ns action→lh_us is SYNestimateonly, not proofthiscandidateclosesphysical50MHz',
        'ifnewphysicalmapping alters numeric primitive mode,bindallactualA/B/C/D/CLK/CE/reset/OREG andprove rather thanassumingoldSYN28map'],
      source_level_limit='completeSVfour-stateandphysicaltimingnotproved; old8770frame/42000cycletestsreusedonlywherecurrentbytesandsequentiallogicbound',
      automatic_SYN=False,automatic_PNR=False,RTL_simulation_started=False,firmware_recompiled=False,
      old_failed_netlist_or_oldglobal_constraints_imported=False,deployment_ready=False)
    save(a.out/'prepared_review.json',record)
    save(REPO/'docs/4_design/evidence/g2_20261023/Ring44_action_preview_source_closure_20261027.json',record)
    print(json.dumps(dict(output=str(a.out),passed=True,checks=len(proof['checks']),changed_RTL=NAMES,EDA=False)))

if __name__=='__main__':main()
