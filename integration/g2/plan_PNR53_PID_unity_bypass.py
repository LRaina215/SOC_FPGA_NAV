"""Read actual PID paths and plan a reachable-domain unity-gain bypass.

Default writes JSON/text evidence only. RTL is retained as Python strings until
an explicit --materialize after the original PNR54 natural-completion gate.
No EDA or mathematical/RTL equivalence test is started by this entry point.
"""
from pathlib import Path
from datetime import datetime
import argparse,difflib,json,re
from prepare_baseline_fusion_retime import metadata,save
from prepare_FW_sin_ROM_RTL_overlay import gate54
from analyze_pnr53_failed_endpoints import parse_path
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
HW=BASE/'SLF128_storage_safeFW_SYN43_prepared1'
PNR=BASE/'SLF128_storage_place1_pnr53'

ORIGINAL_TARGETS='''    wire signed [31:0] wheel_left_target_mmps=MODEL_ENABLE ?
        (amplify_model_wheels ? amplified_model_mmps(model_left_mmps) : model_left_mmps) : LEFT_TARGET_MMPS;
    wire signed [31:0] wheel_right_target_mmps=MODEL_ENABLE ?
        (amplify_model_wheels ? amplified_model_mmps(model_right_mmps) : model_right_mmps) : RIGHT_TARGET_MMPS;
'''
UNITY_TARGETS='''    wire signed [31:0] wheel_left_target_mmps,wheel_right_target_mmps;
    // These model outputs are reset0/hold/or clip(action)*+-600 and therefore
    // stay within[-600,600]. At effectiveQ12 gain4096 the original amplification
    // round/clamp is exactly identity. No ESTOP or PID enable path is pipelined.
    generate if(MODEL_WHEEL_GAIN_Q12==32'sd4096) begin:g_unity_model_wheel_gain
        assign wheel_left_target_mmps=MODEL_ENABLE ? model_left_mmps : LEFT_TARGET_MMPS;
        assign wheel_right_target_mmps=MODEL_ENABLE ? model_right_mmps : RIGHT_TARGET_MMPS;
    end else begin:g_original_model_wheel_gain
        assign wheel_left_target_mmps=MODEL_ENABLE ?
            (amplify_model_wheels ? amplified_model_mmps(model_left_mmps) : model_left_mmps) : LEFT_TARGET_MMPS;
        assign wheel_right_target_mmps=MODEL_ENABLE ?
            (amplify_model_wheels ? amplified_model_mmps(model_right_mmps) : model_right_mmps) : RIGHT_TARGET_MMPS;
    end endgenerate
'''

def read_actual_paths(tr):
    wanted={146,298};found={};current=None;buf=[];active=False
    before=metadata(tr)
    with tr.open(encoding='utf8')as f:
        for line in f:
            if line.rstrip()=='3.3.1 Setup Analysis Report':active=True
            if not active:continue
            if line.rstrip()=='3.3.2 Hold Analysis Report':
                if current in wanted:found[current]=''.join(buf)
                break
            m=re.fullmatch(r'\s*Path(\d+)\s*',line)
            if m:
                if current in wanted:found[current]=''.join(buf)
                current=int(m[1]);buf=[]
            if current in wanted:buf.append(line)
    assert wanted==set(found) and before==metadata(tr)
    return found

def path_cells(vo,wanted):
    before=metadata(vo);cells={};active=None;buf=[];start=0
    with vo.open(encoding='utf8')as f:
        for number,line in enumerate(f,1):
            m=re.match(r'^(\w+)\s+\\(\S+)\s+\(',line)
            if m and m[2]in wanted:active=m[2];buf=[line];start=number
            elif active:
                buf.append(line)
                if line.strip()==');':
                    text=''.join(buf);cells[active]=dict(line=start,text=text,
                        ports=dict(re.findall(r'\.(\w+)\((.*?)\)',text,re.S)),parameters=[]);active=None
            m=re.match(r'^defparam\s+\\(\S+)\s+\.(.*?);',line)
            if m and m[1]in cells:cells[m[1]]['parameters'].append(m[2])
    assert wanted<=set(cells),(wanted-set(cells));assert before==metadata(vo)
    return cells

def run(out,materialize):
    assert not out.exists()
    gating=gate54(materialize)
    core=HW/'control4500_core.v';top=HW/'g2_top.v';act=HW/'control/policy_action_map.v';mem=HW/'control/mem_policy.v';pid=HW/'control/motor520_closed_loop.v'
    c=core.read_text();t=top.read_text();a=act.read_text();m=mem.read_text();p=pid.read_text()
    assert c.count(ORIGINAL_TARGETS)==1 and 'parameter real MODEL_WHEEL_SPEED_GAIN=1.0' in c
    assert 'MODEL_WHEEL_GAIN_Q12=$rtoi(MODEL_WHEEL_SPEED_GAIN*4096.0+0.5)'in c
    assert 'control4500_core u_control('in t and 'defparam u_control'not in t
    # No alternate software/IR writer to the register outputs: the visible
    # source topology binds them directly to this action-map output pair.
    assert '.target_left_mmps(model_left_mmps),.target_right_mmps(model_right_mmps)'in c
    assert '.left_mmps(target_left_mmps),.right_mmps(target_right_mmps)'in m
    assert "left_mmps<=scale(actions[48+:24],-600);"in a and 'right_mmps<=scale(actions[120+:24],600);'in a
    assert a.count('left_mmps<=0;right_mmps<=0;')==2
    assert 'left_mmps<='not in a[a.index('for(i=0;i<6;'):]
    for anchor in('wire command_emergency_stop=EMERGENCY_STOP || (IR_ENABLE && ir_emergency_stop);',
        'wire control_run=command_run && !command_emergency_stop;',
        'wire wheel_control_enable=MOTOR_ENABLE && control_run && (!MODEL_ENABLE || model_active);'):
        assert anchor in c,anchor
    patched=c.replace(ORIGINAL_TARGETS,UNITY_TARGETS)
    assert patched[patched.index('    motor520_closed_loop #('):]==c[c.index('    motor520_closed_loop #('):]
    assert patched.count('!command_emergency_stop')==c.count('!command_emergency_stop')
    tr=PNR/'impl/pnr/g2_top.tr';vo=PNR/'impl/pnr/g2_top.vo';raw=read_actual_paths(tr)
    details={str(n):parse_path(s)for n,s in raw.items()}
    wanted={r['node'].rsplit('/',1)[0]for v in details.values()for r in v['actual_data_rows']}
    cells=path_cells(vo,wanted)
    early=cells['u_control/telemetry_thread/n1726_s9'];shared=cells['u_control/telemetry_thread/n1726_s4']
    assert 'ir_emergency_stop_2'in early['ports']['I0'] and shared['ports']['I3']==early['ports']['F']
    assert cells['u_control/wheel_control/right_control/n1321_s8']['ports']['I1']==shared['ports']['F']
    assert cells['u_control/wheel_control/left_control/n1321_s9']['ports']['I1']==shared['ports']['F']
    classification=BASE/'PNR53_all_failed_endpoints_classification1/result.json'
    classes=json.loads(classification.read_text())['classes'];classes=[r for r in classes if r['category']in('control_left_PID','control_right_PID')]
    assert sorted(r['negative_unique_endpoints']for r in classes)==[362,381]
    out.mkdir()
    for n,s in raw.items():
        assert s==(BASE/f'PNR53_all_failed_endpoints_classification1/worst_class_path{n}.txt').read_text()
        (out/f'actual_TR_path{n}.txt').write_text(s,encoding='utf8')
    save(out/'actual_path_primitives.json',cells)
    result=dict(at=datetime.now().astimezone().isoformat(),phase='source/math induction and actual path plan only; not an equivalence run',
        source_inputs=[metadata(x)for x in(core,top,act,mem,pid,classification,tr,vo)],gate54=gating,
        actual_PID_failed_endpoints=dict(right=362,left=381,total=743,not_a_claim_all743_will_be_fixed=True),
        actual_path_details=details,
        path_interpretation=['same registered IR stop copy atR62C126 feeds telemetry-named shared combinational LUTs, not a UART receive stage',
            'LUTn1726_s9 I0 directly takes IR stop and its F drives LUTn1726_s4 I3',
            'shared n1726_s4 F drives actual left/right n1321 target/amplification selection cones',
            'next_reference and running/opposite/stall predicates feed PID state/ramp CE',
            'right26.887ns data includes8.661ns gates+17.882ns routing+0.344nsC2Q; largest net5.186ns',
            'left25.904ns data includes8.625ns gates+16.934ns routing+0.344nsC2Q; largest net3.968ns'],
        actual_elaborated_gain=dict(top_instance='control4500_core u_control has no parameter overrides/defparam',
            default_real=1.0,compiled_expression='$rtoi(MODEL_WHEEL_SPEED_GAIN*4096.0+0.5)',effective_Q12=4096),
        current_amplifier_math=dict(product='signed64 x*gain',round='p<0 ? -((-p+2048)>>>12) : ((p+2048)>>>12)',
            saturation='signed32 output limited[-600,600]',
            unity_identity='for every known signed32 x within[-600,600], x*4096 is exact multiple4096 and symmetric rounding returnsx; saturation does not change it',
            invalid_generalization='for arbitrary signed32 x outside this reachable model domain the amplifier is NOT identity'),
        inductive_model_range=dict(base='policy_action_map reset or !run explicitly assigns both wheel registers0',
            accepted_action='clip(signed24 action) lies[-4096,4096]; signed64 multiplication by+-600 lies[-2457600,2457600]; symmetric rounding(/4096) lies[-600,600]',
            other_edge='if reset deasserted,run1,action_valid0, both output registers hold their previous values',
            source_topology='mem_policy targets are directly these register outputs; no NAV/MMIO/UART/IR alternate register writer exists',
            no_unconditional_arbitrary_state_or_XZ_equivalence_claim=True,
            required_initial_state='known original reset/INIT and finite real inputs; X/unknown behavior needs explicit rejection/contract rather than hiding it'),
        only_proposed_change='when integer MODEL_WHEEL_GAIN_Q12==4096 explicitly wire model wheel targets directly; MODEL_ENABLE0 fixed targets and every nonunity expression remain original',
        preservation=['same public wheel targets and telemetry on the proven reachable domain, not a private post-telemetry mux',
            'IR emergency_stop latch, command_emergency_stop, control_run and wheel_control_enable remain literal original',
            'original PID/PWM per-cycle enable and reset priority, fault handling and all sample/update/output clocks unchanged',
            'original model external fault, servo last-pose behavior, UART telemetry and no software parallel owner retained',
            'no FF/clock/RAM/weight/capacity change and no new timing exception intended'],
        required_low_cost_proofs_before_full_build=['establish all8193 clipped values and1201 model speeds unity identity; apply invariant to both signs and reset/invalid/hold edges',
            'sequential miter full original action-map and unchanged PID/PWM for arbitrary valid actions, IR transitions at every PID state/sample edge, zero/fault/re-enable',
            'keep all nonunity parameters as original expressions; include representative negative/fractional gain cases if any materializer edits more than the explicit unity branch',
            'reject unknown outputs rather than compare X as equal; no force/internal state substitute'],
        required_actual_mapping_and_physical_checks=['stop must remain direct same-cycle disable/clear influence; no extra ESTOP register or multicycle/false path',
            'actual model target cone must no longer use IR amplification select atunity; inspect drivers rather than auto-name disappearance',
            'new full setup endpoint coverage against original362right/381left plus all other negative categories, not just worst two paths',
            'cell/route delay and actual global/routing compare to completed53 AND54; do not promise that saving LUTs makes PNR succeed',
            'all original memory INIT, clocks/26CDC/control weights/stop/reset/PWM preserved'],
        previous_failures_scope='not a whole32bit arbitrary-input identity and not proof that the whole design meets50MHz; NAV/CPU and other control failures remain',
        mathematical_test_executed=False,RTL_materialized=materialize,EDA_started=False,PNR_inputs_modified=False,deployment_ready=False)
    if materialize:
        (out/'control4500_core.v').write_text(patched,encoding='utf8')
        result['materialized_file']=metadata(out/'control4500_core.v')
    else:assert not list(out.rglob('*.v'))
    save(out/'result.json',result)
    if not materialize:save(REPO/'docs/4_design/evidence/g2_20261023/PNR53_PID_unity_bypass_readonly_plan_20261026.json',result)
    print(json.dumps(dict(output=str(out),actual_paths=2,failed_endpoints=743,materialized=materialize,EDA=False)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path);p.add_argument('--materialize',action='store_true');a=p.parse_args()
    if a.materialize:assert a.out is not None,'explicit new output required'
    run(a.out or BASE/'PNR53_PID_unity_bypass_plan1',a.materialize)
