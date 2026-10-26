"""Prepare the unity target/PID equivalence request without writing Verilog.

All HDL below remains Python text until explicit --materialize/--run, both gated
on the original PNR54 natural completion. No Gowin, full4500, or board action.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,os,psutil,re,subprocess,time,traceback
from prepare_baseline_fusion_retime import metadata,save
from prepare_FW_sin_ROM_RTL_overlay import gate54
from plan_PNR53_PID_unity_bypass import ORIGINAL_TARGETS,UNITY_TARGETS
from retry_frozen_synthesis import memory_status
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
HW=BASE/'SLF128_storage_safeFW_SYN43_prepared1';VIV=Path('E:/2026.1/Vivado/bin')

TARGET_HEADER='''module TARGET_NAME #(
 parameter MODEL_ENABLE=1,IR_ENABLE=1,FIXED_FORWARD_ENABLE=0,UART_SPEED_ENABLE=0,
 parameter signed [31:0] MODEL_WHEEL_GAIN_Q12=4096,
 parameter signed [31:0] LEFT_TARGET_MMPS=0,RIGHT_TARGET_MMPS=0
)(input wire signed [31:0] model_left_mmps,model_right_mmps,
 input wire command_emergency_stop,uart_speed_seen,ir_key_active,
 input wire [7:0] ir_key_code,
 output wire signed [31:0] out_left,out_right);
'''

TB='''`timescale 1ns/1ps
// Direct action publication is the explicit component boundary. There is no
// inference substitute presented as a4500 run, and no internal force/release.
module tb_PID_unity_equivalence;
 reg clk=0;always #10 clk=~clk; // production50MHz
 reg reset_n=0,command_enable=0,sensor_permit=1,ir_stop=0,ir_active=0;
 reg [7:0] ir_key=0;reg uart_seen=0;
 reg action_valid=0;reg [143:0] actions=0;
 reg auto_samples=1;reg signed [15:0] fb_left=0,fb_right=0;
 reg [18:0] sample_counter=0;reg sample_valid=0;
 reg [18:0] frame_counter=0;
 wire map_run=command_enable&&sensor_permit&&!ir_stop;
 wire signed [31:0] model_left,model_right;
 wire have_action;
 wire [15:0] lh,lk,rh,rk;wire [143:0] last_actions;
 policy_action_map map(.clk(clk),.rst_n(reset_n),.run(map_run),.action_valid(action_valid),.actions(actions),
  .ff_lh(12'sd0),.ff_lk(12'sd0),.ff_rh(12'sd0),.ff_rk(12'sd0),
  .left_mmps(model_left),.right_mmps(model_right),.lh_us(lh),.lk_us(lk),.rh_us(rh),.rk_us(rk),
  .last_actions(last_actions),.have_action(have_action));
 wire signed [31:0] target_old_l,target_old_r,target_new_l,target_new_r;
 pnr53_target_old old_select(.model_left_mmps(model_left),.model_right_mmps(model_right),
  .command_emergency_stop(ir_stop),.uart_speed_seen(uart_seen),.ir_key_active(ir_active),.ir_key_code(ir_key),
  .out_left(target_old_l),.out_right(target_old_r));
 pnr53_target_new new_select(.model_left_mmps(model_left),.model_right_mmps(model_right),
  .command_emergency_stop(ir_stop),.uart_speed_seen(uart_seen),.ir_key_active(ir_active),.ir_key_code(ir_key),
  .out_left(target_new_l),.out_right(target_new_r));
 // Original control_run and wheel_control_enable boolean boundary; no pipeline.
 wire wheel_enable=command_enable&&!ir_stop&&(map_run&&have_action);
 wire [5:0] pins_old,pins_new;wire signed [15:0] dl_old,dr_old,dl_new,dr_new;
 wire [2:0] fl_old,fr_old,fl_new,fr_new;
 wire signed [31:0] rl_old,rr_old,el_old,er_old,rl_new,rr_new,el_new,er_new;
 wire vl_old,vr_old,vl_new,vr_new;
 motor520_closed_loop #(.LEFT_KP_Q12(12288),.RIGHT_KP_Q12(12288),
  .LEFT_KI_Q12(1434),.RIGHT_KI_Q12(1434),.LEFT_KD_Q12(8192),.RIGHT_KD_Q12(8192),
  .LEFT_KFF_Q12(3277),.RIGHT_KFF_Q12(3277),.USE_MMPS_FEEDBACK(0),
  .DERIVATIVE_ALPHA_Q16(0),.DERIVATIVE_FILTER_SHIFT(2),.CLOSED_LOOP_AT_ZERO(0)) old_pid(
  .clk(clk),.reset_n(reset_n),.enable(wheel_enable),.target_left_mmps(target_old_l),.target_right_mmps(target_old_r),
  .speed_left_cms(fb_left),.speed_right_cms(fb_right),.valid_left(sample_valid),.valid_right(sample_valid),
  .speed_left_mmps(32'sd0),.speed_right_mmps(32'sd0),
  .pwm_left(pins_old[0]),.dir_left1(pins_old[1]),.dir_left2(pins_old[2]),
  .pwm_right(pins_old[3]),.dir_right1(pins_old[4]),.dir_right2(pins_old[5]),
  .duty_left(dl_old),.duty_right(dr_old),.fault_left(fl_old),.fault_right(fr_old),
  .reference_left_mmps(rl_old),.reference_right_mmps(rr_old),.error_left_mmps(el_old),.error_right_mmps(er_old),
  .control_valid_left(vl_old),.control_valid_right(vr_old));
 motor520_closed_loop #(.LEFT_KP_Q12(12288),.RIGHT_KP_Q12(12288),
  .LEFT_KI_Q12(1434),.RIGHT_KI_Q12(1434),.LEFT_KD_Q12(8192),.RIGHT_KD_Q12(8192),
  .LEFT_KFF_Q12(3277),.RIGHT_KFF_Q12(3277),.USE_MMPS_FEEDBACK(0),
  .DERIVATIVE_ALPHA_Q16(0),.DERIVATIVE_FILTER_SHIFT(2),.CLOSED_LOOP_AT_ZERO(0)) new_pid(
  .clk(clk),.reset_n(reset_n),.enable(wheel_enable),.target_left_mmps(target_new_l),.target_right_mmps(target_new_r),
  .speed_left_cms(fb_left),.speed_right_cms(fb_right),.valid_left(sample_valid),.valid_right(sample_valid),
  .speed_left_mmps(32'sd0),.speed_right_mmps(32'sd0),
  .pwm_left(pins_new[0]),.dir_left1(pins_new[1]),.dir_left2(pins_new[2]),
  .pwm_right(pins_new[3]),.dir_right1(pins_new[4]),.dir_right2(pins_new[5]),
  .duty_left(dl_new),.duty_right(dr_new),.fault_left(fl_new),.fault_right(fr_new),
  .reference_left_mmps(rl_new),.reference_right_mmps(rr_new),.error_left_mmps(el_new),.error_right_mmps(er_new),
  .control_valid_left(vl_new),.control_valid_right(vr_new));
 integer checked_cycles=0,reset_edges=0,stop_phases=0,disable_phases=0,reset_phases=0;
 integer action_cases=0,forward_seen=0,reverse_seen=0,pwm_high_seen=0;
 integer fault1_seen=0,fault2_seen=0,fault3_seen=0;
 reg monitor=0;
 always @(negedge clk) begin
  if(sample_counter==499999)begin sample_counter<=0;sample_valid<=auto_samples;end
  else begin sample_counter<=sample_counter+1;sample_valid<=0;end
  if(frame_counter==399999)frame_counter<=0;else frame_counter<=frame_counter+1;
 end
 always @(posedge clk) begin
  #1;
  if(monitor)begin
   if($isunknown({model_left,model_right,target_old_l,target_old_r,target_new_l,target_new_r,
      pins_old,pins_new,dl_old,dr_old,dl_new,dr_new,fl_old,fr_old,fl_new,fr_new,
      rl_old,rr_old,el_old,er_old,rl_new,rr_new,el_new,er_new,vl_old,vr_old,vl_new,vr_new}))
      $fatal(1,"unknown is not an equivalence pass");
   if(model_left< -600||model_left>600||model_right< -600||model_right>600)$fatal(1,"reachable range");
   if({target_old_l,target_old_r}!={target_new_l,target_new_r})$fatal(1,"target mismatch");
   if({pins_old,dl_old,dr_old,fl_old,fr_old,rl_old,rr_old,el_old,er_old,vl_old,vr_old}!==
      {pins_new,dl_new,dr_new,fl_new,fr_new,rl_new,rr_new,el_new,er_new,vl_new,vr_new})$fatal(1,"percycle PID/PWM mismatch");
   if((!reset_n||!wheel_enable)&&pins_new!=0)$fatal(1,"hard disable delayed");
   checked_cycles++;
   if(pins_new[0]||pins_new[3])pwm_high_seen++;
   if(pins_new[1]||pins_new[4])forward_seen++;
   if(pins_new[2]||pins_new[5])reverse_seen++;
   if(fl_new==1||fr_new==1)fault1_seen++;
   if(fl_new==2||fr_new==2)fault2_seen++;
   if(fl_new==3||fr_new==3)fault3_seen++;
  end
 end
 task clocks(input integer n);repeat(n)@(negedge clk);endtask
 task publish(input signed [23:0] raw_l,input signed [23:0] raw_r);
  begin
   wait(frame_counter==399999);@(negedge clk);
   actions=0;actions[48+:24]=raw_l;actions[120+:24]=raw_r;
   action_valid=1;@(negedge clk);action_valid=0;action_cases++;
  end
 endtask
 task rearm;
  begin
   @(negedge clk);command_enable=0;ir_stop=0;sensor_permit=1;auto_samples=1;fb_left=0;fb_right=0;
   clocks(4);command_enable=1;publish(-24'sd4096,24'sd4096);clocks(4);
  end
 endtask
 task samples(input integer n);repeat(n)begin @(posedge sample_valid);@(negedge clk);end endtask
 task require_zero;
  begin
   #1;if(pins_new!=0||pins_old!=0)$fatal(1,"immediate PWM/dir disable");
   @(posedge clk);#1;
   if(dl_new!=0||dr_new!=0||rl_new!=0||rr_new!=0||vl_new||vr_new)$fatal(1,"nextedge PID clear");
  end
 endtask
 integer phase,i;
 reg signed [23:0] cases[0:12];
 initial begin
  cases[0]=-24'sd8388608;cases[1]=-4097;cases[2]=-4096;cases[3]=-4095;
  cases[4]=-2048;cases[5]=-1;cases[6]=0;cases[7]=1;cases[8]=2048;
  cases[9]=4095;cases[10]=4096;cases[11]=4097;cases[12]=8388607;
  clocks(4);reset_n=1;clocks(4);monitor=1;command_enable=1;
  for(i=0;i<13;i=i+1)begin
   ir_active=i[0];ir_key=(i%3==0)?8'h18:((i%3==1)?8'h52:8'h5a);
   uart_seen=i[1];publish(cases[i],cases[12-i]);clocks(8);
  end
  // Assert each stop/disable/reset in every actual observed PID state.
  for(phase=0;phase<8;phase=phase+1)begin
   rearm();wait(old_pid.left_control.state==phase);@(negedge clk);ir_stop=1;
   require_zero();stop_phases++;clocks(3);
   rearm();wait(old_pid.left_control.state==phase);@(negedge clk);command_enable=0;
   require_zero();disable_phases++;clocks(3);
   rearm();wait(old_pid.left_control.state==phase);@(negedge clk);reset_n=0;
   require_zero();reset_phases++;clocks(3);reset_n=1;
  end
  // Production100Hz feedback and original full thresholds, no faster timers.
  rearm();fb_left=30;fb_right=30;samples(10);
  publish(24'sd4096,-24'sd4096);fb_left=-30;fb_right=-30;samples(16);
  publish(24'sd0,24'sd0);samples(3);clocks(3000);
  rearm();fb_left=-30;fb_right=-30;
  wait(fl_new==3&&fr_new==3);clocks(4);if(!fault3_seen)$fatal(1,"wrongdir uncovered");
  rearm();fb_left=0;fb_right=0;
  wait(fl_new==2&&fr_new==2);clocks(4);if(!fault2_seen)$fatal(1,"stall uncovered");
  rearm();auto_samples=0;
  wait(fl_new==1&&fr_new==1);clocks(4);if(!fault1_seen)$fatal(1,"timeout uncovered");
  rearm();samples(3);
  if(stop_phases!=8||disable_phases!=8||reset_phases!=8||!forward_seen||!reverse_seen||!pwm_high_seen)
   $fatal(1,"coverage incomplete");
  $display("TEST PASSED PID_UNITY checked_cycles=%0d stop_phases=%0d disable_phases=%0d reset_phases=%0d actions=%0d fault1=%0d fault2=%0d fault3=%0d forward=%0d reverse=%0d",
    checked_cycles,stop_phases,disable_phases,reset_phases,action_cases,fault1_seen,fault2_seen,fault3_seen,forward_seen,reverse_seen);
  $finish;
 end
 initial begin #3s;$fatal(1,"finite3s stimulus watchdog, not a PNR deadline");end
endmodule
'''

def source_texts():
    core=HW/'control4500_core.v';c=core.read_text();a=HW/'control/policy_action_map.v';p=HW/'control/motor520_closed_loop.v'
    assert ORIGINAL_TARGETS in c and 'MODEL_WHEEL_GAIN_Q12=$rtoi(MODEL_WHEEL_SPEED_GAIN*4096.0+0.5)'in c
    at=a.read_text();pid=p.read_text()
    for anchor in(
        "begin clip=a>4096 ? 24'sd4096 : a < -4096 ? -24'sd4096 : a; end",
        'product=$signed(clip(a))*gain;',
        'scale=product<0 ? -((-product+2048)>>>12) : ((product+2048)>>>12);',
        'if(!rst_n) begin', 'end else if(!run) begin','end else if(action_valid) begin',
        'left_mmps<=scale(actions[48+:24],-600);','right_mmps<=scale(actions[120+:24],600);'):
        assert anchor in at,('actual mapped action shape changed',anchor)
    assert at.count('left_mmps<=0;right_mmps<=0;')==2
    for anchor in('LEFT_KP=3.00,LEFT_KI=0.35,LEFT_KD=2.00,LEFT_KFF=0.80',
        'RIGHT_KP=3.00,RIGHT_KI=0.35,RIGHT_KD=2.00,RIGHT_KFF=0.80',
        "parameter MOTOR_CLOSED_LOOP_AT_ZERO=1'b0"):
        assert anchor in c,('production parameter changed',anchor)
    for anchor in('FEEDBACK_TIMEOUT_CYCLES=10_000_000','STALL_SAMPLES=100','WRONG_DIRECTION_SAMPLES=10',
        'PWM_PERIOD_CYCLES=2500','RAMP_STEP_MMPS=100','DUTY_MAX=2400'):
        assert anchor in pid,('production PID default changed',anchor)
    start=c.index('    wire amplify_model_wheels=');end=c.index('    localparam signed [31:0] MODEL_WHEEL_GAIN_Q12=',start)
    amplify=c[start:end]
    start=c.index('    function signed [31:0] amplified_model_mmps;');end=c.index('    endfunction',start)+len('    endfunction')
    fn=c[start:end]+'\n'
    def module(name,targets):
        return TARGET_HEADER.replace('TARGET_NAME',name)+amplify+fn+targets+'    assign out_left=wheel_left_target_mmps;\n    assign out_right=wheel_right_target_mmps;\nendmodule\n'
    return [core,a,p],module('pnr53_target_old',ORIGINAL_TARGETS)+module('pnr53_target_new',UNITY_TARGETS)

def prove_math(out):
    from prove_soc_console_remap import z3
    action=z3.BitVec('action24',24);old=z3.BitVec('old_model32',32)
    clip=z3.If(action>4096,z3.BitVecVal(4096,24),z3.If(action< -4096,z3.BitVecVal(-4096,24),action))
    def rounded(p):return z3.If(p<0,-((-p+2048)>>12),(p+2048)>>12)
    tests=[]
    def check(name,bad,assumptions=()):
        s=z3.Solver();s.add(*assumptions,bad);r=s.check();tests.append(dict(name=name,result=str(r)))
        assert r==z3.unsat,(name,str(s.model())if r==z3.sat else str(r))
    check('signed24_clip_range',z3.Or(clip< -4096,clip>4096))
    for gain in(-600,600):
        product=z3.SignExt(40,clip)*z3.BitVecVal(gain,64);scale=z3.Extract(31,0,rounded(product))
        check('all24_action_scale_range_'+str(gain),z3.Or(scale< -600,scale>600))
        reset,run,valid=z3.Bools('reset run valid')
        nxt=z3.If(z3.Or(reset,z3.Not(run)),z3.BitVecVal(0,32),z3.If(valid,scale,old))
        check('reset_valid_hold_range_induction_'+str(gain),z3.Or(nxt< -600,nxt>600),(old>=-600,old<=600))
    v=z3.BitVec('known_model_speed32',32);q=rounded(z3.SignExt(32,v)*4096)
    amp=z3.Extract(31,0,z3.If(q>600,z3.BitVecVal(600,64),z3.If(q< -600,z3.BitVecVal(-600,64),q)))
    check('unity_amplifier_identity',amp!=v,(v>=-600,v<=600))
    select=z3.Bool('arbitrary_IR_selection');check('all_IR_select_unity_targets',z3.If(select,amp,v)!=v,(v>=-600,v<=600))
    save(out/'actual_math_checks.json',dict(passed=True,checks=tests,full_known24_action_domain=True,unknown_initial_state_proven=False))

def prepare(out,materialize):
    assert not out.exists();g=gate54(materialize);sources,target=source_texts();out.mkdir()
    plan=dict(at=datetime.now().astimezone().isoformat(),phase='prepared not executed',gate54=g,
        bindings=[metadata(x)for x in sources]+[metadata(Path(__file__)),metadata(REPO/'integration/g2/plan_PNR53_PID_unity_bypass.py')],
        materialized=materialize,EDA_started=False,
        files_if_materialized=['policy_action_map.v (exact copy)','motor520_closed_loop.v (exact copy; includes real left/right PID and PWM)',
             'target_select_pair.sv (literal old/new selected expressions)','tb_PID_unity_equivalence.sv'],
        mathematical_coverage=['Z3 all2^24 known signed actions through source-bound clip/scale for+-600',
             'reset0/run0/valid/hold inductive range[-600,600]','all1201 known model speeds through gain4096 round/limit',
             'arbitrary IR selection over the proven model domain'],
        RTL_coverage=['production50MHz clock; original100Hz feedback counters and original timeout10M/stall100/wrong10 thresholds',
             'actual policy_action_map + actual two left/right closed loops and PWM; no changed PID function or inference substitute',
             '13 min/max/clip-border/half-scale/zero signed24 actions, held/invalid edges',
             'IR stop,enable loss,reset at each observed actual eight PID states; immediate PWM/dir disable and original next-edge clears',
             'positive/negative reverse/coast,zero targets,missing feedback,wrong direction,stall and explicit re-enable',
             'known after real reset, then explicit X rejection and every-cycle comparison of public targets,duty/reference/error/fault/control_valid/all6PWM-dir outputs'],
        explicit_component_boundary='arbitrary action_valid/action words feed the actual mapper; map_run/model_ready condition is an explicit external boundary, not a claim that full fusion/policy/stop_latched integration was simulated',
        unchanged_nonunity='generator retains literal original nonunity target expressions; this finite sequential DUT selects actual production unity configuration',
        not_yet_checked=['SV compile/elaboration','actual Z3/RTL results','unknown arbitrary initial-state four-state equivalence',
             'full4500 timing/numerics and live IR decoder','actual mapping/physical path/complete743PID negative endpoint coverage'],
        finite_watchdog='3s simulated stimulus bound only; no PNR deadline',
        run_admission='2GiB available physical/4GiB available commit before eachxsim stage; only one sequential xvlog/xelab/xsim chain',
        command='D:/Anaconda/python.exe -B -X utf8 integration/g2/prepare_PID_unity_equivalence_probe.py --run --out <new-output>',
        selected_RTL_or_PNR_inputs_modified=False,deployment_ready=False)
    if materialize:
        for p in sources[1:]:
            (out/p.name).write_bytes(p.read_bytes())
        (out/'target_select_pair.sv').write_text(target,encoding='utf8')
        (out/'tb_PID_unity_equivalence.sv').write_text(TB,encoding='utf8')
        plan['fixture_bindings']=[metadata(p)for p in sorted(out.glob('*'))if p.is_file()]
    else:assert not list(out.glob('*.v')) and not list(out.glob('*.sv'))
    save(out/'plan.json',plan)
    if not materialize:save(REPO/'docs/4_design/evidence/g2_20261023/PNR53_PID_unity_equivalence_prepared_20261026.json',plan)
    return plan

def run(out):
    plan=prepare(out,True);state=dict(plan,passed=False,controller_pid=os.getpid(),controller_created_epoch=psutil.Process().create_time())
    save(out/'job.json',dict(state,phase='starting_gated_component_probe'));runtimes=[]
    try:
        prove_math(out)
        for tool,args in [('xvlog',['-sv','policy_action_map.v','motor520_closed_loop.v','target_select_pair.sv','tb_PID_unity_equivalence.sv']),
             ('xelab',['-debug','off','-timescale','1ns/1ps','tb_PID_unity_equivalence','-s','PID_unity']),('xsim',['PID_unity','-runall'])]:
            m=memory_status();assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3,('memory admission',tool,m)
            cmd=['cmd','/c',str(VIV/(tool+'.bat')),*args]
            with(out/(tool+'_stdout.log')).open('w',encoding='utf8')as log:
                p=subprocess.Popen(cmd,cwd=out,stdout=log,stderr=subprocess.STDOUT,
                    creationflags=subprocess.CREATE_NO_WINDOW|subprocess.BELOW_NORMAL_PRIORITY_CLASS)
                r=dict(tool=tool,pid=p.pid,created_epoch=psutil.Process(p.pid).create_time(),command=cmd,at=datetime.now().astimezone().isoformat(),
                    available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
                state['EDA_started']=True
                save(out/(tool+'_runtime.json'),r);save(out/'job.json',dict(state,phase='running_'+tool,tool_runtime=r))
                while p.poll()is None:
                    try:r['children']=[dict(pid=q.pid,created_epoch=q.create_time(),name=q.name(),command=q.cmdline())for q in psutil.Process(p.pid).children(recursive=True)]
                    except(psutil.NoSuchProcess,psutil.AccessDenied):pass
                    save(out/'tool_runtime.json',r);time.sleep(2)
                r.update(exit_code=p.returncode,ended=datetime.now().astimezone().isoformat());save(out/(tool+'_exit.json'),r);runtimes.append(r)
            assert p.returncode==0,(tool,p.returncode)
        log=(out/'xsim.log').read_text(errors='replace');assert 'TEST PASSED PID_UNITY'in log and not re.search(r'Fatal:|FATAL_ERROR|^FAIL',log,re.M)
        state.update(passed=True,phase='finite_PID_unity_component_passed',output=re.search(r'TEST PASSED PID_UNITY[^\n]*',log)[0],
            EDA_started=True,only_component_RTL_simulation=True,full_G2_or_physical_acceptance=False)
    except Exception as e:state.update(phase='failed_preserved_no_retry',error=str(e),traceback=traceback.format_exc())
    for binding in plan['bindings']+plan.get('fixture_bindings',[]):
        assert metadata(Path(binding['path']))==binding,('frozen input changed',binding['path'])
    state['runtimes']=runtimes;save(out/'result.json',state);save(out/'job.json',state);return state

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path);g=p.add_mutually_exclusive_group();g.add_argument('--materialize',action='store_true');g.add_argument('--run',action='store_true');a=p.parse_args()
    if a.materialize or a.run:assert a.out is not None,'explicit new output directory required'
    out=a.out or BASE/'PNR53_PID_unity_equivalence_plan2'
    r=run(out)if a.run else prepare(out,a.materialize)
    print(json.dumps(dict(output=str(out),materialized=r['materialized'],passed=r.get('passed'),EDA_started=r['EDA_started'])))
