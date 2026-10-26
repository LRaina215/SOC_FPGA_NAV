"""Prepare model/fusion25, peripherals/NAV50; never starts SYN/PnR.

No original frozen build is modified. This is deliberately not wholeu_control25:
the original PWM/PID/encoder/BMI/IR/UART/servo/OLED physical timing stays50MHz.
"""
from pathlib import Path
from datetime import datetime
import argparse,copy,difflib,json,re,shutil,subprocess
from prepare_baseline_fusion_retime import hardware_inputs,metadata,relocate,same_bytes,save
from ddr3.generate_pll import MOD

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_only_syn23'
ROOT=Path(__file__).resolve().parents[2]
MODELS=[('target_left_mmps',32),('target_right_mmps',32),('servo_lh_us',16),('servo_lk_us',16),('servo_rh_us',16),('servo_rk_us',16),('executed_actions',144),('completed_frames',32),('inference_cycles',32)]
FUSION=[('debug_wx',24),('debug_wy',24),('debug_wz',24),('debug_gx',24),('debug_gy',24),('debug_gz',24),('accel_rejected',1)]
STATUS=[('ready',1),('active',1),('calibrating',1),('bias_suspect',1),('status_flags',8),('policy_fault',3)]

def replace_once(text,old,new):
    assert text.count(old)==1,old
    return text.replace(old,new,1)

def pll(out):
    out.mkdir()
    mod=MOD.replace('Nav_DDR3_PLL','Gowin_Model25_PLL').replace('nav_ddr3_pll','model25_pll')
    mod=mod.replace('-odiv0_sel 2\n','-odiv0_sel 32\n').replace('-en_clkout1 true','-en_clkout1 false').replace('-en_clkout2 true','-en_clkout2 false')
    (out/'model25_pll.mod').write_text(mod.replace('OUT_PATH',out.as_posix()),newline='\n')
    with(out/'generate.log').open('w')as log:
        p=subprocess.run(['D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/GowinModGen.exe','-do',str(out/'model25_pll.mod')],cwd=out,stdout=log,stderr=subprocess.STDOUT,timeout=60,creationflags=subprocess.CREATE_NO_WINDOW)
    assert p.returncode==0 and 'Module Generated Successfully'in(out/'generate.log').read_text(errors='replace')
    text=(out/'model25_pll.v').read_text()
    for key,value in [('FCLKIN','"50"'),('IDIV_SEL','1'),('FBDIV_SEL','1'),('MDIV_SEL','16'),('ODIV0_SEL','32'),('CLKOUT0_EN','"TRUE"'),('CLKOUT1_EN','"FALSE"'),('CLKOUT2_EN','"FALSE"')]:
        assert re.search(r'defparam\s+PLL_inst\.'+key+r'\s*=\s*'+re.escape(value)+r'\s*;',text),(key,value)
    assert 'Device Version: C'in text
    return dict(generator='installedGowinModGen',input_mhz=50,VCO_mhz=800,output_mhz=25,device_C=True,actual_physical_global_allocation_verified=False)

def wrapper(original):
    header=original[original.index('module mem_policy #('):original.index(');')+2]
    header=header.replace('module mem_policy #(','module mem_policy_related25 #(',1)
    header=header.replace('PERIOD_CLKS=400000','PERIOD_CLKS=200000').replace('SENSOR_TIMEOUT_CLKS=2500000','SENSOR_TIMEOUT_CLKS=1250000')
    header=header.replace('input wire clk,rst_n,enable','input wire clk,clk_model,rst_n,transport_rst_n,model_rst_n,domain_fault,enable')
    header=header.replace('output wire calibrating,bias_suspect,accel_rejected,','output wire transport_fault,\n    output wire calibrating,bias_suspect,accel_rejected,')
    lines=[header,' wire[31:0]sample_ticks;reg[31:0]source_clock,last_sample_clock;',' always @(posedge clk or negedge rst_n)begin','  if(!rst_n)begin source_clock<=0;last_sample_clock<=0;end','  else begin source_clock<=source_clock+1\'b1;if(sample_valid)last_sample_clock<=source_clock;end',' end',' assign sample_ticks=source_clock-last_sample_clock;']
    # BMI event payload includes the original source-clock interval, preserving
    # integration arithmetic even if a50MHz event shifts one25MHz capture edge.
    lines+=[' wire bmi_commit,bmi_overflow;wire[127:0]bmi_data;',' g2_model25_mailbox #(.WIDTH(128)) bmi_bridge(','  .src_clk(clk),.src_rst_n(transport_rst_n),.src_valid(sample_valid),','  .src_data({sample_ticks,gyro_z,gyro_y,gyro_x,accel_z,accel_y,accel_x}),.src_ready(),.src_overflow(bmi_overflow),','  .dst_clk(clk_model),.dst_rst_n(model_rst_n),.dst_valid(bmi_commit),.dst_data(bmi_data));']
    config=[('enable',1),('sensor_ready',1),('sensor_error',1),('external_fault',1),('left_encoder_delta',32),('right_encoder_delta',32),('command_forward_q12',24),('command_yaw_q12',24),('command_height_q12',24)]
    cw=sum(w for _,w in config)
    lines +=[f' wire[{cw-1}:0]config_data;',f' g2_model25_latest #(.WIDTH({cw})) config_bridge(','.src_clk(clk),.src_rst_n(transport_rst_n),.src_data({'+','.join(n for n,_ in config)+'}),','.dst_clk(clk_model),.dst_rst_n(model_rst_n),.dst_valid(),.dst_data(config_data));']
    for name,width in config:lines+=[f' wire[{width-1}:0]{name}_25;']
    lines+=[' assign {'+','.join(n+'_25'for n,_ in config)+'}=config_data;']
    outputs=MODELS+FUSION+STATUS+[('action_valid',1),('debug_fusion_valid',1)]
    for name,width in outputs:lines +=[f' wire[{width-1}:0]{name}_25;']
    for name,width in STATUS:lines +=[f' wire[{width-1}:0]{name}_raw;']
    lines +=[' mem_policy #(.WEIGHT_BITS(WEIGHT_BITS),.BIAS_SAMPLES(BIAS_SAMPLES),.PERIOD_CLKS(PERIOD_CLKS),',
       '.SENSOR_TIMEOUT_CLKS(SENSOR_TIMEOUT_CLKS),.FUSION_TIMEBASE_HZ(50000000),.EXTERNAL_SAMPLE_TICKS(1),',
       '.LH_ZERO_US(LH_ZERO_US),.LK_ZERO_US(LK_ZERO_US),.RH_ZERO_US(RH_ZERO_US),.RK_ZERO_US(RK_ZERO_US)) model(',
       '.clk(clk_model),.rst_n(model_rst_n),.sample_interval_ticks(bmi_data[127:96]),.sample_valid(bmi_commit),',
       '.accel_x(bmi_data[15:0]),.accel_y(bmi_data[31:16]),.accel_z(bmi_data[47:32]),',
       '.gyro_x(bmi_data[63:48]),.gyro_y(bmi_data[79:64]),.gyro_z(bmi_data[95:80]),',
       ','.join('.'+n+'('+n+'_25)'for n,_ in config)+',',
       ','.join('.'+n+'('+n+'_25)'for n,_ in outputs)+');']
    # Actuator map consumes action_valid one edge after policy publication.
    # Capture the resulting complete set on the next edge, then deliver once50.
    lines+=[' reg action_commit25;always @(posedge clk_model or negedge model_rst_n)','  if(!model_rst_n)action_commit25<=0;else action_commit25<=action_valid_25;',
        ' wire action_commit50,action_overflow;wire[335:0]action_data50;reg[335:0]action_hold50;',
        ' g2_model25_mailbox #(.WIDTH(336)) action_bridge(',
        '.src_clk(clk_model),.src_rst_n(model_rst_n),.src_valid(action_commit25),',
        '.src_data({'+','.join(n+'_25'for n,_ in MODELS)+'}),.src_ready(),.src_overflow(action_overflow),',
        '.dst_clk(clk),.dst_rst_n(transport_rst_n),.dst_valid(action_commit50),.dst_data(action_data50));',
        " reg action_seen;reg action_pulse;always @(posedge clk or negedge rst_n)begin",
        "  if(!rst_n)begin action_seen<=0;action_pulse<=0;action_hold50<={32'd0,32'd0,LH_ZERO_US[15:0],LK_ZERO_US[15:0],RH_ZERO_US[15:0],RK_ZERO_US[15:0],144'd0,32'd0,32'd0};end",
        "  else begin action_pulse<=0;",
        "   if(!ready_raw||domain_fault||external_fault||!enable)begin action_hold50[335:272]<=0;action_hold50[207:64]<=0;action_seen<=0;end",
        "   else if(action_commit50)begin action_hold50<=action_data50;action_seen<=1;action_pulse<=1;end",
        "  end",
        ' end',' assign {'+','.join(n for n,_ in MODELS)+'}=action_hold50;',' assign action_valid=action_pulse;']
    fw=sum(w for _,w in FUSION)
    lines +=[f' wire fusion_commit50,fusion_overflow;wire[{fw-1}:0]fusion_data50;reg[{fw-1}:0]fusion_hold50;reg fusion_pulse;',
        f' g2_model25_mailbox #(.WIDTH({fw})) fusion_bridge(',
        '.src_clk(clk_model),.src_rst_n(model_rst_n),.src_valid(debug_fusion_valid_25),',
        '.src_data({'+','.join(n+'_25'for n,_ in FUSION)+'}),.src_ready(),.src_overflow(fusion_overflow),',
        '.dst_clk(clk),.dst_rst_n(transport_rst_n),.dst_valid(fusion_commit50),.dst_data(fusion_data50));',
        ' always @(posedge clk or negedge rst_n)begin',
        "  if(!rst_n)begin fusion_pulse<=0;fusion_hold50<={24'd0,24'd0,24'd0,24'd0,24'd0,-24'sd4096,1'b0};end",
        '  else begin fusion_pulse<=0;if(fusion_commit50&&!domain_fault)begin fusion_hold50<=fusion_data50;fusion_pulse<=1;end end',
        ' end',' wire accel_rejected_snapshot;',' assign {'+','.join(n for n,_ in FUSION[:-1])+',accel_rejected_snapshot}=fusion_hold50;',
        ' assign accel_rejected=accel_rejected_snapshot&&fusion_pulse;',
        ' assign debug_fusion_valid=fusion_pulse;']
    sw=sum(w for _,w in STATUS)
    lines +=[f' wire[{sw-1}:0]status_data50;',f' g2_model25_latest #(.WIDTH({sw})) status_bridge(',
        '.src_clk(clk_model),.src_rst_n(model_rst_n),.src_data({'+','.join(n+'_25'for n,_ in STATUS)+'}),',
        '.dst_clk(clk),.dst_rst_n(transport_rst_n),.dst_valid(),.dst_data(status_data50));']
    lines+=[' assign {'+','.join(n+'_raw'for n,_ in STATUS)+'}=status_data50;']
    lines +=[' assign ready=ready_raw&&!domain_fault&&!external_fault;',' assign active=active_raw&&action_seen&&!domain_fault&&!external_fault;',
        ' assign calibrating=calibrating_raw;assign bias_suspect=bias_suspect_raw;',
        ' assign status_flags={active,(domain_fault|status_flags_raw[6]),(external_fault|status_flags_raw[5]),status_flags_raw[4:0]};',
        " assign policy_fault=domain_fault ? (policy_fault_raw|3'b100) : policy_fault_raw;",
        '// Any event overflow is a candidate-rejecting condition, never silent.',
        ' reg overflow1,overflow2;',
        ' always @(posedge clk or negedge rst_n)begin',
        '  if(!rst_n)begin overflow1<=0;overflow2<=0;end',
        '  else begin overflow1<=action_overflow|fusion_overflow;overflow2<=overflow1;end',
        ' end',
        ' assign transport_fault=bmi_overflow|overflow2;',
        'endmodule']
    return '\n'.join(lines)+'\n'

def prepare(out):
    assert not out.exists();out.mkdir(parents=True)
    m=json.loads((SOURCE/'manifest.json').read_text());hw=hardware_inputs(SOURCE,m)
    allfiles=[p for p in SOURCE.rglob('*')if p.is_file()and'impl'not in p.relative_to(SOURCE).parts]
    assets={p.relative_to(SOURCE).as_posix():p for p in allfiles if p.suffix.lower()in{'.hex','.mem','.bin','.mif'}}
    headers={p.relative_to(SOURCE).as_posix():p for p in allfiles if p.suffix.lower()in{'.h','.vh'}}
    assert len(hw)==63 and len(assets)==69 and len(headers)==31
    for name,p in {**hw,**assets,**headers}.items():
        dst=out/name;dst.parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(p,dst)
    for directory in('fw','firmware'):shutil.copytree(SOURCE/directory,out/directory,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
    originals={n:(out/n).read_text()for n in('g2_top.v','control4500_core.v','control/mem_policy.v','control/imu_fusion.v')}
    t=originals['control/imu_fusion.v']
    t=replace_once(t,'parameter integer CLK_HZ = 50000000','parameter integer EXTERNAL_SAMPLE_TICKS = 0,\n    parameter integer CLK_HZ = 50000000')
    t=replace_once(t,'input wire sample_valid,','input wire sample_valid,\n    input wire [31:0] sample_interval_ticks,')
    t=replace_once(t,'integration_ticks<=sample_clock-last_sample_clock;','integration_ticks<=EXTERNAL_SAMPLE_TICKS ? sample_interval_ticks : sample_clock-last_sample_clock;')
    (out/'control/imu_fusion.v').write_text(t,newline='\n')
    t=originals['control/mem_policy.v']
    t=replace_once(t,'parameter integer WEIGHT_BITS=16,','parameter integer FUSION_TIMEBASE_HZ=50000000,\n    parameter integer EXTERNAL_SAMPLE_TICKS=0,\n    parameter integer WEIGHT_BITS=16,')
    t=replace_once(t,'input wire clk,rst_n,enable,sensor_ready,sensor_error,sample_valid,external_fault,','input wire clk,rst_n,enable,sensor_ready,sensor_error,sample_valid,external_fault,\n    input wire[31:0]sample_interval_ticks,')
    t=replace_once(t,'imu_fusion #(.BIAS_SAMPLES(BIAS_SAMPLES),.GYRO_MUL(32\'sd4466)) fusion(','imu_fusion #(.BIAS_SAMPLES(BIAS_SAMPLES),.GYRO_MUL(32\'sd4466),.CLK_HZ(FUSION_TIMEBASE_HZ),.EXTERNAL_SAMPLE_TICKS(EXTERNAL_SAMPLE_TICKS)) fusion(')
    t=replace_once(t,'.clk(clk),.rst_n(rst_n),.sample_valid(sample_valid),','.clk(clk),.rst_n(rst_n),.sample_valid(sample_valid),.sample_interval_ticks(sample_interval_ticks),')
    (out/'control/mem_policy.v').write_text(t,newline='\n')
    # Wrapper from the unmodified interface; all ordinary control ports remain.
    (out/'control/mem_policy_related25.v').write_text(wrapper(originals['control/mem_policy.v']),newline='\n')
    shutil.copyfile(ROOT/'integration/g2/model25_related_mailbox.v',out/'model25_related_mailbox.v')
    t=originals['control4500_core.v'];t=replace_once(t,'input wire clk,reset_n,','input wire clk,clk_model,pll_model_lock,reset_n,')
    a='    wire device_reset_n=reset_n && (&startup);'
    b=a+'''\n    wire model_transport_rst_n,model_rst_n,model_domain_fault,model_transport_fault;
    g2_model25_guard model_clock_guard(.clk50(clk),.clk25(clk_model),.reset_n(device_reset_n),.pll_lock(pll_model_lock),
      .transport_fault(model_transport_fault),.io_rst_n(model_transport_rst_n),.model_rst_n(model_rst_n),.fault(model_domain_fault));'''
    t=replace_once(t,a,b)
    t=replace_once(t,'mem_policy #(.WEIGHT_BITS(MODEL_WEIGHT_BITS),.BIAS_SAMPLES(IMU_BIAS_SAMPLES),','mem_policy_related25 #(.WEIGHT_BITS(MODEL_WEIGHT_BITS),.BIAS_SAMPLES(IMU_BIAS_SAMPLES),')
    t=replace_once(t,'.clk(clk),.rst_n(device_reset_n),.enable(MODEL_ENABLE && command_run),','.clk(clk),.clk_model(clk_model),.rst_n(device_reset_n),.transport_rst_n(model_transport_rst_n),\n        .model_rst_n(model_rst_n),.domain_fault(model_domain_fault),.transport_fault(model_transport_fault),.enable(MODEL_ENABLE && command_run),')
    (out/'control4500_core.v').write_text(t,newline='\n')
    t=originals['g2_top.v'];a='    control4500_core u_control('
    b='''    wire clk_model25,model25_pll_lock;
    Gowin_Model25_PLL u_model_pll(.clkin(clk50),.reset(rst),.lock(model25_pll_lock),
        .enclk0(1'b1),.clkout0(clk_model25));
'''+a
    t=replace_once(t,a,b)
    t=replace_once(t,'.clk(clk),.reset_n(~rst),.nav_forward_q12(nav_forward_q12),','.clk(clk),.clk_model(clk_model25),.pll_model_lock(model25_pll_lock),.reset_n(~rst),.nav_forward_q12(nav_forward_q12),')
    (out/'g2_top.v').write_text(t,newline='\n')
    pll_proof=pll(out/'model25_pll')
    changed=sorted(n for n in hw if not same_bytes(SOURCE/n,out/n));assert changed==sorted(originals)
    added=['control/mem_policy_related25.v','model25_related_mailbox.v','model25_pll/model25_pll.v']
    for name in('g2.cst','storage_layout.json'):shutil.copyfile(SOURCE/name,out/name)
    sdc=(BASE/'baseline_fusion32_CDC52_pnr35/g2.sdc').read_text()
    sdc+='\n# RealC-versionPLL,800MHzVCO/32=25MHz; no asynchronous clock exception.\ncreate_generated_clock -name model25 -source [get_ports {clk50}] -divide_by 2 [get_pins {u_model_pll/PLL_inst/CLKOUT0}]\n'
    (out/'g2.sdc').write_text(sdc,newline='\n')
    run=relocate((SOURCE/'run.tcl').read_text(),SOURCE,out)
    marker='run syn';assert run.count(marker)==1
    run=run.replace(marker,'\n'.join('add_file -type verilog {'+(out/n).as_posix()+'}'for n in added)+'\n'+marker)
    (out/'run.tcl').write_text(run,newline='\n')
    manifest=relocate(copy.deepcopy(m),SOURCE,out)
    manifest.update(source_files=[str(out/n)for n in list(hw)+added],frozen_source_build=str(SOURCE),
       candidate='model_fusion25_peripherals_NAV50',source_preparation_only=True,synthesis_complete=False,automatic_pnr=False,
       programmed=False,deployment_accepted=False,modified_rtl_inputs=changed,new_rtl_inputs=added,
       actual_clock_plan_mhz=dict(NAV_CPU_peripherals=50,model_fusion_RL_action=25,radar_constraint=100))
    manifest.pop('effective_options',None);save(out/'manifest.json',manifest)
    patch=[]
    for n,old in originals.items():patch+=list(difflib.unified_diff(old.splitlines(True),(out/n).read_text().splitlines(True),fromfile='a/'+n,tofile='b/'+n))
    (out/'model25_delta.patch').write_text(''.join(patch),newline='\n')
    allout={p.relative_to(out).as_posix():p for p in out.rglob('*')if p.is_file()}
    assert all(same_bytes(p,out/n)for n,p in assets.items())
    assert all(same_bytes(p,out/n)for n,p in headers.items())
    report=dict(prepared=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),out=str(out),
       description='ActualPLL25MHz model/fusion/observation/4500/action domain; original50MHz NAV and every peripheral/PID/PWM/encoder/feedback',
       changed_original_RTL=changed,added_RTL=added,unchanged_original_RTL=59,identical_assets=69,identical_original_headers=31,
       weights_algorithm_all_original_capacities_unchanged=True,FW_not_recompiled=True,
       clocks=dict(original50_every_peripheral_and_NAV=True,new25_model_domain=True,PLL=pll_proof,new_physical_global_route_not_verified=True),
       related_cross_domain_channels=[dict(name='BMI',width=128,direction='50to25',type='event incl original50MHz sample interval'),dict(name='configuration',width=140,direction='50to25',type='latest atomic command72+encoder64+four status'),dict(name='action',width=336,direction='25to50',type='one destination-cycle event'),dict(name='fusion',width=145,direction='25to50',type='one destination-cycle event'),dict(name='status',width=15,direction='25to50',type='latest atomic')],
       feedback_valid_still_original50MHz_single_cycle=True,model_PERIOD_CLKS=200000,model_period_ms=8,model_sensor_timeout_clks=1250000,model_sensor_timeout_ms=50,
       unchanged_physical_timing=['BMI100kHzIIC/10ms poll/startup and retry','encoder10ms/1320CPR conversion','PID100Hz/PWM20kHz/2500tick gain units','servo20ms/50MHz pulse resolution','UART217cycles/230400nominal/100HzV2','IR/OLED original timing','NAV CPU50MHz and lease/watchdogs'],
       reset='shared asynchronous assertion and each-clock two-edge release for transport; sticky50MHz PLL/heartbeat fault; model failure gates motor enable while servo50MHz holds last committed pose',
       new_SYN_PNR_started=False,full_model_numeric_physical_time_equivalence_passed=False,full125Hz_end_to_end_verified=False,full_stop_or_deployment_accepted=False,
       required_next=['finite channel/reset/stop/clock-ratio tests','full original4500 sensor-to-actuator numeric comparison at physical125Hz','domain clock stall and reset sequencing including pending handoffs','actual3clock netlist/global allocation/5bundleddata payload timing and reset audit','all target clock and related-boundary setup/hold/no new false-path exceptions','RK UARTHZ interpretation and real hardware IO/stop signoff'],
       bindings=[metadata(out/n)for n in changed+added+['manifest.json','g2.sdc','g2.cst']])
    save(out/'prepared_source_review.json',report);return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    print(json.dumps(prepare(a.out),ensure_ascii=False,indent=2))
