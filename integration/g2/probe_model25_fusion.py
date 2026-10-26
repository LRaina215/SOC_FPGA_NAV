"""Compare real frozen fusion50 and fusion25 with source50 sample intervals.

Uses shorter synthetic sample intervals and BIAS_SAMPLES=4 in both fixtures;
production defaults stay200 and100Hz. This is not whole4500 or closed-loop proof.
"""
import argparse,json,re
from pathlib import Path
from datetime import datetime
from probe_model25_transport import command
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023')
TB=r'''`timescale 1ns/1ps
module tb_model25_fusion;
 reg c50=0,c25=0,rn=0,sv=0;always #10 c50=~c50;always #20 c25=~c25;
 reg signed[15:0]ax=0,ay=0,az=10920,wx=0,wy=0,wz=0;
 reg[31:0]clock50,last_sample;wire[31:0]interval50=clock50-last_sample;
 always @(posedge c50 or negedge rn)begin
  if(!rn)begin clock50<=0;last_sample<=0;end
  else begin clock50<=clock50+1'b1;if(sv)last_sample<=clock50;end
 end
 wire ready,overflow,dv;wire[127:0]data;
 g2_model25_mailbox #(.WIDTH(128)) bmi(.src_clk(c50),.src_rst_n(rn),.src_valid(sv),
 .src_data({interval50,wz,wy,wx,az,ay,ax}),.src_ready(ready),.src_overflow(overflow),
 .dst_clk(c25),.dst_rst_n(rn),.dst_valid(dv),.dst_data(data));
 wire[23:0]rw0,rw1,rw2,rg0,rg1,rg2,cw0,cw1,cw2,cg0,cg1,cg2;
 wire rv,rc,rb,rr,cv,cc,cb,cr;
 imu_fusion_original #(.BIAS_SAMPLES(4),.GYRO_MUL(32'sd4466),.CLK_HZ(50000000)) original(
 .clk(c50),.rst_n(rn),.sample_valid(sv),.accel_x(ax),.accel_y(ay),.accel_z(az),.gyro_x(wx),.gyro_y(wy),.gyro_z(wz),
 .ang_vel_x(rw0),.ang_vel_y(rw1),.ang_vel_z(rw2),.gravity_x(rg0),.gravity_y(rg1),.gravity_z(rg2),
 .valid(rv),.calibrating(rc),.bias_suspect(rb),.accel_rejected(rr));
 imu_fusion #(.BIAS_SAMPLES(4),.GYRO_MUL(32'sd4466),.CLK_HZ(50000000),.EXTERNAL_SAMPLE_TICKS(1)) candidate(
 .clk(c25),.rst_n(rn),.sample_valid(dv),.sample_interval_ticks(data[127:96]),
 .accel_x(data[15:0]),.accel_y(data[31:16]),.accel_z(data[47:32]),.gyro_x(data[63:48]),.gyro_y(data[79:64]),.gyro_z(data[95:80]),
 .ang_vel_x(cw0),.ang_vel_y(cw1),.ang_vel_z(cw2),.gravity_x(cg0),.gravity_y(cg1),.gravity_z(cg2),
 .valid(cv),.calibrating(cc),.bias_suspect(cb),.accel_rejected(cr));
 reg[146:0]expected[0:127];integer write_index=0,read_index=0,total=0,samples=0;
 always @(negedge c50)if(rn&&rv)begin
  expected[write_index]={rw0,rw1,rw2,rg0,rg1,rg2,rc,rb,rr};write_index=write_index+1;
 end
 always @(negedge c25)if(rn&&cv)begin
  if(read_index>=write_index)$fatal(1,"candidate published without reference");
  if({cw0,cw1,cw2,cg0,cg1,cg2,cc,cb,cr}!==expected[read_index])
   $fatal(1,"fusion mismatch at%0d expected%h actual%h",read_index,expected[read_index],{cw0,cw1,cw2,cg0,cg1,cg2,cc,cb,cr});
  read_index=read_index+1;total=total+1;
 end
 initial begin
  #100;rn=1;
  for(integer block=0;block<2;block=block+1)begin
   for(integer k=0;k<32;k=k+1)begin
    repeat(5001+k*17)@(negedge c50);
    if(!ready||overflow)$fatal(1,"BMI channel not ready");
    // First four bias samples are still; later vectors exercise rejection,
    // negative components and nonconstant odd source-clock intervals.
    ax=k<4?0:(k*193)%5000-2500;ay=k<4?0:(k*127)%4000-2000;
    az=(k>4&&k%11==0)?0:10920;
    wx=k<4?0:(k*29)%1300-650;wy=k<4?0:(k*37)%1100-550;wz=k<4?0:(k*43)%900-450;
    sv=1;samples=samples+1;@(negedge c50);sv=0;
   end
   repeat(5000)@(negedge c50);
   if(write_index!=read_index)$fatal(1,"lost fusion output");
   rn=0;#133;write_index=0;read_index=0;rn=1;
  end
  if(total<50)$fatal(1,"insufficient finite numeric coverage%0d",total);
  $display("MODEL25_FUSION_NUMERIC_PASS samples=%0d exact147bit_outputs=%0d reset_sequences=2 source50_intervals=1",samples,total);$finish;
 end
 initial begin #20000000;$fatal(1,"fusion fixture watchdog");end
endmodule
'''
def run(source,out):
 assert not out.exists();out.mkdir(parents=True)
 old=BASE/'baseline_fusion_only_syn23/control/imu_fusion.v'
 text=old.read_text().replace('IMU_FUSION_V_INCLUDED','IMU_FUSION_REFERENCE_V_INCLUDED').replace('module imu_fusion #(','module imu_fusion_original #(',1).replace('imu_div100_3','imu_div100_3_original')
 (out/'fusion_reference.v').write_text(text)
 (out/'tb_model25_fusion.sv').write_text(TB)
 files=[out/'fusion_reference.v']+[source/n for n in['control/imu_fusion.v','control/imu_isqrt.v','control/imu_recip.v','model25_related_mailbox.v']]
 for i,(tool,args)in enumerate([('xvlog',[str(p)for p in files]),('xvlog',['-sv',str(out/'tb_model25_fusion.sv')]),('xelab',['tb_model25_fusion','-s','fusion_snap']),('xsim',['fusion_snap','-runall'])]):command(out,i,tool,args)
 log=(out/'xsim.log').read_text();assert 'MODEL25_FUSION_NUMERIC_PASS'in log and'Fatal:'not in log
 result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),summary=re.search(r'MODEL25_FUSION_NUMERIC_PASS[^\r\n]+',log)[0],
   scope='Real fusion arithmetic50 vs25 with source50 interval transported atomically; two reduced-bias synthetic sequences, not full4500 or production100Hz timing',
   input_bindings=[metadata(old)]+[metadata(p)for p in files[1:]],TB=metadata(out/'tb_model25_fusion.sv'),full_model_inference_or_physical_setup_hold_passed=False)
 save(out/'result.json',result);return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();print(json.dumps(run(a.source,a.out),indent=2))
