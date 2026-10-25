"""Retain fusion publication cycles using two guaranteed-zero sqrt trial cycles.

Independent RTL candidate only. Never invokes Gowin, implementation or hardware.
"""
import argparse,json,re,subprocess,sys
from pathlib import Path
from datetime import datetime,timezone
import psutil
ROOT=Path(__file__).resolve().parents[2]
BASE=Path('F:/nav_integration_g2_20261023/baseline_fusion_only_syn23/control')
PREPARED=Path('F:/nav_integration_g2_20261023/baseline_fusion_only_prepared1/control')

def generate(source,sqrt_source):
 s=source
 anchor='    reg [63:0] norm_square_x=0,norm_square_y=0,norm_square_z=0;'
 assert s.count(anchor)==1
 s=s.replace(anchor,anchor+'''
    // The original unsigned32/24 absolute operands are retained in32bits;
    // the product and square storage stay64bits, without quantization.
    reg [31:0] norm_abs_x=0,norm_abs_y=0,norm_abs_z=0;
    reg [63:0] norm_product_x=0,norm_product_y=0,norm_product_z=0;
    reg norm_abs_valid=0,norm_product_valid=0;''')
 assert s.count('    imu_isqrt u_sqrt (')==1
 s=s.replace('    imu_isqrt u_sqrt (','    imu_fusion_isqrt_late2 u_sqrt (')
 anchor='            sqrt_start <= 0; recip_start <= 0;div_start<=0;bias_div_start<=0;'
 assert s.count(anchor)==1
 s=s.replace(anchor,anchor+'\n            norm_abs_valid<=0;norm_product_valid<=0;')
 anchor='            case (st)'
 assert s.count(anchor)==1
 s=s.replace(anchor,'''            norm_abs_valid<=0;
            norm_product_valid<=norm_abs_valid;
            if(norm_abs_valid) begin
                norm_product_x<={32'd0,norm_abs_x}*norm_abs_x;
                norm_product_y<={32'd0,norm_abs_y}*norm_abs_y;
                norm_product_z<={32'd0,norm_abs_z}*norm_abs_z;
            end
            if(norm_product_valid) begin
                norm_square_x<=norm_product_x;
                norm_square_y<=norm_product_y;
                norm_square_z<=norm_product_z;
            end
'''+anchor)
 for suffix in 'xyz':
  old=f"norm_square_{suffix}<={{32'd0, abs16(accel_{suffix})}} * abs16(accel_{suffix});"
  assert s.count(old)==1
  s=s.replace(old,f'norm_abs_{suffix}<=abs16(accel_{suffix});')
  old=f'norm_square_{suffix}<=abs24(eg{suffix}) * abs24(eg{suffix});'
  assert s.count(old)==1
  s=s.replace(old,f"norm_abs_{suffix}<={{8'd0,abs24(eg{suffix})}};")
 for anchor in ['                norm_abs_z<=abs16(accel_z);',"                norm_abs_z<={8'd0,abs24(egz)};"]:
  assert s.count(anchor)==1;s=s.replace(anchor,anchor+'\n                norm_abs_valid<=1;')
 helper=sqrt_source[sqrt_source.index('module imu_isqrt'):sqrt_source.index('endmodule')+len('endmodule')]
 helper=helper.replace('module imu_isqrt','module imu_fusion_isqrt_late2')
 assert helper.count('                    rem <= value;')==1
 helper=helper.replace('                    rem <= value;','                    rem <= 0; // valid sum arrives during the two zero trials')
 anchor='''            end else begin
                if (rem >= (acc + bitv)) begin'''
 assert helper.count(anchor)==1
 helper=helper.replace(anchor,'''            end else if(cnt<6'd2) begin
                // Fusion's exact sum is<=3*2^46<2^48. The original trials
                // at2^62 and2^60 therefore both leave acc=0,rem=value.
                // Capture the two-cycle-late sum on the second zero trial;
                // cnt,bitv,busy,done and the final root retain every cycle.
                if(cnt==6'd1) rem<=value;
                acc<=0;bitv<=bitv>>2;cnt<=cnt+1'b1;
            end else begin
                if (rem >= (acc + bitv)) begin''')
 assert s.endswith('`endif\n')
 s=s[:-len('`endif\n')]+'''// Private to this fusion contract: NOT a generic64bit sqrt replacement.
// Input must be ready on the second busy trial and remain below2^60.
'''+helper+'\n\n`endif\n'
 return s

def make_tb():
 s=(ROOT/'integration/g2/tb_fusion_equivalence.sv').read_text()
 old='''  if($time>0 && {original.st,original.sqrt_start,original.sqrt_value}!=={candidate.st,candidate.sqrt_start,candidate.sqrt_value})
   $fatal(1,"fusion state or sqrt input cycle mismatch HZ=%0d",HZ);'''
 assert s.count(old)==1
 s=s.replace(old,'''  if($time>0 && {original.st,original.sqrt_start,original.recip_start,original.recip_d,
        original.sqrt_done,original.sqrt_root,original.u_sqrt.busy,original.u_sqrt.cnt,original.u_sqrt.acc,original.u_sqrt.bitv}
       !=={candidate.st,candidate.sqrt_start,candidate.recip_start,candidate.recip_d,
        candidate.sqrt_done,candidate.sqrt_root,candidate.u_sqrt.busy,candidate.u_sqrt.cnt,candidate.u_sqrt.acc,candidate.u_sqrt.bitv})
   $fatal(1,"fusion state/engine publication cycle mismatch HZ=%0d",HZ);
  if(rst_n && original.u_sqrt.busy && original.u_sqrt.cnt>=2
     && original.u_sqrt.rem!==candidate.u_sqrt.rem)
   $fatal(1,"sqrt residual differs after exact late load HZ=%0d",HZ);''')
 anchor=' reg[31:0] rng=4500,math_rng=32\'h8884500;integer cycle=0,math_cases=0,busy_reset=0;'
 assert s.count(anchor)==1
 s=s.replace(anchor,''' reg reset_r=0;wire vr;reg reset_proof_done=0;integer pipeline_resets=0;
 fusion_pair pr(clk,reset_r,1'b0,16'sd0,16'sd0,16'sd0,16'sd0,16'sd0,16'sd0,vr);
 initial begin
  wait(rst_n);repeat(6)@(negedge clk);reset_r=1;
  for(integer phase=0;phase<4;phase++)begin
   @(negedge clk);#1;
   pr.original.egx=-8388608;pr.original.egy=8388607;pr.original.egz=-1;
   pr.candidate.egx=-8388608;pr.candidate.egy=8388607;pr.candidate.egz=-1;
   pr.original.avx=0;pr.original.avy=0;pr.original.avz=0;
   pr.candidate.avx=0;pr.candidate.avy=0;pr.candidate.avz=0;
   pr.original.st=9;pr.candidate.st=9;
   repeat(phase+1)@(negedge clk);#1;reset_r=0;
   repeat(2)@(negedge clk);#1;reset_r=1;repeat(3)@(negedge clk);#1;
   pr.original.egx=4096;pr.original.egy=-4096;pr.original.egz=1;
   pr.candidate.egx=4096;pr.candidate.egy=-4096;pr.candidate.egz=1;
   pr.original.avx=0;pr.original.avy=0;pr.original.avz=0;
   pr.candidate.avx=0;pr.candidate.avy=0;pr.candidate.avz=0;
   pr.original.st=9;pr.candidate.st=9;
   @(negedge clk);while(!vr)@(negedge clk);
   pipeline_resets++;repeat(3)@(negedge clk);
  end
  reset_proof_done=1;
 end
'''+anchor)
 anchor='  if(cycle==200000)begin'
 assert s.count(anchor)==1
 s=s.replace(anchor,anchor+'\n   if(!reset_proof_done || pipeline_resets!=4)$fatal(1,"pipeline reset coverage incomplete");\n   $display("PIPELINE RESETS PASSED phases=%0d",pipeline_resets);')
 return s

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--test',action='store_true');a=ap.parse_args()
 me=psutil.Process();me.nice(psutil.IDLE_PRIORITY_CLASS);me.cpu_affinity([16]);out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
 original=(BASE/'imu_fusion.v').read_text();assert (BASE/'imu_fusion.v').read_bytes()==(PREPARED/'imu_fusion.v').read_bytes()
 sqrt_source=(BASE/'imu_isqrt.v').read_text();candidate=generate(original,sqrt_source)
 (out/'source_imu_fusion.v').write_text(original,encoding='utf-8');(out/'imu_fusion.v').write_text(candidate,encoding='utf-8')
 ref=original.replace('IMU_FUSION_V_INCLUDED','ORIGINAL_IMU_FUSION_V_INCLUDED');ref=re.sub(r'\bimu_fusion\b','fusion_original',ref);ref=re.sub(r'\bimu_div100_3\b','original_divider3',ref)
 (out/'fusion_original.v').write_text(ref,encoding='utf-8');(out/'imu_isqrt.v').write_text(sqrt_source,encoding='utf-8');(out/'imu_recip.v').write_bytes((BASE/'imu_recip.v').read_bytes());(out/'tb_fusion_equivalence.sv').write_text(make_tb(),encoding='utf-8')
 result=dict(schema=1,passed=False,source=str(BASE/'imu_fusion.v'),PNR32_prepared_source_bytes_equal=True,candidate=str(out/'imu_fusion.v'),modified_hardware_inputs=['control/imu_fusion.v'],new_private_modules=['imu_fusion_isqrt_late2'],public_ports_parameters_or_clocks_changed=False,original_math_widths_retained=True,three32bit_absolute_registers=True,three64bit_product_registers=True,original_three64bit_square_registers_retained=True,sqrt_start_and_done_cycle_contract_retained=True,private_square_and_rem_values_retimed_not_full_state_equivalence=True,joint_SYN_or_PNR_started=False,physical_timing_proved=False,programmed=False,execution_policy=dict(pid=me.pid,priority=me.nice(),affinity=me.cpu_affinity()))
 if a.test:
  viv=Path('E:/2026.1/Vivado/bin');runs=[]
  def run(name,args):
   with (out/(name+'_stdout.log')).open('w',encoding='utf-8')as log:
    p=subprocess.run(['cmd','/c',str(viv/(name+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW|subprocess.IDLE_PRIORITY_CLASS)
   runs.append(dict(tool=name,exit_code=p.returncode));assert p.returncode==0,(name,p.returncode)
  run('xvlog',['-sv','imu_isqrt.v','imu_recip.v','fusion_original.v','imu_fusion.v','tb_fusion_equivalence.sv'])
  run('xelab',['-debug','off','-timescale','1ns/1ps','tb_fusion_equivalence','-s','snap'])
  run('xsim',['snap','-runall']);log=(out/'xsim.log').read_text();assert 'TEST PASSED'in log and 'Fatal:'not in log and 'PIPELINE RESETS PASSED phases=4'in log
  m=re.search(r'frames50m=(\d+) frames100k=(\d+) accepted=(\d+) rejected=(\d+) math=(\d+) busy_reset=(\d+)',log);assert m
  result.update(passed=True,runs=runs,clock_cycles=200000,frames50m=int(m[1]),frames100k=int(m[2]),accepted_accel=int(m[3]),rejected_accel=int(m[4]),full_signed24_extreme_random_triples=int(m[5]),ordinary_busy_reset=int(m[6]),pipeline_reset_phases=4,external_outputs_flags_valid_and_engine_publish_cycle_exact=True,sqrt_acc_bitv_cnt_and_rem_after_cnt2_equal_each_cycle=True,stimulus_reference=str(ROOT/'integration/g2/tb_fusion_equivalence.sv'),previous_proof_reused='stimulus and prior published original-vs-retimed proof only; this new candidate was separately simulated against exactPNR32RTL',not_proved=['Joint physical timing/resource/fanout','Full control/board/vehicle behavior','All possible sample/reset sequences byformal equivalence'])
 result['recorded_at']=datetime.now(timezone.utc).isoformat();(out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,ensure_ascii=False))
if __name__=='__main__':main()
