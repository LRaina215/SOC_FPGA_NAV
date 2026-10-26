"""Elaborate real model25 chain and separately simulate unmodified publicPLL.

No inference test double is used in the model elaboration. Elaboration is not a
functional4500 test or implementation. The clock fixture advances only the
vendor publicPLL model; no hardware clock is forced.
"""
import argparse,json,re
from pathlib import Path
from probe_model25_transport import command,LIB
from prepare_baseline_fusion_retime import metadata,save
from datetime import datetime

FILES=['control/fc_v4_round_sat.v','control/imu_fusion.v','control/imu_isqrt.v','control/imu_recip.v',
 'control/leg_ik_ff.v','control/policy_action_map.v','control/rl_observation_v5.v','control/rl_policy_tick_v5.v',
 'fc_v4_top.v','fixed4500_weight_rom.v','control/mem_policy.v','control/mem_policy_related25.v','model25_related_mailbox.v']
TB='''`timescale 1ns/1ps
module tb_model25_pll;
 reg clk=0,reset=1;always #10 clk=~clk;
 wire out,lock;GSR GSR(.GSRI(1'b1));
 Gowin_Model25_PLL pll(.clkin(clk),.reset(reset),.lock(lock),.enclk0(1'b1),.clkout0(out));
 real last=0;integer measured=0;real period;
 initial begin
  #200;reset=0;wait(lock===1'b1);repeat(5)@(posedge out);
  last=$realtime;
  repeat(1000)begin @(posedge out);period=$realtime-last;
   if(period<39.999||period>40.001)$fatal(1,"PLL25 period=%f",period);
   last=$realtime;measured=measured+1;
  end
  reset=1;#50;if(lock!==0)$fatal(1,"PLLreset did not clear lock");
  $display("MODEL25_PUBLIC_PLL_PASS periods=%0d memoryless25MHz=40ns",measured);$finish;
 end
 initial begin #2000000;$fatal(1,"clock fixture watchdog");end
endmodule
'''

def run(source,out):
 assert not out.exists();out.mkdir(parents=True)
 files=[source/n for n in FILES]
 command(out,0,'xvlog',[str(p)for p in files])
 command(out,1,'xelab',['mem_policy_related25','-s','real_model25_elaborated'])
 elaboration_log=(out/'1_xelab.log').read_text(errors='replace')
 assert 'ERROR:'not in elaboration_log
 clock=out/'clock';clock.mkdir()
 text=LIB.read_text();models=[]
 for name in('GSR','PLL'):
  m=re.search(r'\bmodule '+name+r'\s*\(.*?\bendmodule',text,re.S);assert m;models.append(m[0])
 (clock/'vendor_public_PLL.v').write_text('`timescale 1ns/1ps\n'+'\n'.join(models))
 (clock/'tb_model25_pll.sv').write_text(TB)
 for i,(tool,args)in enumerate([
  ('xvlog',[str(clock/'vendor_public_PLL.v'),str(source/'model25_pll/model25_pll.v')]),
  ('xvlog',['-sv',str(clock/'tb_model25_pll.sv')]),
  ('xelab',['tb_model25_pll','-s','clock_snap']),('xsim',['clock_snap','-runall'])]):command(clock,i,tool,args)
 log=(clock/'xsim.log').read_text();assert 'MODEL25_PUBLIC_PLL_PASS'in log and'Fatal:'not in log
 result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),real_model25_chain_elaborated_without_inference_substitute=True,
   public_PLL_finite_summary=re.search(r'MODEL25_PUBLIC_PLL_PASS[^\r\n]+',log)[0],
   original_vendor_model=metadata(LIB),hardware_inputs=[metadata(p)for p in files]+[metadata(source/'model25_pll/model25_pll.v')],
   full4500_numeric_or_8ms_WCET_passed=False,physical_clock_global_resources_setup_hold_passed=False,new_SYN_PNR_started=False)
 save(out/'result.json',result);return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 print(json.dumps(run(a.source,a.out),indent=2))
