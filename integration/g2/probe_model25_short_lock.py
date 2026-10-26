from pathlib import Path
from datetime import datetime
import argparse,json,re,shutil
from probe_model25_transport import command,ROOT
from prepare_baseline_fusion_retime import metadata,save
def run(source,out,expect_fixed):
 assert not out.exists();out.mkdir(parents=True)
 rtl=source/'model25_related_mailbox.v';tb=ROOT/'integration/g2/tb_model25_short_lock.sv'
 shutil.copyfile(rtl,out/rtl.name);(out/tb.name).write_text('`define EXPECT_STICKY '+str(int(expect_fixed))+'\n'+tb.read_text())
 commands=[('xvlog',[str(out/rtl.name)]),('xvlog',['-sv',str(out/tb.name)]),('xelab',['tb_model25_short_lock','-s','short_lock_snap']),('xsim',['short_lock_snap','-runall'])]
 for i,(tool,args)in enumerate(commands):command(out,i,tool,args)
 log=(out/'xsim.log').read_text()
 marker='MODEL25_SHORT_LOCK_FIXED_PASS'if expect_fixed else'MODEL25_SHORT_LOCK_COUNTEREXAMPLE'
 assert marker in log and'Fatal:'not in log
 result=dict(recorded_at=datetime.now().astimezone().isoformat(),fixture_completed=True,safety_contract_pass=expect_fixed,
   summary=re.search(marker+r'[^\r\n]+',log)[0],source=str(source),binding=[metadata(rtl),metadata(out/tb.name)],physical_PLL_fault_pulse_minimum_not_verified=True)
 save(out/'result.json',result);return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--expect-fixed',action='store_true');a=p.parse_args()
 print(json.dumps(run(a.source,a.out,a.expect_fixed),indent=2))
