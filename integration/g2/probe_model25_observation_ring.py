"""Finite4state real observation + real scheduler/core-substitute comparison.
No real4500/fullG2 claim, licensed ModelSim, joint synthesis or PNR is started.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re,shutil
from prepare_model25_observation_ring import SOURCE,PREP,FILES
from prepare_baseline_fusion_retime import metadata
from probe_model25_transport import command
def run(out):
 out.mkdir(exist_ok=False);proof=json.loads((PREP/'prepared_source_review.json').read_text());assert proof['prepared']
 for row in proof['bindings']:assert metadata(Path(row['path']))==row
 for folder,suffix in((SOURCE,'ref'),(PREP,'candidate')):
  for name in('rl_observation_v5','rl_policy_tick_v5'):
   text=(folder/'control'/(name+'.v')).read_text()
   text=re.sub(r'\b'+name+r'\b',name+'_'+suffix,text)
   text=text.replace(name.upper()+'_V_INCLUDED',name.upper()+'_V_INCLUDED_'+suffix.upper())
   (out/(name+'_'+suffix+'.v')).write_text(text,encoding='utf8')
 tb=Path(__file__).with_name('tb_model25_observation_ring.sv');shutil.copyfile(tb,out/tb.name)
 commands=[('xvlog',[p.name for p in sorted(out.glob('*.v'))]),('xvlog',['-sv',tb.name]),('xelab',['tb_observation_ring','-s','ring_protocol_snap']),('xsim',['ring_protocol_snap','-runall'])]
 for i,(tool,args)in enumerate(commands):command(out,i,tool,args)
 log=(out/'xsim.log').read_text();assert 'OBSERVATION_RING_PROTOCOL_PASS'in log and'Fatal:'not in log
 summary=re.search(r'OBSERVATION_RING_PROTOCOL_PASS[^\r\n]+',log)[0]
 result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),summary=summary,real_original_observation_math=True,real_original_and_candidate_scheduler=True,core_is_explicit_protocol_substitute=True,test_PERIOD_CLKS=2500,production_PERIOD200000_and125Hz_not_measured=True,private_RAM_not_initialized_or_forced=True,original_core_and_input_mem_and_weights_untouched=True,full4500_425trace_actions_and_physical_timing_not_accepted=True,source=str(PREP),source_bindings=proof['bindings'],no_SYN_PNR_or_ModelSim_started=True)
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();print(json.dumps(run(a.out),indent=2))
