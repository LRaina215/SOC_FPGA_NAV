from pathlib import Path
import sys,json,re,shutil,datetime,argparse
sys.path.insert(0,'D:/Desktop/SOC FPGA/nav/integration/g2')
from probe_model25_transport import command
BASE=Path('F:/nav_integration_g2_20261023');HERE=Path(__file__).parent
ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);args=ap.parse_args();OUT=args.out
assert not OUT.exists();OUT.mkdir()
ref=BASE/'baseline_fusion_model25_NAV50_prepared4/model25_related_mailbox.v'
new=BASE/'baseline_fusion_model25_private_source_noreset_prepared1/model25_related_mailbox.v'
for p,suffix in[(ref,'ref'),(new,'candidate')]:
 text=p.read_text(encoding='utf-8-sig')
 text=re.sub(r'\bg2_model25_(mailbox|latest|guard)\b',lambda m:m[0]+'_'+suffix,text)
 (OUT/(suffix+'.v')).write_text(text,encoding='utf8')
shutil.copyfile(HERE/'tb_model25_private_source_noreset.sv',OUT/'tb_private_noreset.sv')
for i,(tool,args)in enumerate([('xvlog',['ref.v','candidate.v']),('xvlog',['-sv','tb_private_noreset.sv']),('xelab',['tb_private_noreset','-s','private_snap']),('xsim',['private_snap','-runall'])]):command(OUT,i,tool,args)
log=(OUT/'xsim.log').read_text();assert 'PRIVATE_NORESET_PASS'in log and'Fatal:'not in log
summary=re.search(r'PRIVATE_NORESET_PASS[^\r\n]+',log)[0]
result=dict(passed=True,recorded_at=datetime.datetime.now().astimezone().isoformat(),summary=summary,reference_source=str(ref),candidate_source=str(new),actual_RTL_four_state=True,private_initial_values_forced_or_initialized=False,common_reset_protocol_only=True,new_ModelSim_session_started=False,production_probe3_modified=False,fullG2_or4500_or_physical_timing_or_vehicle_stop_signoff=False)
(OUT/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8');print(json.dumps(result))
