"""Prepare only a private source-data reset candidate; starts no EDA/simulator."""
from pathlib import Path
from datetime import datetime
import json,re,shutil,difflib,argparse
BASE=Path('F:/nav_integration_g2_20261023')
SRC=BASE/'baseline_fusion_model25_NAV50_prepared4'
DST=BASE/'baseline_fusion_model25_private_source_noreset_prepared1'
OUT=BASE/'model25_private_source_data_noreset_candidate1'
load=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def same(a,b):
 if a.stat().st_size!=b.stat().st_size:return False
 with a.open('rb')as x,b.open('rb')as y:
  while True:
   m,n=x.read(1024*1024),y.read(1024*1024)
   if m!=n:return False
   if not m:return True

def derive(s):
 assert s.count('holding<=0;')==s.count('holding<=src_data;')==1
 assert s.count('last_sent<=0;')==s.count('last_sent<=src_data;')==1
 oldmail=s.split('module g2_model25_latest',1)[0]
 oldlatest=s.split('module g2_model25_latest',1)[1].split('module g2_model25_guard',1)[0]
 s=s.replace('holding<=0;','').replace('holding<=src_data;','').replace('last_sent<=0;','').replace('last_sent<=src_data;','')
 marker=' assign src_ready=request==ack2;\n'
 s=s.replace(marker,marker+'''\n // Private payload is deliberately arbitrary before its first accepted write.
 // It may write during reset: public request/ACK/valid remain reset, so no
 // destination can observe it. Do not add reset back through a per-bit CE.
 always @(posedge src_clk)
  if(src_valid && src_ready)holding<=src_data;
''',1)
 marker=' wire send=ready&&(!sent||src_data!=last_sent);\n'
 s=s.replace(marker,marker+'''\n // sent remains asynchronously reset. sent=0 forces a complete fresh write
 // before the unreset last_sent value can affect later change detection.
 always @(posedge src_clk)
  if(send)last_sent<=src_data;
''',1)
 mailbox=s.split('module g2_model25_latest',1)[0]
 latest=s.split('module g2_model25_latest',1)[1].split('module g2_model25_guard',1)[0]
 block=lambda t:re.search(r' always @\(posedge src_clk or negedge src_rst_n\).*?(?=\n always @|\n g2_model25_mailbox|\nendmodule)',t,re.S)[0]
 assert block(oldmail).replace('holding<=0;','').replace('holding<=src_data;','')==block(mailbox)
 assert block(oldlatest).replace('last_sent<=0;','').replace('last_sent<=src_data;','')==block(latest)
 dst=lambda t:t.split(' always @(posedge dst_clk or negedge dst_rst_n)',1)[1]
 assert dst(mailbox)==dst(oldmail)
 return s

def main():
 assert not DST.exists()
 skip=shutil.ignore_patterns('impl','__pycache__','finite_proof_summary.json','prepared_source_review.json','model25_delta.patch')
 shutil.copytree(SRC,DST,ignore=skip)
 old=(SRC/'model25_related_mailbox.v').read_text(encoding='utf8');new=derive(old)
 assert old.split('module g2_model25_guard',1)[1]==new.split('module g2_model25_guard',1)[1]
 (DST/'model25_related_mailbox.v').write_text(new,encoding='utf8')
 m=load(SRC/'manifest.json')
 def relocate(v):
  if isinstance(v,str):return v.replace(str(SRC),str(DST)).replace(SRC.as_posix(),DST.as_posix())
  if isinstance(v,list):return[relocate(x)for x in v]
  if isinstance(v,dict):return{k:relocate(x)for k,x in v.items()}
  return v
 m=relocate(m);m.update(candidate='private_source_holding_and_latest_last_sent_noreset',stage='source_candidate_pending_proof',source_preparation_only=True,synthesis_complete=False,automatic_pnr=False,automatic_SYN_PNR=False,deployment_accepted=False,modified_rtl_inputs=['model25_related_mailbox.v'],new_rtl_inputs=[],base_prepared=str(SRC),private_data_powerup_assumption='arbitrary, not zero')
 save(DST/'manifest.json',m)
 run=(DST/'run.tcl').read_text(encoding='utf8');run=run.replace(str(SRC),str(DST)).replace(SRC.as_posix(),DST.as_posix())
 (DST/'run.tcl.disabled_pending_proof').write_text(run,encoding='utf8');(DST/'run.tcl').unlink()
 rel=[Path(n).relative_to(SRC)for n in load(SRC/'manifest.json')['source_files']]
 assert len(rel)==66
 changed=[n.as_posix()for n in rel if not same(SRC/n,DST/n)];assert changed==['model25_related_mailbox.v']
 assets=[n.relative_to(SRC)for n in SRC.rglob('*')if n.is_file()and n.suffix.lower()in('.hex','.mem','.bin','.mif')]
 headers=[n.relative_to(SRC)for n in SRC.rglob('*')if n.is_file()and n.suffix.lower()in('.h','.vh')]
 assert len(assets)==69 and len(headers)==31
 assert all(same(SRC/n,DST/n)for n in assets+headers)
 fw={}
 for d in('fw','firmware'):
  names=[n.relative_to(SRC)for n in(SRC/d).rglob('*')if n.is_file()and'__pycache__'not in n.parts]
  assert all(same(SRC/n,DST/n)for n in names);fw[d]=len(names)
 (OUT/'source_delta.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='original/model25_related_mailbox.v',tofile='candidate/model25_related_mailbox.v')),encoding='utf8')
 save(OUT/'prepared_candidate.json',dict(preparation_passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SRC),candidate=str(DST),hardware_inputs=66,only_changed_input=changed,unchanged_hardware_inputs=65,unchanged_assets=69,unchanged_headers=31,unchanged_firmware_files=fw,public_source_control_reset_blocks_unchanged_after_private_assignment_removal=True,public_destination_reset_block_byte_identical=True,clock_guard_byte_identical=True,no_clock_capacity_weight_period_or_stop_change=True,no_assumption_private_register_INIT_zero=True,not_formal_or_simulation_or_physical_acceptance=True,SYN_started=False,PNR_started=False,simulation_started=False,automatic_launch=False,run_tcl_disabled_pending_proof=True))
 print(json.dumps(dict(candidate=str(DST),only_changed_input=changed,prepared_only=True),ensure_ascii=False))
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,default=SRC);ap.add_argument('--prepared',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();SRC=a.source;DST=a.prepared;OUT=a.out;OUT.mkdir(exist_ok=False);main()
