"""One frozen old/new FIFO RTL comparison; no joint EDA or automatic retry."""
from pathlib import Path
from datetime import datetime
import argparse,json,psutil,re,subprocess,time,traceback
from prepare_baseline_fusion_retime import metadata,save
from retry_frozen_synthesis import memory_status
BASE=Path('F:/nav_integration_g2_20261023');SRC=BASE/'stage3_FIFO_async_reset_syntax_candidate1';OUT=BASE/'stage3_FIFO_async_reset_syntax_RTL_probe1';VIV=Path('E:/2026.1/Vivado/bin');GIB=1024**3
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def run():
 assert not OUT.exists();OUT.mkdir();review=load(SRC/'prepared_review.json');bindings=review['bindings']+[review['source_binding']]
 def frozen():
  for b in bindings:assert metadata(Path(b['path']))==b,b
 frozen();me=psutil.Process();state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),candidate=str(SRC),output=str(OUT),automatic_retry=False,no_force=True,SYN_started=False,PNR_started=False,programmed=False)
 def phase(n,**kw):state.update(phase=n,at=datetime.now().astimezone().isoformat(),**kw);save(OUT/'job.json',state)
 runs=[]
 try:
  cmds=[('xvlog',['-sv',str(SRC/'nav_async_fifo_before.v'),str(SRC/'nav_async_fifo.v'),str(SRC/'nav_cdc.v'),str(SRC/'tb_fifo_reset_syntax.sv')]),('xelab',['-debug','off','-timescale','1ns/1ps','tb_fifo_reset_syntax','-s','canonical']),('xsim',['canonical','-runall'])]
  for tool,args in cmds:
   while True:
    mem=memory_status();phase('waiting_memory_'+tool,physical=mem.avail_phys,commit=mem.avail_page)
    if mem.avail_phys>=2*GIB and mem.avail_page>=4*GIB:break
    time.sleep(17)
   frozen();cmd=['cmd','/c',str(VIV/(tool+'.bat')),*args]
   with(OUT/(tool+'_stdout.log')).open('w')as log:
    p=subprocess.Popen(cmd,cwd=OUT,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
    rt=dict(tool=tool,command=cmd,wrapper_pid=p.pid,wrapper_created_epoch=psutil.Process(p.pid).create_time(),started_at=datetime.now().astimezone().isoformat(),admission_physical_bytes=mem.avail_phys,admission_commit_bytes=mem.avail_page);phase('running_'+tool,runtime=rt)
    while p.poll()is None:
     try:rt['processes']=[dict(pid=q.pid,created_epoch=q.create_time(),name=q.name(),command=q.cmdline())for q in psutil.Process(p.pid).children(recursive=True)]
     except(psutil.NoSuchProcess,psutil.AccessDenied):pass
     save(OUT/(tool+'_runtime.json'),rt);time.sleep(1)
    rt.update(exit_code=p.returncode,ended_at=datetime.now().astimezone().isoformat());runs.append(rt);save(OUT/(tool+'_runtime.json'),rt)
   assert p.returncode==0,(tool,p.returncode)
  text=(OUT/'xsim.log').read_text(errors='replace');assert 'TEST PASSED FIFO CANONICAL RESET'in text and not re.search('Fatal:|TEST FAILED',text)
  lines=[x for x in text.splitlines()if x.startswith('FIFO_CANONICAL_PAIR_PASS')];assert len(lines)==3
  metrics=[{k:int(v)for k,v in re.findall(r'(\w+)=(\d+)',x)}for x in lines]
  assert{(r['DW'],r['AW'])for r in metrics}=={(10,11),(10,12),(9,11)}
  frozen();r=dict(state,phase='finite_RTL_complete',passed=True,runtimes=runs,metrics=metrics,pointwise_four_state_identity_cases=16,complete_source_transfer_expression_identity=True,actual_common_reset_and_stopped_domains_tested=True,source_frozen_unchanged=True,not_proven=['C compiler mapping until isolated actual synthesis','full formal FIFO protocol, unilateral reset, physical CDC/reset/MTBF/board','joint source resource/timing/stop signoff'])
  save(OUT/'result.json',r);save(OUT/'job.json',r);print(json.dumps(dict(passed=True,metrics=metrics,output=str(OUT)),ensure_ascii=False));return 0
 except Exception as e:
  r=dict(state,phase='failed_preserved',passed=False,error=str(e),traceback=traceback.format_exc(),runtimes=runs);save(OUT/'result.json',r);save(OUT/'job.json',r);print(r['traceback']);return 1
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');a=p.parse_args();assert a.run;raise SystemExit(run())
