"""Run only the frozen LF repair miter and original128 five-frame regression."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
import traceback
import psutil

from review_stage3_slf128_memories import meta
from retry_frozen_synthesis import memory_status

VIV=Path('E:/2026.1/Vivado/bin');GIB=1024**3


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True);ap.add_argument('--candidate',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();assert not a.out.exists();a.out.mkdir(parents=True)
    prep=json.loads((a.candidate/'preparation.json').read_text());assert prep['source']==meta(a.source/'nav/rtl/match/slf_lf.v')
    frozen=[prep['source'],prep['repaired_source'],*prep['frozen_validation_inputs']]
    miter=a.out/'old_new_miter';model=a.out/'five_frame_model';miter.mkdir();model.mkdir()
    for n in ('old.v','new.v','tb.sv'):shutil.copyfile(a.candidate/n,miter/n)
    vectors=Path('F:/nav_integration_g2_20261023/stage3_SLF128_storage_regression_inputs2')
    for n in ('pts.hex','frm.hex','cand.hex','res.hex','sm.hex','smm.hex','prior.hex','pres.hex','counts.txt','sin_lut.hex','tb_slf.sv'):shutil.copyfile(vectors/n,model/n)
    for name,rel in [('nav_ram_sdp.v','common/nav_ram_sdp.v'),('slf_extract.v','match/slf_extract.v'),('slf_match.v','match/slf_match.v')]:shutil.copyfile(a.source/'nav/rtl'/rel,model/name)
    shutil.copyfile(a.candidate/'slf_lf.v',model/'slf_lf.v')
    (model/'wrapper.sv').write_text('module tb_lf_fixed_model;wire done;tb_slf #(.SLF_LOG2N(7)) model(done);initial begin wait(done);#1;$display("TEST PASSED LF128_ORIGINAL_5FRAME_MODEL");$finish;end endmodule\n',encoding='utf8')
    for p in sorted(a.out.rglob('*')):
        if p.is_file():frozen.append(meta(p))
    (a.out/'frozen_inputs.json').write_text(json.dumps(frozen,indent=2)+'\n')
    me=psutil.Process();state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(a.source),candidate=str(a.candidate),joint_SYN_started=False,PNR_started=False,programmed=False,automatic_retry=False,profiles=[])
    def status(phase,**kwargs):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kwargs);(a.out/'job.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    def verify():
        for row in frozen:assert row==meta(Path(row['path'])),('Frozen validation input changed',row['path'])
    def run(folder,tool,args):
        while True:
            mem=memory_status();status('waiting_memory_'+tool,available_physical_bytes=mem.avail_phys,available_commit_bytes=mem.avail_page)
            if mem.avail_phys>=2*GIB and mem.avail_page>=4*GIB:break
            time.sleep(10)
        verify();cmd=['cmd','/c',str(VIV/(tool+'.bat')),*args]
        with (folder/(tool+'_stdout.log')).open('w')as log:
            p=subprocess.Popen(cmd,cwd=folder,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            rt=dict(command=cmd,wrapper_pid=p.pid,wrapper_created_epoch=psutil.Process(p.pid).create_time(),started_at=datetime.now().astimezone().isoformat(),admission_physical_bytes=mem.avail_phys,admission_commit_bytes=mem.avail_page,cancelled=False)
            status('running_'+folder.name+'_'+tool,current_output=str(folder),runtime=rt)
            t=time.monotonic()
            while p.poll()is None:
                children=[]
                try:children=psutil.Process(p.pid).children(recursive=True)
                except psutil.NoSuchProcess:pass
                rt['processes']=[];private=0
                for q in children:
                    try:
                        rt['processes'].append(dict(pid=q.pid,name=q.name(),created_epoch=q.create_time(),command=q.cmdline()));private+=getattr(q.memory_info(),'private',q.memory_info().rss)
                    except(psutil.NoSuchProcess,psutil.AccessDenied):pass
                mem=memory_status()
                if time.monotonic()-t>1200 or mem.avail_page<512*1024**2 or private>3*GIB:
                    rt.update(cancelled=True,reason='Owned RTL tool exceeded time or memory safety reserve')
                    for q in reversed(children):
                        try:q.terminate()
                        except psutil.NoSuchProcess:pass
                    p.terminate();p.wait();break
                (folder/(tool+'_runtime.json')).write_text(json.dumps(rt,indent=2)+'\n');time.sleep(.5)
            rt.update(exit_code=p.returncode,ended_at=datetime.now().astimezone().isoformat());(folder/(tool+'_runtime.json')).write_text(json.dumps(rt,indent=2)+'\n')
        assert p.returncode==0 and not rt['cancelled'],(tool,p.returncode,str(folder))
        verify();return rt
    try:
        for folder,top,files,marker in [(miter,'tb_slf128_lf_run_read',['old.v','new.v','tb.sv'],'TEST PASSED LF128_RUN_READ'),(model,'tb_lf_fixed_model',['nav_ram_sdp.v','slf_extract.v','slf_lf.v','slf_match.v','tb_slf.sv','wrapper.sv'],'TEST PASSED LF128_ORIGINAL_5FRAME_MODEL')]:
            runtimes=[]
            for tool,args in [('xvlog',['-sv',*files]),('xelab',['-debug','off','-timescale','1ns/1ps',top,'-s','lf_repair']),('xsim',['lf_repair','-runall'])]:runtimes.append(run(folder,tool,args))
            text=(folder/'xsim.log').read_text(errors='replace');assert marker in text and not re.search(r'Fatal:|TEST FAILED',text),text[-2500:]
            row=dict(profile=folder.name,passed=True,output=str(folder),markers=[s for s in text.splitlines()if any(x in s for x in ('TEST PASSED','LF_MITER','SLF_TIMING','一致','STAGE3_SLF128_MODEL_PASS'))],runtimes=runtimes)
            (folder/'result.json').write_text(json.dumps(row,ensure_ascii=False,indent=2)+'\n',encoding='utf8');state['profiles'].append(row);status('profile_complete')
        status('LF_repair_RTL_validation_complete',passed=True,physical_mapping_or_board_accepted=False)
    except Exception as e:status('failed_preserved_no_retry',passed=False,error=str(e),traceback=traceback.format_exc())
    (a.out/'result.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in state.items()if k!='profiles'},ensure_ascii=False));return 0 if state.get('passed')else 1


if __name__=='__main__':raise SystemExit(main())
