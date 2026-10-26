"""Validate the selected source-only packed-high common-enable repair."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import time
import traceback
import psutil
from retry_frozen_synthesis import memory_status
from review_stage3_slf128_memories import meta


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--candidate',type=Path,required=True);ap.add_argument('--source',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();assert not a.out.exists();a.out.mkdir(parents=True)
    prep=json.loads((a.candidate/'preparation.json').read_text());assert prep['changed_statements']==15
    for n in('old.v','new.v','tb.sv'):shutil.copyfile(a.candidate/n,a.out/n)
    shutil.copyfile(a.source/'src/mem/weights.mem',a.out/'weights.mem');(a.out/'src/mem').mkdir(parents=True)
    for f in(a.source/'src/mem').glob('weight9_*.mem'):shutil.copyfile(f,a.out/'src/mem'/f.name)
    bindings=[prep['ancestor'],prep['repaired_source']]+[meta(p)for p in a.out.rglob('*')if p.is_file()];(a.out/'frozen_inputs.json').write_text(json.dumps(bindings,indent=2)+'\n')
    me=psutil.Process();r=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),passed=False,candidate=str(a.candidate),SYN_started=False,PNR_started=False,automatic_retry=False,runtimes=[])
    def status(phase,**kw):r.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);(a.out/'job.json').write_text(json.dumps(r,indent=2)+'\n')
    try:
        for tool,args in [('xvlog',['-sv','old.v','new.v','tb.sv']),('xelab',['-debug','off','-timescale','1ns/1ps','tb_weight_common_en','-s','common_en']),('xsim',['common_en','-runall'])]:
            while True:
                m=memory_status();status('waiting_memory_'+tool,available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
                if m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3:break
                time.sleep(10)
            for b in bindings:assert b==meta(Path(b['path'])),b['path']
            cmd=['cmd','/c',str(Path('E:/2026.1/Vivado/bin')/(tool+'.bat')),*args]
            with(a.out/(tool+'_stdout.log')).open('w')as stream:
                p=subprocess.Popen(cmd,cwd=a.out,stdout=stream,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW);rt=dict(command=cmd,pid=p.pid,created_epoch=psutil.Process(p.pid).create_time(),started_at=datetime.now().astimezone().isoformat());status('running_'+tool,runtime=rt)
                p.wait(timeout=300);rt.update(exit_code=p.returncode,ended_at=datetime.now().astimezone().isoformat());r['runtimes'].append(rt)
            assert p.returncode==0,(tool,p.returncode)
        log=(a.out/'xsim.log').read_text(errors='replace');assert 'TEST PASSED WEIGHT_COMMON_EN'in log and not re.search(r'Fatal:|COMMON_EN_DIFF|COMMON_EN_HOLD|COMMON_EN_GOLD',log)
        status('source_contract_passed',passed=True,markers=[s for s in log.splitlines()if 'TEST PASSED'in s])
    except Exception as e:status('failed_preserved',passed=False,error=str(e),traceback=traceback.format_exc())
    (a.out/'result.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r));return 0 if r['passed']else 1


if __name__=='__main__':raise SystemExit(main())
