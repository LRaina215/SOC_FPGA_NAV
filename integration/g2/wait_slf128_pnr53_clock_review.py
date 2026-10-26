"""One read-only PNR53 preservation audit after actual memory admission.

The existing PNR54 remains untouched. This controller reserves its remaining
observed-process growth budget and never kills, pauses or restarts a tool.
"""
from pathlib import Path
from datetime import datetime
import json,subprocess,sys,time,psutil
from retry_frozen_synthesis import memory_status
from prepare_baseline_fusion_retime import metadata

R=Path(__file__).resolve().parents[2]
B=Path('F:/nav_integration_g2_20261023')
C=B/'PNR53_clock_stop_review_wait1'
OUT=B/'PNR53_actual_clock_stop_review1'
G=1024**3


def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf8')


def main():
    assert not C.exists() and not OUT.exists(),'No duplicate controller or audit'
    rt=json.loads((B/'SLF128_storage_place1_pnr53/tool_runtime.json').read_text())
    assert rt.get('ended_at') and rt.get('tool_exit')==0 and rt['cancelled'] is False
    C.mkdir();me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),parent_pid=me.ppid(),
        output=str(OUT),EDA_started=False,PNR54_modified=False)
    save(C/'launch_claim.json',state)
    entry=R/'integration/g2/review_slf128_pair_physical_clock_stop.py'
    bindings=[metadata(p) for p in [entry,B/'SLF128_storage_safeFW_syn43/impl/gwsynthesis/g2_top.vg',
        B/'SLF128_storage_place1_pnr53/impl/pnr/g2_top.vo']]
    command=[sys.executable,'-B','-X','utf8',str(entry),'--source',str(B/'SLF128_storage_safeFW_syn43'),
        '--physical',str(B/'SLF128_storage_place1_pnr53'),'--out',str(OUT),'--review']
    def status(phase,**kw):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw)
        temp=C/'job.tmp';save(temp,state);temp.replace(C/'job.json')
    try:
        while True:
            rows=[]
            for p in psutil.process_iter(['name']):
                try:
                    if (p.info['name'] or '').lower()!='gw_sh.exe':continue
                    if Path(p.cwd()).resolve()!=(B/'SLF128_storage_place2_pnr54').resolve():continue
                    m=p.memory_info();rows.append(dict(pid=p.pid,created_epoch=p.create_time(),rss=m.rss,private=m.private))
                except(psutil.NoSuchProcess,psutil.AccessDenied):continue
            mp=2*G+sum(max(0,int(3.5*G)-x['rss']) for x in rows)
            mc=4*G+sum(max(0,int(3.5*G)-x['private']) for x in rows)
            m=memory_status();status('waiting_actual_memory_plus_PNR54_growth',physical_bytes=m.avail_phys,
                commit_bytes=m.avail_page,minimum_physical=mp,minimum_commit=mc,PNR54_processes=rows)
            if m.avail_phys>=mp and m.avail_page>=mc:break
            time.sleep(17)
        assert all(metadata(Path(x['path']))==x for x in bindings),'Frozen review input changed'
        assert not OUT.exists()
        save(C/'actual_review_claim.json',dict(command=command,admission=state,bindings=bindings))
        status('running_read_only_review',command=command)
        with (C/'stdout.log').open('w',encoding='utf8') as stdout,(C/'stderr.log').open('w',encoding='utf8') as stderr:
            child=subprocess.Popen(command,cwd=R,stdout=stdout,stderr=stderr,creationflags=subprocess.CREATE_NO_WINDOW)
            cp=psutil.Process(child.pid);status('running_read_only_review',child_pid=cp.pid,child_created_epoch=cp.create_time())
            exitcode=child.wait()
        proof=OUT/'result.json'
        passed=exitcode==0 and proof.is_file() and json.loads(proof.read_text())['passed']
        status('complete' if passed else 'failed_preserved_no_retry',tool_exit=exitcode,passed=passed,result=str(proof))
        return int(not passed)
    except BaseException as exc:
        status('failed_preserved_no_retry',error=f'{type(exc).__name__}: {exc}')
        raise


if __name__=='__main__':raise SystemExit(main())
