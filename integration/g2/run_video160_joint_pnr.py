"""One reviewed video joint PNR, all new video paths timed, no deadlines."""
import argparse,json,re,shutil,time
from pathlib import Path
from datetime import datetime
import psutil
from retry_frozen_synthesis import memory_status
from preflight import validate
from bounded_tool import run_tool
from collect_pnr_result import finalize_gate
from check_setup_endpoint_coverage import scan_report

REFERENCE=Path('F:/nav_integration_g2_20261023/K_pnr63_s50p2r0')
TOOL=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')

def save(p,v):p.write_text(json.dumps(v,indent=2),encoding='utf-8')
def run(source,out,review):
    assert not out.exists()
    proof=json.loads(review.read_text(encoding='utf-8'))
    assert proof['passed_for_initial_exploratory_PNR']and Path(proof['build']).resolve()==source.resolve()
    assert proof['source_BSRAM']==307 and proof['physical_split_prediction']==318
    out.mkdir()
    shutil.copyfile(source/'impl/gwsynthesis/g2_top.vg',out/'g2_top.vg')
    shutil.copyfile(source/'g2.cst',out/'g2.cst');shutil.copyfile(review,out/'actual_SYN_review.json')
    sdc=(source/'g2.sdc').read_text(encoding='utf-8')
    assert sdc.count('set_false_path')==26 and sdc.count('set_max_delay')==26
    assert len(re.findall(r'^create_clock ',sdc,re.M))==3 and 'create_generated_clock -name lcd33'in sdc
    sdc+='\nreport_timing -setup -max_paths 5000 -max_common_paths 1\n'
    sdc+='report_timing -hold -max_paths 5000 -max_common_paths 1\n'
    (out/'g2.sdc').write_text(sdc,encoding='utf-8',newline='\n')
    tcl=(REFERENCE/'run.tcl').read_text(encoding='utf-8').replace(REFERENCE.as_posix(),out.as_posix())
    assert 'set_option -place_option 2'in tcl and 'set_option -route_option 0'in tcl
    tcl+='\nexit\n';(out/'run.tcl').write_text(tcl,encoding='utf-8',newline='\n')
    save(out/'manifest.json',dict(source=str(source),source_manifest=json.loads((source/'manifest.json').read_text(encoding='utf-8')),
        reviewed_netlist=str(review),reused_completed_synthesis=True,place_option=2,route_option=0,
        original26_hold_only_max_delay_kept=True,new_video_paths_fully_timed=True,
        diagnostic_only=True,programmed=False,deployment_accepted=False))
    me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS)
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(source),output=str(out),
        total_limit_s=0,silent_limit_s=0,programmed=False,automatic_retry=False)
    def status(phase,**kw):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw)
        save(out/'job.json',state);print(json.dumps(dict(phase=phase,**kw)),flush=True)
    def wait_memory():
        while True:
            m=memory_status()
            status('waiting_for_actual_memory_admission',physical_available=m.avail_phys,commit_available=m.avail_page)
            if m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3:return
            time.sleep(30)
    try:
        wait_memory()
        state['effective_options']=validate(TOOL,out,[s for s in tcl.splitlines()if s.strip()not in ('run pnr','exit')],50,1,dict(replicate_resources=1,place_option=2,route_option=0))
        assert (out/'g2_top.vg').read_bytes()==(source/'impl/gwsynthesis/g2_top.vg').read_bytes()
        wait_memory();status('starting_unique_physical_run')
        result=run_tool([str(TOOL),'run.tcl'],out,max_runtime_s=0,max_no_progress_s=0,capture_system_telemetry=True)
        status('physical_tool_exited',runtime=result)
        finalize_gate(out)
        tr=out/'impl/pnr/g2_top.tr'
        if tr.exists():save(out/'setup_endpoint_coverage.json',scan_report(tr))
        status('physical_complete_requires_actual_review',runtime=result)
        return result['tool_exit']
    except Exception as e:
        import traceback
        status('failed_preserved_no_retry',error=str(e),traceback=traceback.format_exc());return 1

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--review',type=Path,required=True)
    a=p.parse_args();raise SystemExit(run(a.source.resolve(),a.out.resolve(),a.review.resolve()))
