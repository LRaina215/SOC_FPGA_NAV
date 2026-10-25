"""One independent memory-gated SYN28; no active PNR exit dependency."""
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import psutil
from retry_frozen_synthesis import memory_status
from prepare_baseline_fusion_retime import metadata,same_bytes
from run_baseline_single_factor_pnr import hardware,inputs

BASE=Path('F:/nav_integration_g2_20261023')
PREP=BASE/'baseline_fusion_action_preview_prepared1'
ORIGIN=BASE/'baseline_fusion_only_syn23'
OUTPUT=BASE/'baseline_fusion_action_preview_syn28'
CONTROL=BASE/'baseline_fusion_action_preview_one_shot'
BUILD_LABEL='SYN28'
REVIEW_SCRIPT=Path(__file__).with_name('review_baseline_action_preview_syn.py')
CHANGED=sorted(['control/policy_action_map.v','control/rl_policy_tick_v5.v','control/mem_policy.v'])
GIB=1024**3

def source_check():
    proof=json.loads((PREP/'prepared_source_review.json').read_text());assert proof['passed']
    assert sorted(proof['modified_rtl_inputs'])==CHANGED
    for row in proof['bindings']:assert metadata(Path(row['path']))==row
    a,b=hardware(ORIGIN),hardware(PREP);assert a.keys()==b.keys()
    assert sorted(n for n in a if not same_bytes(a[n],b[n]))==CHANGED
    for suffixes,count in[({'.hex','.mem','.bin','.mif'},69),({'.h','.vh'},31)]:
        a,b=inputs(ORIGIN,suffixes),inputs(PREP,suffixes);assert a.keys()==b.keys()and len(a)==count
        assert all(same_bytes(a[n],b[n])for n in a)
    assert all(same_bytes(ORIGIN/n,PREP/n)for n in('g2.cst','g2.sdc'))

def main():
    source_check();assert not OUTPUT.exists();CONTROL.mkdir(exist_ok=False)
    me=psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(list(range(psutil.cpu_count())))
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(PREP),output=str(OUTPUT),PNR38_exit_dependency=False,SYN_max_runtime_minutes=40,automatic_PNR=False,programmed=False)
    with(CONTROL/'launch_claim.json').open('x')as file:json.dump(state,file)
    def status(phase,**fields):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**fields)
        tmp=CONTROL/(str(os.getpid())+'.tmp');tmp.write_text(json.dumps(state,indent=2));tmp.replace(CONTROL/'job.json')
    try:
        while True:
            m=memory_status();status('waiting_for_actual_memory_admission',free_bytes=m.avail_phys,available_commit_bytes=m.avail_page,minimum_physical_bytes=2*GIB,minimum_commit_bytes=4*GIB)
            if m.avail_phys>=2*GIB and m.avail_page>=4*GIB:break
            time.sleep(13)
        source_check();assert not OUTPUT.exists()
        cmd=[sys.executable,'-X','utf8',str(Path(__file__).with_name('retry_frozen_synthesis.py')),str(PREP),'--out',str(OUTPUT),'--start-now','--tool-priority','normal']
        with(CONTROL/('full_'+BUILD_LABEL+'_stdout.log')).open('w')as log:
            child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            status('running_unique_full_'+BUILD_LABEL,runner_pid=child.pid,runner_created_epoch=psutil.Process(child.pid).create_time(),command=cmd)
            code=child.wait()
        assert code==0,('full synthesis did not finish successfully',code)
        status(BUILD_LABEL+'_complete_waiting_actual_new_netlist_review')
        cmd=[sys.executable,'-X','utf8',str(REVIEW_SCRIPT),'--source',str(OUTPUT),'--prepared',str(PREP),'--out',str(CONTROL/('actual_'+BUILD_LABEL+'_review.json'))]
        with(CONTROL/'actual_review_stdout.log').open('w')as log:
            child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            status('reviewing_actual_'+BUILD_LABEL+'_RAM_INIT_clocks_26CDC',reviewer_pid=child.pid,reviewer_created_epoch=psutil.Process(child.pid).create_time())
            code=child.wait()
        assert code==0,('actual new netlist review failed',code)
        status('actual_generic_review_passed_product_mapping_and_PNR_preparation_pending')
    except Exception as error:
        import traceback
        status('failed_preserved_no_automatic_retry',error=str(error),traceback=traceback.format_exc());return 1
    return 0

if __name__=='__main__':raise SystemExit(main())
