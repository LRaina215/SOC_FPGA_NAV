"""Wait for reserve, then consume one already-validated settings-only module SYN.

No new RTL test, settings tool, full synthesis, PNR or programming is started.
Only the one isolated action module SYN is allowed, with a permanent claim.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import psutil

from retry_frozen_synthesis import memory_status


GIB=1024**3


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def metadata(path):
    path=Path(path).resolve();stat=path.stat()
    return dict(path=str(path),bytes=stat.st_size,mtime_ns=stat.st_mtime_ns)


def save(path,value):
    path=Path(path);temporary=path.with_name(path.name+'.'+str(os.getpid())+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    temporary.replace(path)


def check_unstarted(candidate,out):
    proof=load(candidate/'result.json')
    assert proof['passed'] and proof['frames']==8770
    assert proof['exhaustive_scale_values_checked']==32772
    assert Path(proof['candidate']).resolve()==(candidate/'policy_action_map.v').resolve()
    for row in proof['source_bindings']:
        assert metadata(row['path'])==row,('proved source changed',row['path'])
    assert out.is_dir() and not (out/'impl').exists()
    assert not any((out/name).exists() for name in ('gw_sh.log','module_runtime.json','launch_claim.json')),'module SYN already started; never repeat'
    expected=(candidate/'policy_action_map.v').read_text(encoding='utf-8').replace('module policy_action_map #(','module g2_top #(',1)
    assert (out/'action.v').read_text(encoding='utf-8')==expected
    assert (out/'clock.sdc').read_text(encoding='utf-8')=='create_clock -name clk -period 20 [get_ports {clk}]\n'
    script=(out/'run.tcl').read_text(encoding='utf-8')
    assert [line.strip() for line in script.splitlines() if line.strip().startswith('run ')]==['run syn']
    assert 'set_device -device_version C GW5AT-LV138PG484AC1/I0' in script
    assert 'set_option -global_freq 50' in script
    assert (out/'preflight.log').is_file() and (out/'effective_options.tcl').is_file()
    return proof


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate-dir',type=Path,required=True)
    parser.add_argument('--module-out',type=Path,required=True)
    parser.add_argument('--state-out',type=Path,required=True)
    parser.add_argument('--continuation-attempt',type=int,default=0,
        help='explicit recovery only after an unstarted memory-gate refusal; preserve every old claim/job')
    args=parser.parse_args()
    candidate,out,state_out=[p.resolve() for p in (args.candidate_dir,args.module_out,args.state_out)]
    me=psutil.Process();me.nice(psutil.IDLE_PRIORITY_CLASS);me.cpu_affinity([16])
    check_unstarted(candidate,out)
    assert not (state_out/'job.json').exists(),'preserve existing controller result'
    state_out.mkdir(parents=True,exist_ok=True)
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),
        candidate=str(candidate),module_out=str(out),state_out=str(state_out),
        recorded_at=datetime.now().astimezone().isoformat(),programmed=False,
        scope='One isolated module SYN only; no existing build is touched',
        minimum_physical_bytes=2*GIB,minimum_commit_bytes=4*GIB)
    assert args.continuation_attempt>=0
    if args.continuation_attempt:
        previous='continuation_claim.json' if args.continuation_attempt==1 else 'continuation_claim_attempt_'+str(args.continuation_attempt-1)+'.json'
        prior=load(out/previous);prior_state=Path(prior['state_out'])
        old=load(prior_state/'job.json')
        assert old['phase']=='failed_preserved_without_retry'
        assert Path(old['candidate']).resolve()==candidate and Path(old['module_out']).resolve()==out
        assert "2/4GiB launch recheck" in (prior_state/'module_SYN_stdout.log').read_text(encoding='utf-8')
        for key,stamp in [('controller_pid','controller_created_epoch'),('child_pid','child_created_epoch')]:
            try:
                proc=psutil.Process(old[key])
                assert abs(proc.create_time()-old[stamp])>.01,'previous controller/child still active'
            except psutil.NoSuchProcess:pass
        state['unstarted_memory_refusal_recovery_of']=str(prior_state/'job.json')
        state['continuation_attempt']=args.continuation_attempt
    # Per-module, not only per-controller-directory, so another state path
    # cannot create a second waiter for this same preserved settings stage.
    claim_name='continuation_claim.json' if not args.continuation_attempt else 'continuation_claim_attempt_'+str(args.continuation_attempt)+'.json'
    with (out/claim_name).open('x',encoding='utf-8') as file:
        json.dump(state,file,ensure_ascii=False,indent=2)
    def status(phase,**extra):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**extra)
        save(state_out/'job.json',state)
    files=[candidate/'result.json',candidate/'policy_action_map.v']+[out/name for name in
           ('action.v','clock.sdc','run.tcl','preflight.log','effective_options.tcl')]
    bindings=[metadata(path) for path in files]
    command=[sys.executable,'-X','utf8',str(Path(__file__).with_name('measure_action_constant_csa.py')),
        '--candidate-dir',str(candidate),'--out',str(out),'--resume-settings-only','--wait-for-memory']
    try:
        while True:
            check_unstarted(candidate,out)
            assert [metadata(path) for path in files]==bindings,'frozen candidate/settings changed while waiting'
            memory=memory_status()
            status('waiting_for_only_module_SYN_memory',available_physical_bytes=memory.avail_phys,
                available_commit_bytes=memory.avail_page,input_bindings=bindings,command=command)
            if memory.avail_phys>=2*GIB and memory.avail_page>=4*GIB:
                break
            time.sleep(13)
        with (state_out/'module_SYN_stdout.log').open('w',encoding='utf-8') as log:
            child=subprocess.Popen(command,cwd=Path(__file__).resolve().parents[2],
                stdout=log,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW|subprocess.IDLE_PRIORITY_CLASS)
            identity=psutil.Process(child.pid)
            status('running_only_isolated_module_SYN',child_pid=child.pid,
                   child_created_epoch=identity.create_time(),child_command=command)
            code=child.wait()
        assert code==0,('module child refused or failed; preserve without retry',code)
        result=load(out/'result.json')
        assert result['passed'] and result['runtime']['tool_exit']==0 and not result['runtime']['cancelled']
        removed=result['target_action_DSP_removed_in_isolated_mapping']
        status('module_SYN_complete_DSP_removed' if removed else 'module_SYN_complete_DSP_still_present',
            child_exit=code,module_result=str(out/'result.json'),resources=result['resources'],
            DSP_primitives=result['DSP_primitives'],DSP_removed=removed,
            no_joint_SYN_PNR_or_programming_started=True,physical_timing_passed=False)
        return int(not removed)
    except Exception as error:
        import traceback
        status('failed_preserved_without_retry',error=str(error),error_type=type(error).__name__,traceback=traceback.format_exc())
        return 1


if __name__=='__main__':
    raise SystemExit(main())
