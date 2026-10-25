"""One independent action-CSA module proof -> SYN27 -> actual review -> PNR37.

Neither existing PNR35/36 nor their exit state is an input. Every expensive
stage requires the real2/4GiB reserve; PNR has zero total/silent deadline.
"""
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import psutil

import run_baseline_single_factor_pnr as engine
from prepare_baseline_action_csa import ORIGIN,BASELINE,CANDIDATE,ACTION,bind_contract,prepare
from retry_frozen_synthesis import memory_status


ROOT=Path('F:/nav_integration_g2_20261023')
PREPARED=ROOT/'baseline_fusion_action_CSA_prepared1'
SOURCE=ROOT/'baseline_fusion_action_CSA_syn27'
OUTPUT=ROOT/'baseline_fusion_action_CSA_pnr37'
CONTROL=ROOT/'baseline_fusion_action_CSA_one_shot'
MODULE_CONTROL=ROOT/'action_constant_csa_module_one_shot/continuation_attempt_1'
REVIEW=CONTROL/'actual_SYN27_netlist_review.json'
PROFILE=dict(source=SOURCE,prepared=PREPARED,output=OUTPUT,changed=ACTION,interval=17)
GIB=1024**3


def load(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def save(path,value):
    path=Path(path);temporary=path.with_name(path.name+'.'+str(os.getpid())+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    temporary.replace(path)


def alive(pid,created):
    try:return abs(psutil.Process(pid).create_time()-created)<.01
    except psutil.NoSuchProcess:return False


def check_source(profile):
    assert profile['source']==SOURCE and profile['prepared']==PREPARED and profile['output']==OUTPUT
    job=engine.load_synthesis_job(SOURCE);manifest=load(SOURCE/'manifest.json')
    assert job['phase']=='synthesis_complete' and job['tool_exit']==0
    assert Path(job['source']).resolve()==PREPARED.resolve()
    assert Path(manifest['frozen_source_build']).resolve()==PREPARED.resolve()
    assert manifest['action_map_constant_CSA'] and manifest['fusion_norm_pipeline']
    assert not any(manifest.get(k) for k in ('action_map_exact_widths','fusion_square_pipeline2','pose_predecode','control_weights_changed','programmed'))
    assert manifest['clock_core_mhz']==50 and manifest['wide_split']==1
    proof=load(PREPARED/'prepared_source_review.json')
    assert proof['passed'] and proof['modified_rtl_inputs']==[ACTION]
    assert [proof[k] for k in ('unchanged_hardware_inputs','identical_assets','identical_headers')]==[62,69,31]
    bind_contract(SOURCE,PREPARED,proof)
    old,prep,new=[engine.hardware(path) for path in (ORIGIN,PREPARED,SOURCE)]
    assert old.keys()==prep.keys()==new.keys()
    assert sorted(name for name in old if not engine.equal_bytes(old[name],prep[name]))==[ACTION]
    assert all(engine.equal_bytes(prep[name],new[name]) for name in prep)
    for suffixes,count in [(engine.ASSETS,69),({'.h','.vh'},31)]:
        original,frozen,actual=[engine.inputs(path,suffixes) for path in (ORIGIN,PREPARED,SOURCE)]
        assert original.keys()==frozen.keys()==actual.keys() and len(original)==count
        assert all(engine.equal_bytes(original[name],frozen[name]) and engine.equal_bytes(original[name],actual[name]) for name in original)
    for directory,count in [('fw',61),('firmware',92)]:
        paths=[p for p in (ORIGIN/directory).rglob('*') if p.is_file() and '__pycache__' not in p.parts]
        assert len(paths)==count
        assert all(engine.equal_bytes(path,PREPARED/path.relative_to(ORIGIN)) and engine.equal_bytes(path,SOURCE/path.relative_to(ORIGIN)) for path in paths)
    assert engine.equal_bytes(ORIGIN/'impl/gwsynthesis/g2_top.vg',BASELINE/'g2_top.vg')
    for name in ('g2.cst','g2.sdc'):
        assert all(engine.equal_bytes(BASELINE/name,path/name) for path in (ORIGIN,PREPARED,SOURCE))
    assert (SOURCE/'g2.sdc').read_text().splitlines()==[
        'create_clock -name clk50 -period 20 [get_ports {clk50}]',
        'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]']
    after=engine.load_synthesis_job(SOURCE)
    assert all(after.get(key)==job.get(key) for key in ('phase','source','tool_exit','tool_pid','tool_created_epoch','synthesis_started_at'))
    return manifest


def check_review():
    result=engine.check_review(PROFILE,REVIEW)
    assert result['target_action_DSP_removed_in_actual_joint_mapping'] is True
    assert result['action_CSA_source_numeric_cycle_and_module_mapping_bound'] is True
    assert result['action_constant_CSA_actual_mapping']['DSP_cells']=={}
    return result


def main():
    me=psutil.Process();initial_affinity=me.cpu_affinity()
    me.nice(psutil.IDLE_PRIORITY_CLASS);me.cpu_affinity([18])
    CONTROL.mkdir(parents=True,exist_ok=True)
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),
        source=str(SOURCE),prepared=str(PREPARED),output=str(OUTPUT),actual_review=str(REVIEW),
        module_control=str(MODULE_CONTROL),total_PNR_limit_s=0,silent_PNR_limit_s=0,
        programmed=False,deployment_accepted=False)
    with (CONTROL/'chain_claim.json').open('x',encoding='utf-8') as file:json.dump(state,file,indent=2)
    def status(phase,**extra):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**extra)
        save(CONTROL/'job.json',state)
    def wait_memory(phase):
        while True:
            memory=memory_status()
            status(phase,available_physical_bytes=memory.avail_phys,available_commit_bytes=memory.avail_page,
                minimum_physical_bytes=2*GIB,minimum_commit_bytes=4*GIB)
            if memory.avail_phys>=2*GIB and memory.avail_page>=4*GIB:return
            time.sleep(17)
    def child(command,phase,logname):
        with (CONTROL/logname).open('w',encoding='utf-8') as log:
            process=subprocess.Popen(command,cwd=Path(__file__).resolve().parents[2],stdout=log,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW|subprocess.IDLE_PRIORITY_CLASS)
            identity=psutil.Process(process.pid)
            status(phase,child_pid=process.pid,child_created_epoch=identity.create_time(),child_command=command)
            result=process.wait()
        assert result==0,(phase,'child failed; preserve without retry',result)
    try:
        # The separate unique module controller owns admission and the one
        # small synthesis. This chain only observes it; it cannot restart it.
        status('waiting_for_existing_unique_module_SYN_result')
        while True:
            module_job=load(MODULE_CONTROL/'job.json')
            if module_job['phase']=='module_SYN_complete_DSP_removed':break
            assert module_job['phase'] in ('waiting_for_only_module_SYN_memory','running_only_isolated_module_SYN'),('module stage did not pass',module_job['phase'])
            assert alive(module_job['controller_pid'],module_job['controller_created_epoch']),'module controller exited without passed result'
            status('waiting_for_existing_unique_module_SYN_result',module_phase=module_job['phase'],module_controller_pid=module_job['controller_pid'],module_controller_created_epoch=module_job['controller_created_epoch'])
            time.sleep(13)
        module=load(CANDIDATE/'module_syn1/result.json')
        assert module['passed'] and module['target_action_DSP_removed_in_isolated_mapping'] and module['DSP_primitives']=={}
        assert module['runtime']['tool_exit']==0 and not module['runtime']['cancelled']
        assert not any(path.exists() for path in (PREPARED,SOURCE,OUTPUT,REVIEW)),'preserve existing candidate/build/review; no automatic repeat'
        status('preparing_exact_single_action_CSA_frozen_source')
        prep=prepare(PREPARED)
        assert prep['passed']
        wait_memory('waiting_for_unique_full_SYN27_memory')
        command=[sys.executable,'-X','utf8',str(Path(__file__).with_name('retry_frozen_synthesis.py')),
            str(PREPARED),'--out',str(SOURCE),'--start-now','--tool-priority','idle','--tool-cpu-core','18']
        child(command,'running_unique_full_SYN27','full_SYN27_stdout.log')
        check_source(PROFILE)
        wait_memory('waiting_for_new_actual_SYN27_review_memory')
        assert not REVIEW.exists()
        command=[sys.executable,'-X','utf8',str(Path(__file__).with_name('review_baseline_action_csa_syn.py')),
            '--source',str(SOURCE),'--prepared',str(PREPARED),'--out',str(REVIEW)]
        child(command,'reviewing_new_actual_SYN27','actual_review_stdout.log')
        check_review()
        assert not OUTPUT.exists()
        engine.prepare(PROFILE,REVIEW)
        wait_memory('waiting_for_unique_PNR37_memory')
        engine.validate_output(PROFILE,REVIEW);check_review()
        wait_memory('rechecking_memory_immediately_before_unique_PNR37')
        me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(initial_affinity)
        status('starting_unique_PNR37',PNR_priority=int(me.nice()),PNR_affinity=me.cpu_affinity())
        runtime=engine.run_tool([str(engine.TOOL),'run.tcl'],OUTPUT,max_runtime_s=0,max_no_progress_s=0,capture_system_telemetry=True)
        status('PNR37_exited_collecting',runtime=runtime)
        actual=OUTPUT/'impl/pnr/cmd.do'
        normalize=lambda text,root:text.replace(str(root),'<BUILD>').replace(root.as_posix(),'<BUILD>').split()
        options=actual.exists() and normalize(actual.read_text(),OUTPUT)==normalize((BASELINE/'impl/pnr/cmd.do').read_text(),BASELINE)
        engine.finalize_gate(OUTPUT);gate=load(OUTPUT/'g2_gate.json')
        physical=engine.physical_connections(PROFILE,status)
        passed=gate['routing_passed']
        status('PNR37_routed_requires_actual_timing_review' if passed else 'PNR37_physical_failure_preserved_no_retry',
            runtime=runtime,routing_passed=passed,actual_options_equal_to_PNR32=options,
            pure26_physical_connections_passed=physical,gate=str(OUTPUT/'g2_gate.json'))
        return int(runtime['cancelled'] or runtime['tool_exit']!=0 or not passed or not options or not physical)
    except Exception as error:
        import traceback
        status('failed_preserved_without_retry',error=str(error),error_type=type(error).__name__,traceback=traceback.format_exc())
        return 1


# Change only this new controller process's imported engine reference.
engine.ORIGIN=ORIGIN;engine.BASELINE=BASELINE;engine.check_source=check_source

if __name__=='__main__':raise SystemExit(main())
