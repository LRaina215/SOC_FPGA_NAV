"""Review the existing unique SYN26, then run one cycle-exact fusion PNR36.

Never starts synthesis. The input netlist must be a completed SYN26 with the
new actual RAM/INIT/two-clock/26-CDC review. PNR32's original route0 script,
CST and20/10ns SDC are preserved, with no added timing exceptions.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from datetime import datetime

import psutil
import run_baseline_single_factor_pnr as acceptance

from bounded_tool import run_tool
from collect_pnr_result import finalize_gate
from retry_frozen_synthesis import memory_status
from run_baseline_single_factor_pnr import (
    check_review, equal_bytes, hardware, inputs, load_synthesis_job,
    metadata, physical_connections,
)

ROOT=Path('F:/nav_integration_g2_20261023')
SOURCE=ROOT/'baseline_fusion_square2_syn26'
PREPARED=ROOT/'baseline_fusion_square2_prepared1'
ORIGIN=ROOT/'baseline_fusion_only_syn23'
REFERENCE=ROOT/'baseline_fusion_only_pnr32'
acceptance.BASELINE=REFERENCE
OUTPUT=ROOT/'baseline_fusion_square2_pnr36'
CONTROL=ROOT/'baseline_fusion_square2_one_shot'
REVIEW=CONTROL/'actual_SYN26_netlist_review.json'
TOOL=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
PROFILE=dict(source=SOURCE,prepared=PREPARED,output=OUTPUT,changed='control/imu_fusion.v',interval=13)
GIB=1024**3


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def save(path, data):
    path=Path(path);tmp=path.with_name(path.name+'.'+str(os.getpid())+'.tmp')
    tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    tmp.replace(path)


def check_source():
    job=load_synthesis_job(SOURCE);manifest=load(SOURCE/'manifest.json')
    assert job['phase']=='synthesis_complete' and job['tool_exit']==0
    assert Path(job['source']).resolve()==PREPARED.resolve()
    assert manifest['fusion_square_pipeline2'] and not manifest.get('pose_predecode') and not manifest.get('action_map_exact_widths')
    assert manifest['clock_core_mhz']==50 and manifest['wide_split']==1
    assert not manifest['control_weights_changed'] and not manifest['programmed']
    assert Path(manifest['frozen_source_build']).resolve()==PREPARED.resolve()
    prep=load(PREPARED/'prepared_source_review.json')
    assert prep['passed'] and Path(prep['source']).resolve()==ORIGIN.resolve()
    assert prep['modified_rtl_inputs']==['control/imu_fusion.v']
    assert (prep['unchanged_hardware_inputs'],prep['identical_assets'],prep['identical_headers'])==(62,69,31)
    for row in prep['bindings']:
        assert metadata(Path(row['path']))==row, ('prepared proof binding changed',row['path'])
    original,prepared,actual=[hardware(p) for p in (ORIGIN,PREPARED,SOURCE)]
    assert list(original)==list(prepared)==list(actual)
    assert [n for n in original if not equal_bytes(original[n],prepared[n])]==['control/imu_fusion.v']
    assert all(equal_bytes(prepared[n],actual[n]) for n in prepared)
    for suffixes,count in [({'.hex','.mem','.bin','.mif'},69),({'.h','.vh'},31)]:
        old,new,real=[inputs(p,suffixes) for p in (ORIGIN,PREPARED,SOURCE)]
        assert old.keys()==new.keys()==real.keys() and len(old)==count
        assert all(equal_bytes(old[n],new[n]) and equal_bytes(old[n],real[n]) for n in old)
    for directory in ('fw','firmware'):
        for path in (ORIGIN/directory).rglob('*'):
            if not path.is_file() or '__pycache__' in path.parts:continue
            relative=path.relative_to(ORIGIN)
            assert equal_bytes(path,PREPARED/relative) and equal_bytes(path,SOURCE/relative)
    assert equal_bytes(ORIGIN/'impl/gwsynthesis/g2_top.vg',REFERENCE/'g2_top.vg')
    for name in ('g2.cst','g2.sdc'):
        assert all(equal_bytes(REFERENCE/name,p/name) for p in (ORIGIN,PREPARED,SOURCE))
    assert (SOURCE/'g2.sdc').read_text().splitlines()==[
        'create_clock -name clk50 -period 20 [get_ports {clk50}]',
        'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]']
    return manifest


def reference_tcl():
    original=(REFERENCE/'run.tcl').read_bytes()
    assert b'set_option -route_option 0' in original
    assert [s.strip() for s in original.decode().splitlines() if s.strip().startswith('run ')]==['run pnr']
    relocated=original.replace(REFERENCE.as_posix().encode(),OUTPUT.as_posix().encode()).replace(str(REFERENCE).encode(),str(OUTPUT).encode())
    assert relocated.replace(OUTPUT.as_posix().encode(),REFERENCE.as_posix().encode()).replace(str(OUTPUT).encode(),str(REFERENCE).encode())==original
    return relocated


def prepare():
    review=check_review(PROFILE,REVIEW);manifest=check_source()
    assert not OUTPUT.exists(), 'physical directory already exists; never repeat'
    OUTPUT.mkdir()
    for origin,name in [(SOURCE/'impl/gwsynthesis/g2_top.vg','g2_top.vg'),(REFERENCE/'g2.cst','g2.cst'),(REFERENCE/'g2.sdc','g2.sdc')]:
        shutil.copyfile(origin,OUTPUT/name)
    (OUTPUT/'run.tcl').write_bytes(reference_tcl())
    shutil.copyfile(REVIEW,OUTPUT/'actual_SYN_preflight_review.json')
    save(OUTPUT/'pure_CDC_connection_reference.json',dict(source_review=str(REVIEW),synthesis_connection_preflight_passed=True,paths=review['paths'],added_timing_exceptions=0))
    save(OUTPUT/'manifest.json',dict(source=str(SOURCE),prepared=str(PREPARED),source_manifest=manifest,
        frozen_netlist=True,reused_completed_synthesis=True,pure_RTL_single_factor='control/imu_fusion.v',
        option_reference=str(REFERENCE),actual_SYN_review=str(REVIEW),original_SDC_bytes_preserved=True,
        added_timing_exceptions=0,max_runtime_minutes=0,max_no_progress_minutes=0,
        reference_input_metadata=[metadata(REFERENCE/n) for n in ('run.tcl','g2.cst','g2.sdc','impl/pnr/cmd.do')],
        diagnostic_only=True,automatic_programming=False,programmed=False,deployment_accepted=False,
        scope='Cycle-preserving fusion implementation experiment; no physical or vehicle signoff.'))


def validate_output():
    review=check_review(PROFILE,REVIEW);check_source()
    assert load(OUTPUT/'actual_SYN_preflight_review.json')==review
    for origin,name in [(SOURCE/'impl/gwsynthesis/g2_top.vg','g2_top.vg'),(REFERENCE/'g2.cst','g2.cst'),(REFERENCE/'g2.sdc','g2.sdc')]:
        assert equal_bytes(origin,OUTPUT/name),('PNR input changed',name)
    assert (OUTPUT/'run.tcl').read_bytes()==reference_tcl()
    manifest=load(OUTPUT/'manifest.json')
    assert manifest['reference_input_metadata']==[metadata(REFERENCE/n) for n in ('run.tcl','g2.cst','g2.sdc','impl/pnr/cmd.do')]
    assert manifest['max_runtime_minutes']==manifest['max_no_progress_minutes']==manifest['added_timing_exceptions']==0
    assert not (OUTPUT/'cdc_reference_audit.json').exists()
    assert not (OUTPUT/'tool_runtime.json').exists() and not (OUTPUT/'impl').exists()


def main():
    CONTROL.mkdir(exist_ok=True);me=psutil.Process();initial_affinity=me.cpu_affinity()
    me.nice(psutil.IDLE_PRIORITY_CLASS);me.cpu_affinity([16])
    state=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),source=str(SOURCE),prepared=str(PREPARED),output=str(OUTPUT),source_review=str(REVIEW),total_limit_s=0,silent_limit_s=0,programmed=False,deployment_accepted=False)
    with (CONTROL/'chain_claim.json').open('x',encoding='utf-8') as file:json.dump(state,file,indent=2)
    checker_bindings=[];snapshots=CONTROL/'checker_snapshot';snapshots.mkdir(exist_ok=False)
    for name in ('run_fusion_square2_chain.py','review_fusion_square2_syn.py','review_baseline_single_factor_syn.py','run_baseline_single_factor_pnr.py'):
        original=Path(__file__).with_name(name);copy=snapshots/name
        shutil.copyfile(original,copy);assert equal_bytes(original,copy)
        checker_bindings.extend((metadata(original),metadata(copy)))
    state['checker_code_bindings']=checker_bindings
    def status(phase,**extra):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**extra)
        save(CONTROL/'job.json',state)
    def wait_memory(phase):
        while True:
            memory=memory_status()
            status(phase,free_bytes=memory.avail_phys,available_commit_bytes=memory.avail_page,minimum_physical_bytes=2*GIB,minimum_commit_bytes=4*GIB)
            if memory.avail_phys>=2*GIB and memory.avail_page>=4*GIB:return
            time.sleep(13)
    try:
        launch=load(CONTROL/'launch.json')
        assert Path(launch['synthesis']).resolve()==SOURCE.resolve()
        status('waiting_for_existing_SYN26')
        while True:
            job=load_synthesis_job(SOURCE)
            if job['phase']=='synthesis_complete':break
            assert job['phase'] in ('validating_frozen_settings','waiting_for_memory','synthesizing'),('existing SYN cannot continue',job['phase'])
            try:
                process=psutil.Process(launch['controller_pid'])
                assert abs(process.create_time()-launch['controller_created_epoch'])<.01
                assert not process.status()==psutil.STATUS_ZOMBIE
            except psutil.NoSuchProcess:
                raise AssertionError('SYN controller exited without complete result; preserve, do not restart')
            time.sleep(10)
        check_source();wait_memory('waiting_for_new_actual_SYN_review_memory')
        assert all(metadata(Path(row['path']))==row for row in checker_bindings), 'checker code changed; preserve existing SYN for explicit review continuation'
        assert not REVIEW.exists(),'preserve existing review; explicit targeted continuation required'
        status('reviewing_new_actual_SYN26_netlist')
        command=[sys.executable,'-X','utf8',str(Path(__file__).with_name('review_fusion_square2_syn.py')),'--source',str(SOURCE),'--prepared',str(PREPARED),'--out',str(REVIEW)]
        with (CONTROL/'actual_SYN_review_stdout.log').open('w',encoding='utf-8') as log:
            review_process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,cwd=Path(__file__).parents[2],creationflags=subprocess.CREATE_NO_WINDOW|subprocess.IDLE_PRIORITY_CLASS)
            review_identity=psutil.Process(review_process.pid)
            status('reviewing_new_actual_SYN26_netlist',review_pid=review_process.pid,review_created_epoch=review_identity.create_time(),review_command=command)
            code=review_process.wait()
        assert code==0,('new actual netlist review failed',code,str(REVIEW))
        assert all(metadata(Path(row['path']))==row for row in checker_bindings), 'checker changed during actual review'
        check_review(PROFILE,REVIEW);prepare();wait_memory('waiting_for_unique_PNR36_memory')
        validate_output();wait_memory('rechecking_memory_immediately_before_unique_PNR36')
        # Low-priority single-core review must not accidentally throttle PNR.
        # Restore only this controller; no existing process is touched.
        me.nice(psutil.NORMAL_PRIORITY_CLASS);me.cpu_affinity(initial_affinity)
        status('starting_unique_PNR36',PNR_priority=me.nice(),PNR_affinity=me.cpu_affinity())
        runtime=run_tool([str(TOOL),'run.tcl'],OUTPUT,max_runtime_s=0,max_no_progress_s=0,capture_system_telemetry=True)
        status('PNR36_tool_exited_collecting',runtime=runtime)
        command=OUTPUT/'impl/pnr/cmd.do'
        normalize=lambda text,root:text.replace(str(root),'<BUILD>').replace(root.as_posix(),'<BUILD>').split()
        same_options=command.exists() and normalize(command.read_text(),OUTPUT)==normalize((REFERENCE/'impl/pnr/cmd.do').read_text(),REFERENCE)
        finalize_gate(OUTPUT)
        gate=load(OUTPUT/'g2_gate.json')
        physical=physical_connections(PROFILE,status)
        routing_passed=gate['routing_passed']
        status('PNR36_completed_requires_actual_timing_review' if routing_passed else 'PNR36_physical_failure_preserved_no_retry',runtime=runtime,actual_options_equal_to_PNR32=same_options,pure26_physical_connections_passed=physical,routing_passed=routing_passed,gate=str(OUTPUT/'g2_gate.json'))
        return int(runtime['cancelled'] or runtime['tool_exit']!=0 or not same_options or not physical or not routing_passed)
    except Exception as error:
        import traceback
        status('failed_preserved_without_retry',error=str(error),error_type=type(error).__name__,traceback=traceback.format_exc())
        return 1


if __name__=='__main__':raise SystemExit(main())
