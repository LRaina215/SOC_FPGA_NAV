"""One reviewed SLF128/storage/safety synthesis; never automatically starts PNR.

The final combined candidate remains immutable. A separate execution input is
made only after the root source review has bound the combined RTL regression
and the newly linked firmware/128KiB initialization. Historical preparation
directories intentionally retain their disabled run.tcl files.
"""
from pathlib import Path
from datetime import datetime
import argparse
import json
import re
import shutil
import subprocess
import sys
import time
import traceback
import psutil

from prepare_baseline_fusion_retime import hardware_inputs, load, metadata, relocate, same_bytes, save
from retry_frozen_synthesis import memory_status

BASE = Path('F:/nav_integration_g2_20261023')
REFERENCE = BASE/'baseline_fusion50_ring_MAX_prepared1'
PREP = BASE/'SLF128_storage_safeFW_SYN41_prepared1'
OUTPUT = BASE/'SLF128_storage_safeFW_syn41'
CONTROL = BASE/'SLF128_storage_safeFW_SYN41_one_shot'
GIB = 1024**3


def checked_review(path):
    review = load(path)
    assert review['passed'] and review['combined_RTL_contracts_passed']
    assert review['new_ELF_BIN_128KiB_INIT_bound']
    assert review['production_SLF_LOG2N'] == 7
    assert review['CPU_RAM_bytes'] == 131072
    assert review['joint_SYN_authorized'] and not review['PNR_authorized']
    for row in review['bindings']:
        assert metadata(row['path']) == row, ('Frozen reviewed input changed', row)
    return review


def prepare(review_path):
    review_path = review_path.resolve()
    review = checked_review(review_path)
    source = Path(review['candidate']).resolve()
    manifest = load(source/'manifest.json')
    hardware = hardware_inputs(source, manifest)
    assert len(hardware) == 63
    assert manifest['SLF_LOG2N'] == 7 and not manifest['control_weights_changed']
    assert not manifest.get('real_sfp') and not manifest.get('real_ddr')
    assert not PREP.exists() and not OUTPUT.exists() and not CONTROL.exists()
    live = [p.info for p in psutil.process_iter(['pid', 'name'])
            if (p.info['name'] or '').lower() in
            {'xvlog.exe', 'xelab.exe', 'xsim.exe', 'xsimk.exe', 'vsim.exe', 'vsimk.exe', 'vlog.exe', 'vopt.exe'}]
    assert not live, ('Freeze RTL/TB before EDA', live)
    for name in ('g2.cst', 'g2.sdc', 'control4500_core.v', 'command_lease_guard.v',
                 'control/imu_fusion.v', 'control/rl_observation_v5.v',
                 'control/rl_policy_tick_v5.v', 'control/ir_remote_control.v',
                 'control/oled.v', 'control/oled_driver.v', 'control/oled_front.v',
                 'nav/rtl/common/nav_fifo_ob.v'):
        assert same_bytes(source/name, REFERENCE/name), ('Preserved function changed', name)
    tcl = (REFERENCE/'run.tcl').read_text(encoding='utf8')
    assert [s.strip() for s in tcl.splitlines() if s.strip().startswith('run ')] == ['run syn']
    tcl = relocate(tcl, REFERENCE, PREP)
    listed = re.findall(r'^add_file -type verilog \{(.+)\}$', tcl, re.M)
    assert {Path(x).relative_to(PREP).as_posix() for x in listed} == set(hardware)
    shutil.copytree(source, PREP, ignore=shutil.ignore_patterns('impl', '__pycache__', '*.log', 'job.json', 'tool_runtime.json'))
    m = relocate(manifest, source, PREP)
    m.update(stage='syn', source_preparation_only=False, automatic_pnr=False,
             automatic_PNR=False, automatic_SYN=False, synthesis_complete=False,
             programmed=False, deployment_accepted=False, firmware_rebuild_pending=False,
             inherited_CPU_INIT_not_new_FW=False, new_firmware_RV_ELF_BIN_128KiB_lanes_bound=True,
             root_source_review=str(review_path), frozen_source_build=str(source),
             modified_rtl_inputs=review['changed_15_RTL'])
    m.pop('effective_options', None)
    save(PREP/'manifest.json', m)
    (PREP/'run.tcl').write_text(tcl, encoding='utf8', newline='\n')
    for rel, path in hardware.items():
        assert same_bytes(path, PREP/rel), rel
    bindings = []
    for p in PREP.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:
            bindings.append(metadata(p))
    result = dict(passed=True, candidate=str(source), root_review=str(review_path),
                  execution_source=str(PREP), output=str(OUTPUT),
                  same_63_RTL=True, original_SYN40_options_device_CST_SDC=True,
                  root_review_binding=metadata(review_path), bindings=bindings,
                  automatic_PNR=False, synthesis_started=False)
    save(PREP/'execution_source_review.json', result)
    return result


def frozen():
    r = load(PREP/'execution_source_review.json')
    checked_review(Path(r['root_review']))
    assert metadata(r['root_review']) == r['root_review_binding']
    for row in r['bindings']:
        assert metadata(row['path']) == row, row
    return r


def start():
    frozen()
    assert not OUTPUT.exists()
    CONTROL.mkdir(exist_ok=False)
    me = psutil.Process()
    me.nice(psutil.NORMAL_PRIORITY_CLASS)
    me.cpu_affinity(list(range(psutil.cpu_count())))
    state = dict(controller_pid=me.pid, controller_created_epoch=me.create_time(),
                 controller_parent_pid=me.ppid(), source=str(PREP), output=str(OUTPUT),
                 automatic_PNR=False, programmed=False, build_label='SYN41',
                 minimum_physical_bytes=2*GIB, minimum_commit_bytes=4*GIB,
                 original_runtime_memory_protection=True, SYN_max_runtime_minutes=40,
                 no_other_PNR_exit_dependency=True)
    save(CONTROL/'launch_claim.json', state)

    def status(phase, **kw):
        state.update(phase=phase, updated_at=datetime.now().astimezone().isoformat(), **kw)
        tmp = CONTROL/'job.tmp'
        save(tmp, state)
        tmp.replace(CONTROL/'job.json')

    try:
        while True:
            mem = memory_status()
            status('waiting_for_actual_memory_admission', free_bytes=mem.avail_phys,
                   available_commit_bytes=mem.avail_page)
            if mem.avail_phys >= 2*GIB and mem.avail_page >= 4*GIB:
                break
            time.sleep(13)
        frozen()
        cmd = [sys.executable, '-B', '-X', 'utf8',
               str(Path(__file__).with_name('retry_frozen_synthesis.py')),
               str(PREP), '--out', str(OUTPUT), '--start-now', '--tool-priority', 'normal']
        with (CONTROL/'full_SYN41_stdout.log').open('w', encoding='utf8') as log:
            child = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                     creationflags=subprocess.CREATE_NO_WINDOW)
            status('running_unique_joint_SYN41', runner_pid=child.pid,
                   runner_created_epoch=psutil.Process(child.pid).create_time(), command=cmd,
                   admission_physical_bytes=mem.avail_phys, admission_commit_bytes=mem.avail_page)
            code = child.wait()
        assert code == 0, ('Read actual failure; no automatic repeat', code)
        frozen()
        status('SYN41_complete_actual_mapping_review_pending',
               actual_mapping_acceptance=False, physical_resources_measured=False)
        return 0
    except Exception as exc:
        status('failed_preserved_no_automatic_retry', error=str(exc), traceback=traceback.format_exc())
        return 1


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepare', type=Path, metavar='ROOT_REVIEW_JSON')
    p.add_argument('--start', action='store_true')
    a = p.parse_args()
    assert bool(a.prepare) != a.start
    if a.prepare:
        print(json.dumps(prepare(a.prepare), ensure_ascii=False))
    else:
        raise SystemExit(start())
