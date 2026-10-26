"""One real model25/NAV50 joint synthesis, parallel to production simulation.

This only measures the new hardware netlist. PNR and deployment require separate
actual clock, memory, related-boundary, reset and functional reviews.
"""
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import psutil

from prepare_baseline_fusion_retime import metadata, same_bytes
from retry_frozen_synthesis import memory_status
from run_baseline_single_factor_pnr import inputs

BASE = Path('F:/nav_integration_g2_20261023')
ORIGIN = BASE/'baseline_fusion_only_syn23'
PREP = BASE/'baseline_fusion_model25_NAV50_prepared4'
OUTPUT = BASE/'baseline_fusion_model25_NAV50_syn34'
CONTROL = BASE/'baseline_fusion_model25_NAV50_one_shot'
BUILD_LABEL = 'SYN34'
CHANGED = sorted(['g2_top.v', 'control4500_core.v', 'control/mem_policy.v',
                  'control/imu_fusion.v'])
ADDED = sorted(['control/mem_policy_related25.v', 'model25_related_mailbox.v',
                'model25_pll/model25_pll.v'])
GIB = 1024**3


def hardware(root):
    manifest = json.loads((root/'manifest.json').read_text())
    paths = [Path(p) for p in manifest['source_files']]
    result = {p.relative_to(root).as_posix(): p for p in paths}
    assert len(paths) == len(result), 'duplicate hardware input'
    return result


def source_check():
    proof = json.loads((PREP/'prepared_source_review.json').read_text())
    assert proof['prepared'] and sorted(proof['changed_original_RTL']) == CHANGED
    assert sorted(proof['added_RTL']) == ADDED
    for row in proof['bindings']:
        assert metadata(Path(row['path'])) == row, row['path']
    old, new = hardware(ORIGIN), hardware(PREP)
    assert len(old) == 63 and len(new) == 66
    assert sorted(set(new)-set(old)) == ADDED and not set(old)-set(new)
    assert sorted(n for n in old if not same_bytes(old[n], new[n])) == CHANGED
    counts = {}
    for suffixes, count in [({'.hex', '.mem', '.bin', '.mif'}, 69),
                            ({'.h', '.vh'}, 31)]:
        a, b = inputs(ORIGIN, suffixes), inputs(PREP, suffixes)
        assert a.keys() == b.keys() and len(a) == count
        assert all(same_bytes(a[n], b[n]) for n in a)
        counts['assets' if count == 69 else 'headers'] = count
    for directory in ('fw', 'firmware'):
        a = {p.relative_to(ORIGIN/directory): p for p in (ORIGIN/directory).rglob('*')
             if p.is_file() and '__pycache__' not in p.parts}
        b = {p.relative_to(PREP/directory): p for p in (PREP/directory).rglob('*')
             if p.is_file() and '__pycache__' not in p.parts}
        assert a.keys() == b.keys() and all(same_bytes(a[n], b[n]) for n in a)
    assert same_bytes(ORIGIN/'g2.cst', PREP/'g2.cst')
    manifest = json.loads((PREP/'manifest.json').read_text())
    assert manifest['actual_clock_plan_mhz'] == dict(
        NAV_CPU_peripherals=50, model_fusion_RL_action=25, radar_constraint=100)
    assert not manifest['control_weights_changed'] and not manifest['programmed']
    return dict(original_RTL=63, changed_RTL=CHANGED, added_RTL=ADDED,
                identical_original_RTL=59, identical_assets=counts['assets'],
                identical_headers=counts['headers'], original_FW_unchanged=True,
                bindings=[metadata(p) for p in new.values()])


def main():
    checked = source_check()
    assert not OUTPUT.exists()
    CONTROL.mkdir(exist_ok=False)
    me = psutil.Process()
    me.nice(psutil.NORMAL_PRIORITY_CLASS)
    me.cpu_affinity(list(range(psutil.cpu_count())))
    state = dict(controller_pid=me.pid, controller_created_epoch=me.create_time(),
                 source=str(PREP), output=str(OUTPUT), source_review=checked,
                 SYN_max_runtime_minutes=40, automatic_PNR=False,
                 production_simulation_exit_dependency=False, programmed=False,
                 actual_new_netlist_review_passed=False)
    with (CONTROL/'launch_claim.json').open('x') as file:
        json.dump(state, file, indent=2)

    def status(phase, **fields):
        state.update(phase=phase, updated_at=datetime.now().astimezone().isoformat(), **fields)
        tmp = CONTROL/(str(os.getpid())+'.tmp')
        tmp.write_text(json.dumps(state, indent=2))
        tmp.replace(CONTROL/'job.json')

    try:
        while True:
            m = memory_status()
            status('waiting_for_actual_memory_admission', free_bytes=m.avail_phys,
                   available_commit_bytes=m.avail_page,
                   minimum_physical_bytes=2*GIB, minimum_commit_bytes=4*GIB)
            if m.avail_phys >= 2*GIB and m.avail_page >= 4*GIB:
                break
            time.sleep(13)
        source_check()
        cmd = [sys.executable, '-X', 'utf8', str(Path(__file__).with_name(
            'retry_frozen_synthesis.py')), str(PREP), '--out', str(OUTPUT),
            '--start-now', '--tool-priority', 'normal']
        with (CONTROL/('full_'+BUILD_LABEL+'_stdout.log')).open('w') as log:
            child = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                     creationflags=subprocess.CREATE_NO_WINDOW)
            status('running_unique_full_'+BUILD_LABEL, runner_pid=child.pid,
                   runner_created_epoch=psutil.Process(child.pid).create_time(), command=cmd)
            code = child.wait()
        assert code == 0, ('full synthesis did not finish successfully', code)
        status(BUILD_LABEL+'_complete_actual_3clock_RAM_INIT_related_boundaries_review_pending',
               review_required=['all RAM geometry and INIT', 'CPU128KiB and original FW',
                                'actual 50/25/100MHz clock drivers and PLL parameters',
                                'original26 CDC plus five atomic related-clock channels',
                                'async fault latch and independent reset releases',
                                'actual globals, no stale CST aliases or timing exceptions'])
    except Exception as error:
        import traceback
        status('failed_preserved_no_automatic_retry', error=str(error), traceback=traceback.format_exc())
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
