"""Single-factor incremental PNR48 using routed35's vendor .pr checkpoint.

SUG100-4.4.7E, pp42-43, supports incremental placement+routing on Arora V
and selects a .pr checkpoint. Reuse complete SYN33; do not regenerate RTL,
relax timing, add global requests or modify the original routed checkpoint.
The preparation alone is not a physical attempt or proof of checkpoint reuse.
"""
from pathlib import Path
from datetime import datetime
import argparse
import json
import shutil
import time

import psutil
import run_action_dsp_input_register_pnr as variant

run = variant.run
BASE = run.BASE
run.OUTPUT = BASE/'baseline_fusion_DSP_AREG_incremental35_pnr48'
run.CONTROL = BASE/'baseline_fusion_DSP_AREG_incremental35_one_shot'
run.PNR_LABEL = 'PNR48'
run.PROFILE = dict(source=run.SOURCE, prepared=run.PREPARED, output=run.OUTPUT, interval=13)
CHECKPOINT = run.REFERENCE/'impl/pnr/g2_top.pr'
COPIED_CHECKPOINT = run.OUTPUT/'routed35.pr'
original_source_check = run.source_check


def source_check():
    actual, sdc, tcl = original_source_check()
    assert CHECKPOINT.is_file() and CHECKPOINT.stat().st_size == 238642736
    assert run.load(run.REFERENCE/'g2_gate.json')['routing_passed']
    assert tcl.count(b'run pnr') == 1 and b'inc_pnr' not in tcl
    command = ('set_option -inc_pnr {'+COPIED_CHECKPOINT.as_posix()+'}\n').encode()
    tcl = tcl.replace(b'run pnr', command+b'run pnr', 1)
    return actual, sdc, tcl


run.source_check = source_check


def prepare():
    run.prepare()
    shutil.copyfile(CHECKPOINT, COPIED_CHECKPOINT)
    assert run.same(CHECKPOINT, COPIED_CHECKPOINT)
    proof = run.load(run.OUTPUT/'prepared_request_review.json')
    proof['bindings'].append(run.meta(CHECKPOINT))
    proof['prepared_bindings'].append(run.meta(COPIED_CHECKPOINT))
    proof.update(incremental_checkpoint=str(COPIED_CHECKPOINT),
                 original_routed_checkpoint=str(CHECKPOINT),
                 only_physical_factor_vs_PNR47='set_option -inc_pnr <routed35.pr>',
                 documented_support='SUG100-4.4.7E pp42-43: Arora V / Arora III, placement+routing .pr',
                 actual_checkpoint_reuse_not_yet_proven=True)
    run.save(run.OUTPUT/'prepared_request_review.json', proof)
    manifest = run.load(run.OUTPUT/'manifest.json')
    manifest.update(incremental_pnr_checkpoint=str(COPIED_CHECKPOINT),
                    reference_checkpoint=str(CHECKPOINT),
                    only_physical_factor_vs_PNR47='incremental placement and routing from routed35',
                    no_new_synthesis_RTL_INIT_CST_SDC_clock_global_or_timing_exception=True)
    run.save(run.OUTPUT/'manifest.json', manifest)
    run.validate()


def main(start):
    if not run.OUTPUT.exists():
        prepare()
    run.validate()
    if not start:
        print(json.dumps(dict(prepared=True, actual_PNR_started=False, output=str(run.OUTPUT))))
        return 0
    run.CONTROL.mkdir(exist_ok=False)
    me = psutil.Process();me.nice(psutil.NORMAL_PRIORITY_CLASS)
    me.cpu_affinity(list(range(psutil.cpu_count())))
    state = dict(controller_pid=me.pid, controller_created_epoch=me.create_time(),
                 source=str(run.SOURCE), output=str(run.OUTPUT), checkpoint=str(COPIED_CHECKPOINT),
                 total_limit_s=0, silent_limit_s=0, programmed=False, deployment_accepted=False)
    with (run.CONTROL/'launch_claim.json').open('x') as f:
        json.dump(state, f)

    def status(phase, **fields):
        state.update(phase=phase, updated_at=datetime.now().astimezone().isoformat(), **fields)
        run.save(run.CONTROL/'job.json', state)

    def memory():
        while True:
            m = run.memory_status()
            status('waiting_for_memory_admission', free_bytes=m.avail_phys,
                   available_commit_bytes=m.avail_page,
                   minimum_physical_bytes=2*run.GIB, minimum_commit_bytes=4*run.GIB)
            if m.avail_phys >= 2*run.GIB and m.avail_page >= 4*run.GIB:
                return
            time.sleep(13)
    try:
        memory();run.validate();memory();status('starting_unique_PNR48')
        runtime = run.run_tool([str(run.engine.TOOL), 'run.tcl'], run.OUTPUT,
                               max_runtime_s=0, max_no_progress_s=0,
                               capture_system_telemetry=True)
        status('tool_exited_collecting', runtime=runtime)
        memory();run.finalize_gate(run.OUTPUT)
        gate = run.load(run.OUTPUT/'g2_gate.json')
        tr = run.OUTPUT/'impl/pnr/g2_top.tr'
        if tr.exists():
            run.save(run.OUTPUT/'actual_setup_endpoint_coverage.json', run.scan_report(tr))
        status('routed_requires_independent_checkpoint_resource_and_timing_review' if gate['routing_passed']
               else 'physical_failure_preserved_no_retry', runtime=runtime,
               routing_passed=gate['routing_passed'], unrouted=gate['unrouted_nets'],
               TR_available=tr.exists(), actual_incremental_reuse_requires_tool_log_review=True,
               actual_device_cfg_same=run.same(run.OUTPUT/'impl/pnr/device.cfg',
                                               run.REFERENCE/'impl/pnr/device.cfg'))
        return int(runtime['tool_exit'] != 0 or not gate['routing_passed'])
    except Exception as error:
        import traceback
        status('failed_preserved_no_automatic_retry', error=str(error), traceback=traceback.format_exc())
        return 1


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--start', action='store_true')
    raise SystemExit(main(p.parse_args().start))
