"""Run one already-materialized NAV bus comparison; never write RTL or EDA inputs."""
from pathlib import Path
from datetime import datetime
import argparse
import json
import subprocess
import time
import traceback

import psutil

from prepare_baseline_fusion_retime import metadata
from prepare_ring_stride32_private_reader import natural_pair_complete
from retry_frozen_synthesis import memory_status


ROOT = Path(__file__).resolve().parents[2]
OUT = Path('F:/nav_integration_g2_20261023/NAV_command_predecode_bus_probe1')
CANDIDATE = Path('F:/nav_integration_g2_20261023/SLF128_NAV_predecode_prepared1')
VIV = Path('E:/2026.1/Vivado/bin')
G = 1024 ** 3


def main():
    global OUT
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, default=OUT)
    OUT = ap.parse_args().out.resolve()
    assert OUT.parent == CANDIDATE.parent and OUT != CANDIDATE
    plan = json.loads((OUT / 'run_plan.json').read_text(encoding='utf8'))
    assert not (OUT / 'run_claim.json').exists(), 'Existing run; no retry or overwrite'
    assert plan['phase'] == 'finite_bus_miter_inputs_prepared_not_run'
    natural_pair_complete()
    manifest = json.loads((CANDIDATE / 'manifest.json').read_text(encoding='utf8'))
    files = [Path(p) for p in manifest['source_files']]
    files += [OUT / n for n in ('nav_reference.v', 'nav_candidate.v',
                                'bus_wrappers.sv', 'tb_NAV_command_predecode.sv',
                                'sin_lut.hex', 'run_plan.json')]
    bindings = [metadata(p) for p in files]
    me = psutil.Process()
    me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    state = dict(controller_pid=me.pid, controller_created_epoch=me.create_time(),
                 parent_pid=me.ppid(), automatic_SYN=False, automatic_PNR=False,
                 input_RTL_modified=False, completed_tools=[])
    (OUT / 'run_claim.json').write_text(json.dumps(state, indent=2), encoding='utf8')

    def status(phase, **kw):
        state.update(phase=phase, updated_at=datetime.now().astimezone().isoformat(), **kw)
        temp = OUT / 'job.tmp'
        temp.write_text(json.dumps(state, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
        temp.replace(OUT / 'job.json')

    try:
        for index, step in enumerate(plan['commands_not_executed']):
            while True:
                mem = memory_status()
                status('waiting_actual_memory', next_tool=step['tool'],
                       physical=mem.avail_phys, commit=mem.avail_page)
                if mem.avail_phys >= 2 * G and mem.avail_page >= 4 * G:
                    break
                time.sleep(15)
            assert bindings == [metadata(p) for p in files], 'Frozen comparison input changed'
            cmd = ['cmd', '/c', str(VIV / (step['tool'] + '.bat')), *step['arguments']]
            logpath = OUT / f'{index}_{step["tool"]}.log'
            with logpath.open('w', encoding='utf8') as log:
                child = subprocess.Popen(cmd, cwd=OUT, stdout=log,
                                         stderr=subprocess.STDOUT,
                                         creationflags=subprocess.CREATE_NO_WINDOW)
                proc = psutil.Process(child.pid)
                runtime = dict(tool=step['tool'], wrapper_pid=proc.pid,
                               created_epoch=proc.create_time(), command=cmd,
                               started_at=datetime.now().astimezone().isoformat(),
                               admission_physical=mem.avail_phys, admission_commit=mem.avail_page)
                status('running_' + step['tool'], runtime=runtime)
                low = 0
                while child.poll() is None:
                    mem_now = memory_status()
                    danger = mem_now.avail_page < 512 * 1024**2 or (
                        mem_now.avail_phys < 256 * 1024**2 and mem_now.avail_page < G)
                    low = low + 1 if danger else 0
                    if low >= 15:
                        # Only this new simulation tool tree, never another EDA job.
                        for p in reversed(proc.children(recursive=True)):
                            try:
                                p.kill()
                            except psutil.NoSuchProcess:
                                pass
                        child.kill()
                        child.wait()
                        raise RuntimeError('Own probe stopped after sustained low memory; no retry')
                    time.sleep(2)
                runtime.update(exit_code=child.returncode,
                               ended_at=datetime.now().astimezone().isoformat())
            (OUT / f'{index}_{step["tool"]}_runtime.json').write_text(
                json.dumps(runtime, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
            assert child.returncode == 0, (step['tool'], child.returncode, str(logpath))
            state['completed_tools'].append(runtime)
        text = (OUT / '3_xsim.log').read_text(encoding='utf8', errors='replace')
        assert 'TEST PASSED NAV_COMMAND_BUS' in text and 'Fatal:' not in text, 'No valid comparison PASS'
        assert bindings == [metadata(p) for p in files]
        result = dict(passed=True, source=str(CANDIDATE), finite_bus_comparison=True,
                      raw_FIFO_MBX_console_concurrent_proof_complete=False,
                      numerical_uninitialized_memory_payloads_not_accepted=True,
                      inputs_unchanged=True, bindings=bindings,
                      actual_tools=state['completed_tools'], automatic_SYN=False,
                      automatic_PNR=False, physical_timing_unproved=True)
        (OUT / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
        status('finite_bus_comparison_completed_requires_review', passed=True)
        return 0
    except BaseException:
        status('failed_preserved_no_retry', passed=False, error=traceback.format_exc())
        raise


if __name__ == '__main__':
    raise SystemExit(main())
