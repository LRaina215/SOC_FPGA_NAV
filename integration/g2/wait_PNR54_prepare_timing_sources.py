"""One natural-completion handoff: materialize isolated sources, never EDA.

The frozen PNR53/54 inputs are read-only. Preparation is a single sequence
after their original tools exit. Each called preparer repeats the gate and
refuses an existing output. Validation and implementation remain separate.
"""
from pathlib import Path
from datetime import datetime
import json
import subprocess
import sys
import time

import psutil

from prepare_baseline_fusion_retime import metadata
from prepare_ring_stride32_private_reader import natural_pair_complete


R = Path(__file__).resolve().parents[2]
B = Path('F:/nav_integration_g2_20261023')
OUT = B / 'PNR54_timing_source_handoff1'
PY = 'D:/Anaconda/python.exe'
TOOLS = R / 'integration/g2'
RING = B / 'SLF128_ring_stride32_prepared1'
NAV = B / 'SLF128_NAV_predecode_prepared1'


def main():
    assert not OUT.exists(), 'Existing handoff: no duplicate or retry'
    targets = [RING, NAV, B / 'ring_stride32_real4500_probe1',
               B / 'NAV_command_predecode_bus_probe1',
               B / 'FW_sin_ROM_RTL_overlay_prepared1',
               B / 'PID_unity_bypass_RTL_overlay_prepared1']
    assert not any(p.exists() for p in targets), 'One or more source outputs already exist'
    steps = [
        ('ring_source', 'prepare_ring_stride32_private_reader.py',
         ['--materialize', '--candidate', str(RING), '--authorization',
          'materialize-only-after-natural-PNR53-PNR54-completion',
          '--receipt', str(OUT / 'ring_source.json')]),
        ('NAV_source', 'prepare_nav_command_predecode.py',
         ['--materialize', '--candidate', str(NAV), '--authorization',
          'materialize-nav-predecode-after-natural-PNR53-PNR54-completion',
          '--receipt', str(OUT / 'NAV_source.json')]),
        ('ring_validation_inputs', 'prepare_ring_stride32_real4500_probe.py',
         ['--materialize', '--candidate', str(RING), '--out', str(targets[2]),
          '--authorization', 'freeze-only-after-natural-PNR53-PNR54-completion']),
        ('NAV_validation_inputs', 'prepare_nav_command_predecode.py',
         ['--prepare-bus-miter', '--candidate', str(NAV),
          '--validation-out', str(targets[3]), '--authorization',
          'prepare-bus-miter-only-no-tools',
          '--receipt', str(OUT / 'NAV_validation_inputs.json')]),
        ('FW_ROM_overlay', 'prepare_FW_sin_ROM_RTL_overlay.py',
         ['--materialize', '--out', str(targets[4])]),
        ('PID_unity_overlay', 'plan_PNR53_PID_unity_bypass.py',
         ['--materialize', '--out', str(targets[5])]),
    ]
    entries = [TOOLS / step[1] for step in steps]
    dependencies = entries + [TOOLS / 'prepare_baseline_fusion_retime.py']
    bindings = [metadata(p) for p in dependencies]
    OUT.mkdir()
    me = psutil.Process()
    me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    state = dict(controller_pid=me.pid, controller_created_epoch=me.create_time(),
                 parent_pid=me.ppid(), EDA_started=False, automatic_SYN=False,
                 automatic_PNR=False, programmed=False, source_outputs_selected=False,
                 targets=[str(p) for p in targets], completed=[])

    def save(phase, **kw):
        state.update(phase=phase, updated_at=datetime.now().astimezone().isoformat(), **kw)
        tmp = OUT / 'job.tmp'
        tmp.write_text(json.dumps(state, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
        tmp.replace(OUT / 'job.json')

    (OUT / 'launch_claim.json').write_text(json.dumps(state, indent=2), encoding='utf8')
    try:
        # A live original process, even after the log reaches its last stage,
        # is not permission to write new RTL.
        while True:
            ready = True
            rows = []
            for name in ('SLF128_storage_place1_pnr53', 'SLF128_storage_place2_pnr54'):
                rt = json.loads((B / name / 'tool_runtime.json').read_text(encoding='utf8'))
                assert rt['cancelled'] is False
                assert rt['max_runtime_s'] == rt['max_no_observable_progress_s'] == 0
                start = datetime.fromisoformat(rt['started_at']).timestamp()
                live = False
                try:
                    p = psutil.Process(rt['pid'])
                    live = abs(p.create_time() - start) < 2
                except psutil.NoSuchProcess:
                    pass
                final = bool(rt.get('ended_at')) and rt.get('tool_exit') is not None
                ready = ready and final and not live
                rows.append(dict(build=name, original_pid=rt['pid'], live=live,
                                 final=final, tool_exit=rt.get('tool_exit')))
            save('waiting_original_PNR53_PNR54_natural_completion', original_tools=rows)
            if ready:
                break
            time.sleep(17)
        pair = natural_pair_complete()
        assert bindings == [metadata(p) for p in dependencies], 'Preparer changed during wait'
        save('materializing_isolated_sources_no_EDA', natural_pair=pair)
        for label, entry, args in steps:
            command = [PY, '-B', '-X', 'utf8', str(TOOLS / entry), *args]
            save('materializing_isolated_sources_no_EDA', current=label, command=command)
            with (OUT / (label + '.log')).open('w', encoding='utf8') as log:
                child = subprocess.Popen(command, cwd=R, stdout=log,
                                         stderr=subprocess.STDOUT,
                                         creationflags=subprocess.CREATE_NO_WINDOW)
                code = child.wait()
            assert code == 0, (label, code, str(OUT / (label + '.log')))
            state['completed'].append(label)
        save('source_and_validation_inputs_ready_requires_root_review',
             all_six_preparations_completed=True, automatically_validated=False)
        return 0
    except BaseException as exc:
        save('failed_preserved_no_retry', error=f'{type(exc).__name__}: {exc}')
        raise


if __name__ == '__main__':
    raise SystemExit(main())
