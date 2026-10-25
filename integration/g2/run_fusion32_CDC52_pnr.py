"""One PNR35: routed PNR32 inputs plus 26 audited hold/max constraints.

No synthesis, RTL/CST/clock-period/route-option change or programming occurs.
The public Gowin flow has no documented fixed-route STA-only entry point.
Existing PNR32 files are read only; PNR35 owns a fresh permanent claim.
"""
from datetime import datetime
import json
from pathlib import Path
import shutil
import time
import traceback

import psutil

from audit_cdc_first_stages import audit, sdc_fragment
from bounded_tool import run_tool
from collect_pnr_result import finalize_gate
from retry_frozen_synthesis import memory_status


ROOT = Path('F:/nav_integration_g2_20261023')
SOURCE = ROOT/'baseline_fusion_only_pnr32'
OUTPUT = ROOT/'baseline_fusion32_CDC52_pnr35'
REVIEW = ROOT/'fusion32_CDC52_preflight1/review_result.json'
TOOL = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
GIB = 1024**3


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def save(path, value):
    temporary = path.with_name(path.name+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    temporary.replace(path)


def metadata(path):
    path = Path(path); stat = path.stat()
    return dict(path=str(path), bytes=stat.st_size, mtime_ns=stat.st_mtime_ns)


def same(first, second):
    first, second = Path(first), Path(second)
    if first.stat().st_size != second.stat().st_size:
        return False
    with first.open('rb') as a, second.open('rb') as b:
        while True:
            x, y = a.read(65536), b.read(65536)
            if x != y:
                return False
            if not x:
                return True


def expected_inputs():
    proof = load(REVIEW)
    assert proof['passed'] and proof['reviewed_paths'] == 26 and proof['actual_registers_audited'] == 78
    assert Path(proof['source_frozen_netlist']).resolve() == (SOURCE/'g2_top.vg').resolve()
    assert Path(proof['actual_physical_netlist']).resolve() == (SOURCE/'impl/pnr/g2_top.vo').resolve()
    assert proof['all_observed_12_negative_hold_are_audited_async_first_stage']
    assert proof['existing_setup_violated_endpoints'] == 3781
    assert proof['hold_only_false_paths'] == proof['explicit_max_delays'] == 26
    assert proof['second_stage_paths_not_excepted'] and proof['reset_and_pulse_width_paths_not_excepted']
    assert proof['same_domain_setup_constraints_relaxed'] is False
    for path, record in proof['metadata'].items():
        assert metadata(Path(path)) == record, ('reviewed input changed', path)
    assert load(SOURCE/'g2_gate.json')['routing_passed'] is True
    original = (SOURCE/'g2.sdc').read_bytes()
    assert original.decode('utf-8').splitlines() == [
        'create_clock -name clk50 -period 20 [get_ports {clk50}]',
        'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]']
    fragment = sdc_fragment(dict(passed=True, paths=proof['paths']))
    expected = original+b'\n'+fragment.encode('utf-8')
    assert expected == Path(proof['candidate_full_sdc']).read_bytes()
    active = [line.strip() for line in expected.decode().splitlines()
              if line.strip() and not line.lstrip().startswith('#')]
    assert len(active) == 54 and len(set(active)) == 54
    assert sum(line.startswith('set_false_path ') and line.endswith(' -hold') for line in active) == 26
    assert sum(line.startswith('set_max_delay ') for line in active) == 26
    tcl = (SOURCE/'run.tcl').read_bytes()
    assert b'set_option -route_option 0' in tcl
    assert b'inc_pnr' not in tcl and b'inc_place' not in tcl
    assert [line.strip() for line in tcl.decode().splitlines() if line.strip().startswith('run ')] == ['run pnr']
    moved = tcl.replace(SOURCE.as_posix().encode(), OUTPUT.as_posix().encode())
    moved = moved.replace(str(SOURCE).encode(), str(OUTPUT).encode())
    assert moved != tcl
    return proof, expected, moved


def prepare():
    proof, sdc, tcl = expected_inputs()
    OUTPUT.mkdir(exist_ok=False)
    for name in ('g2_top.vg', 'g2.cst'):
        shutil.copyfile(SOURCE/name, OUTPUT/name)
    (OUTPUT/'g2.sdc').write_bytes(sdc)
    (OUTPUT/'run.tcl').write_bytes(tcl)
    shutil.copyfile(REVIEW, OUTPUT/'source32_actual_26_CDC_review.json')
    save(OUTPUT/'cdc_reference_audit.json', dict(netlist=str(OUTPUT/'g2_top.vg'),
        passed=True, paths=proof['paths'], expected_paths=26, errors=[],
        scope='Actual source32 VG/VO/INIT/reset/CE and unique-first-fanout binding; recheck physical35 and all52 active constraints after PNR.'))
    save(OUTPUT/'manifest.json', dict(source=str(SOURCE), source_manifest=load(SOURCE/'manifest.json'),
        frozen_netlist=True, reused_completed_synthesis=True, no_synthesis=True,
        only_change='26 audited source-Q to async-first-D hold-only exceptions plus26 bounded max-delays; clocks20/10, RTL/CST/route0 unchanged',
        source_CDC_review=str(REVIEW), hold_only_false_paths=26, explicit_max_delays=26,
        max_runtime_minutes=0, max_no_progress_minutes=0,
        diagnostic_only=True, programmed=False, deployment_accepted=False,
        no_fixed_route_STA_API_found=True, incremental_PNR_not_enabled=True))
    validate()


def validate():
    proof, sdc, tcl = expected_inputs()
    for name in ('g2_top.vg', 'g2.cst'):
        assert same(SOURCE/name, OUTPUT/name), ('frozen input changed', name)
    assert (OUTPUT/'g2.sdc').read_bytes() == sdc
    assert (OUTPUT/'run.tcl').read_bytes() == tcl
    assert same(REVIEW, OUTPUT/'source32_actual_26_CDC_review.json')
    assert load(OUTPUT/'cdc_reference_audit.json')['paths'] == proof['paths']
    assert not (OUTPUT/'impl').exists() and not (OUTPUT/'tool_runtime.json').exists(), 'refuse repeated PNR'


def main():
    prepare()
    me = psutil.Process()
    state = dict(source=str(SOURCE), output=str(OUTPUT), controller_pid=me.pid,
        controller_created_epoch=me.create_time(), source_review=str(REVIEW),
        total_limit_s=0, silent_limit_s=0, synthesis_started=False,
        diagnostic_only=True, programmed=False, deployment_accepted=False)
    with (OUTPUT/'launch_claim.json').open('x', encoding='utf-8') as file:
        json.dump(state, file, ensure_ascii=False, indent=2)
    def status(phase, **values):
        state.update(phase=phase, updated_at=datetime.now().astimezone().isoformat(), **values)
        save(OUTPUT/'job.json', state)
    try:
        while True:
            memory = memory_status()
            status('waiting_for_memory_admission', available_physical_bytes=memory.avail_phys,
                available_commit_bytes=memory.avail_page, minimum_physical_bytes=2*GIB,
                minimum_commit_bytes=4*GIB)
            if memory.avail_phys >= 2*GIB and memory.avail_page >= 4*GIB:
                validate()
                # A fresh sample follows all input I/O. A transient reserve dip
                # waits here; it never repeats a started physical build.
                memory = memory_status()
                if memory.avail_phys >= 2*GIB and memory.avail_page >= 4*GIB:
                    break
            time.sleep(10)
        status('starting_unique_PNR35', launch_available_physical_bytes=memory.avail_phys,
               launch_available_commit_bytes=memory.avail_page)
        runtime = run_tool([str(TOOL), 'run.tcl'], OUTPUT, max_runtime_s=0,
            max_no_progress_s=0, capture_system_telemetry=True)
        status('tool_exited_collecting_actual_result', runtime=runtime)
        command = OUTPUT/'impl/pnr/cmd.do'
        if not command.exists():
            status('failed_before_PNR_command_report', actual_options_verified=False)
            return 1
        normalize = lambda text, root: text.replace(str(root), '<BUILD>').replace(root.as_posix(), '<BUILD>').split()
        equal = normalize(command.read_text(), OUTPUT) == normalize((SOURCE/'impl/pnr/cmd.do').read_text(), SOURCE)
        if (OUTPUT/'impl/pnr/g2_top.rpt.txt').exists():
            finalize_gate(OUTPUT)
        physical = OUTPUT/'impl/pnr/g2_top.vo'
        if physical.exists():
            actual = audit(physical)
            keys = ('group', 'source_register', 'first_register', 'second_register', 'source_clock', 'target_clock')
            sig = lambda rows: sorted(tuple(row[key] for key in keys) for row in rows)
            expected = load(OUTPUT/'cdc_reference_audit.json')
            actual['matches_audited26'] = sig(actual['paths']) == sig(expected['paths'])
            actual['full_CDC_timing_signoff'] = False
            save(OUTPUT/'physical26_connection_review.json', actual)
        status('completed_requires_actual_route_setup_hold52_constraint_review',
            runtime=runtime, actual_PNR_options_equal_to_PNR32=equal,
            actual_result_collected=(OUTPUT/'g2_gate.json').exists())
        return int(not equal or runtime['cancelled'] or runtime['tool_exit'] != 0)
    except Exception as error:
        status('failed_preserved_without_retry', error=str(error), traceback=traceback.format_exc())
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
