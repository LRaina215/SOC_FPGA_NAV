"""Two distinct place options of one admitted SLF128 netlist, without deadlines.

Root supplies the completed actual mapping/resource/clock/CDC admission. This
entry cannot turn synthesis estimates into deployment or program a board.
"""
from pathlib import Path
from datetime import datetime
import argparse
import json
import re
import shutil
import time
import traceback
import psutil

from audit_cdc_first_stages import sdc_fragment
from bounded_tool import run_tool
from check_setup_endpoint_coverage import scan_report
from collect_pnr_result import finalize_gate
from preflight import validate
from prepare_baseline_fusion_retime import load, metadata, relocate, same_bytes, save
from retry_frozen_synthesis import memory_status

BASE = Path('F:/nav_integration_g2_20261023')
REFERENCE = BASE/'baseline_fusion32_CDC52_pnr35'
TOOL = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
GIB = 1024**3
REPORTS = (
    'report_timing -setup -max_paths 5000 -max_common_paths 1',
    'report_timing -hold -max_paths 5000 -max_common_paths 1',
    'report_timing -recovery -max_paths 5000 -max_common_paths 1',
    'report_timing -removal -max_paths 5000 -max_common_paths 1',
    'report_min_pulse_width -nworst 5000 -detail',
    'report_route_congestion -max_grids 100',
    'report_high_fanout_nets -max_nets 100 -slr',
)


def paths(place):
    assert place in (1, 2)
    number = 52+place
    return (BASE/f'SLF128_storage_place{place}_pnr{number}',
            BASE/f'SLF128_storage_place{place}_pnr{number}_one_shot')


def checked(path):
    root = load(path)
    assert root['passed'] and root['SYN_resource_gate_passed']
    assert root['actual_storage_INIT_and_read_gate_legal']
    assert root['actual_two_clocks_and_stop_review_passed']
    assert root['original26_revalidated_after_FIFO_reset_change']
    assert root['predicted_physical_BSRAM'] <= 300
    assert root['actual_SYN_LUT'] < 45825 and root['actual_SYN_FF'] < 31120
    for row in root['bindings']:
        assert metadata(row['path']) == row, row
    return root


def prepare(root_path, place):
    root_path = root_path.resolve()
    root = checked(root_path)
    source = Path(root['source'])
    job = load(source/'job.json')
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0
    radar = load(Path(root['radar26_file']))
    assert radar['passed'] and len(radar['paths']) == 26
    out, control = paths(place)
    assert not out.exists() and not control.exists()
    sdc = ('create_clock -name clk50 -period 20 [get_ports {clk50}]\n'
           'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]\n'
           + sdc_fragment(radar) + '\n# One documented report per type in this same SDC.\n'
           + '\n'.join(REPORTS) + '\n')
    assert len(re.findall(r'^set_false_path .* -hold$', sdc, re.M)) == 26
    assert len(re.findall(r'^set_max_delay ', sdc, re.M)) == 26
    assert not re.search(r'^create_generated_clock|^set_clock_groups|^set_multicycle_path', sdc, re.M)
    assert same_bytes(source/'g2.cst', REFERENCE/'g2.cst')
    tcl = relocate((REFERENCE/'run.tcl').read_text(encoding='utf8'), REFERENCE, out)
    assert tcl.count('set_option -place_option 1') == 1
    tcl = tcl.replace('set_option -place_option 1', f'set_option -place_option {place}')
    assert tcl.count('run pnr') == 1 and tcl.count('add_file -type sdc') == 1
    assert '-inc_pnr' not in tcl and 'set_option -route_option 0' in tcl
    out.mkdir()
    shutil.copyfile(source/'impl/gwsynthesis/g2_top.vg', out/'g2_top.vg')
    shutil.copyfile(source/'g2.cst', out/'g2.cst')
    (out/'g2.sdc').write_text(sdc, encoding='utf8', newline='\n')
    (out/'run.tcl').write_text(tcl, encoding='utf8', newline='\n')
    shutil.copyfile(REFERENCE/'impl/pnr/device.cfg', out/'expected_from35_device.cfg')
    save(out/'cdc_reference_audit.json', radar)
    proof = dict(passed=True, source=str(source), root_review=str(root_path), place_option=place,
                 route_option=0, same_frozen_VG=True, no_new_global_requests=True,
                 same_IO_CST=True, true_clock_targets_MHz=dict(clk50=50, rgmii_rxc=100),
                 actual_new26_only_hold_and_maxdelay=True, total_limit_s=0, silent_limit_s=0,
                 documented_report_commands=list(REPORTS),
                 actual_report_coverage_requires_final_TR_review=True,
                 clock_MPW_does_not_accept_external_reset_pulse=True,
                 bindings=root['bindings']+[metadata(root_path)],
                 prepared_bindings=[metadata(out/n) for n in ('g2_top.vg', 'g2.cst', 'g2.sdc', 'run.tcl')],
                 physical_routing_timing_and_resource_acceptance=False, programmed=False)
    save(out/'prepared_request_review.json', proof)
    save(out/'manifest.json', dict(source=str(source), source_manifest=load(source/'manifest.json'),
                                  place_option=place, frozen_netlist=True, reused_completed_synthesis=True,
                                  max_runtime_minutes=0, max_no_progress_minutes=0,
                                  requires_independent_full_physical_signoff=True,
                                  pending_deployment_items=root.get('pending_deployment_items', [
                                      'P11-39 command/owner/arrival-age implementation and validation',
                                      'actual busy CPU host-loss/350ms zero submission',
                                      'external IO/reset pulse/CDC/physical stop signoff']),
                                  programmed=False, deployment_accepted=False))
    return proof


def other_peak_growth(place):
    """Avoid launching both tools against the same still-unallocated RAM reserve."""
    other, _ = paths(3-place)
    rows = []
    for p in psutil.process_iter(['pid', 'name']):
        try:
            if (p.info['name'] or '').lower() != 'gw_sh.exe' or Path(p.cwd()).resolve() != other.resolve():
                continue
            info = p.memory_info()
            rows.append(dict(pid=p.pid, created_epoch=p.create_time(), rss=info.rss,
                             private=getattr(info, 'private', info.vms)))
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    # Previous full PNRs peaked near3.3GiB. This3.5GiB reserve is an admission
    # estimate only; it never terminates or pauses an already running PNR.
    return (sum(max(0, int(3.5*GIB)-r['rss']) for r in rows),
            sum(max(0, int(3.5*GIB)-r['private']) for r in rows), rows)


def start(place):
    out, control = paths(place)
    proof = load(out/'prepared_request_review.json')
    checked(Path(proof['root_review']))
    control.mkdir(exist_ok=False)
    me = psutil.Process()
    me.nice(psutil.NORMAL_PRIORITY_CLASS)
    me.cpu_affinity(list(range(psutil.cpu_count())))
    state = dict(controller_pid=me.pid, controller_created_epoch=me.create_time(),
                 controller_parent_pid=me.ppid(), output=str(out), place_option=place,
                 total_limit_s=0, silent_limit_s=0, programmed=False)
    save(control/'launch_claim.json', state)

    def status(phase, **kw):
        state.update(phase=phase, updated_at=datetime.now().astimezone().isoformat(), **kw)
        temp = control/'job.tmp'
        save(temp, state)
        temp.replace(control/'job.json')

    try:
        settings = [s for s in (out/'run.tcl').read_text().splitlines() if s.strip() != 'run pnr']
        effective = validate(TOOL, out, settings, 50, 1,
                             extra_expected=dict(place_option=place, route_option=0, replicate_resources=1))
        save(out/'actual_preflight_options.json', effective)
        while True:
            mem = memory_status()
            pg, cg, others = other_peak_growth(place)
            status('waiting_for_actual_memory_admission', physical_bytes=mem.avail_phys,
                   commit_bytes=mem.avail_page, other_running_PNRs=others,
                   minimum_physical_bytes=2*GIB+pg, minimum_commit_bytes=4*GIB+cg)
            if mem.avail_phys >= 2*GIB+pg and mem.avail_page >= 4*GIB+cg:
                break
            time.sleep(17)
        for row in proof['bindings']+proof['prepared_bindings']:
            assert metadata(row['path']) == row, row
        assert not (out/'impl').exists() and not (out/'tool_runtime.json').exists()
        status('starting_PNR_without_deadline', admission_physical_bytes=mem.avail_phys,
               admission_commit_bytes=mem.avail_page)
        runtime = run_tool([str(TOOL), 'run.tcl'], out, max_runtime_s=0,
                           max_no_progress_s=0, capture_system_telemetry=True)
        status('tool_exited_collecting', runtime=runtime)
        finalize_gate(out)
        gate = load(out/'g2_gate.json')
        tr = out/'impl/pnr/g2_top.tr'
        if tr.exists():
            save(out/'actual_setup_endpoint_coverage.json', scan_report(tr))
        status('routed_requires_independent_full_signoff' if gate['routing_passed'] else 'physical_failure_preserved',
               runtime=runtime, routing_passed=gate['routing_passed'], unrouted=gate['unrouted_nets'],
               TR_available=tr.exists(), deployment_accepted=False)
        return int(runtime['tool_exit'] != 0 or not gate['routing_passed'])
    except Exception as exc:
        status('failed_preserved_no_automatic_retry', error=str(exc), traceback=traceback.format_exc())
        return 1


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--place', type=int, choices=(1, 2), required=True)
    p.add_argument('--prepare', type=Path, metavar='ROOT_MAPPING_REVIEW_JSON')
    p.add_argument('--start', action='store_true')
    a = p.parse_args()
    assert bool(a.prepare) != a.start
    if a.prepare:
        r = prepare(a.prepare, a.place)
        print(json.dumps(dict(passed=True, place_option=a.place, source=r['source']), ensure_ascii=False))
    else:
        raise SystemExit(start(a.place))
