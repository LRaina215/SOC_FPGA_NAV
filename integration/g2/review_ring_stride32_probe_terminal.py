"""Read-only terminal archive of the completed stride32 real4500 probe2.

Never launches a tool or modifies an existing file. Use Python -B. The only
write is a new terminal-review JSON (default: probe2/terminal_review.json).
This archives finite simulation evidence; it is not an inductive RAM proof,
whole-G2 acceptance, physical timing acceptance, or deployment approval.
"""
import argparse
from datetime import datetime
import difflib
import json
from pathlib import Path
import re

import psutil


BASE = Path('F:/nav_integration_g2_20261023')
PROBE = BASE / 'ring_stride32_real4500_probe2'
PREVIOUS = BASE / 'ring_stride32_real4500_probe1'
CORE = ('rl_observation_v5_ref.v', 'rl_observation_v5_candidate.v',
        'rl_policy_tick_v5_ref.v', 'rl_policy_tick_v5_candidate.v',
        'fc_v4_top.v', 'fixed4500_weight_rom.v', 'control/fc_v4_round_sat.v')
TOOLS = ('0_xvlog', '1_xvlog', '2_xelab', '3_xsim')
ASSET_SUFFIXES = {'.hex', '.mem', '.mif', '.bin'}


def metadata(path):
    stat = path.stat()
    return dict(path=str(path.resolve()), bytes=stat.st_size, mtime_ns=stat.st_mtime_ns)


def identity_status(pid, created, sources):
    row = dict(pid=pid, recorded_created_epoch=created, record_sources=sorted(sources))
    try:
        proc = psutil.Process(pid)
        current = proc.create_time()
        same = abs(current - created) < 0.001
        row.update(original_identity_exited=not same,
                   current_pid_created_epoch=current, current_pid_name=proc.name(),
                   pid_reused=not same)
    except psutil.NoSuchProcess:
        row.update(original_identity_exited=True, current_pid_present=False, pid_reused=False)
    except psutil.AccessDenied:
        row.update(original_identity_exited=None, error='process identity inaccessible')
    return row


def review(probe=PROBE, previous=PREVIOUS, out=None):
    probe, previous = Path(probe).resolve(), Path(previous).resolve()
    out = Path(out).resolve() if out is not None else probe / 'terminal_review.json'
    if out.exists():
        raise FileExistsError('Refuse to overwrite existing evidence: ' + str(out))
    if out.suffix.lower() != '.json':
        raise ValueError('Review output must be a new JSON file')
    captured = {}

    def raw(path):
        path = Path(path).resolve()
        captured.setdefault(str(path), metadata(path))
        return path.read_bytes()

    def text(path):
        return raw(path).decode('utf-8-sig', errors='replace')

    def load(path):
        return json.loads(text(path))

    checks = {}
    result = load(probe / 'result.json')
    job = load(probe / 'job.json')
    claim = load(probe / 'run_claim.json')
    wmi = load(probe / 'WMI_launch.json')
    prep = load(probe / 'preparation.json')
    load(probe / 'fixture_declaration_order_review.json')
    checks['completed_result_and_controller_job'] = bool(result.get('passed') and job.get('passed') and
        job.get('phase') == 'completed_finite_comparison')
    checks['expected_controller_27184_identity_recorded_consistently'] = (
        job.get('controller_pid') == claim.get('controller_pid') == wmi.get('pid') == 27184 and
        job.get('controller_created_epoch') == claim.get('controller_created_epoch') and
        abs(datetime.fromisoformat(wmi['created_at']).timestamp() - job['controller_created_epoch']) < 0.001)
    logs = {name: text(probe / (name + '.log')) for name in TOOLS}
    native = text(probe / 'xsim.log')
    pass_rows = {}
    for marker, key in [('OBSERVATION_RING_REAL4500_PASS', 'normal'),
                        ('STRIDE32_TARGETED_PASS', 'targeted')]:
        rows = re.findall(r'^' + marker + r'[^\r\n]*', logs['3_xsim'], re.M)
        native_rows = re.findall(r'^' + marker + r'[^\r\n]*', native, re.M)
        pass_rows[key] = rows
        checks[key + '_PASS_present_once_and_matches_result_and_native_log'] = (
            len(rows) == 1 and rows == native_rows and rows[0] == result.get(key))
    errors = {name: re.findall(r'^.*\b(?:Fatal|FATAL|ERROR):[^\r\n]*', body, re.M)
              for name, body in {**logs, 'xsim_native': native}.items()}
    checks['no_Fatal_or_ERROR_in_four_tool_logs_or_native_xsim'] = not any(errors.values())
    frame_rows = re.findall(r'^RING_REAL4500_FRAME_MATCH[^\r\n]*', logs['3_xsim'], re.M)
    metrics = {k: float(v) if '.' in v else int(v) for k, v in
               re.findall(r'\b(\w+)=(-?\d+(?:\.\d+)?)', (pass_rows['normal'] or [''])[0])}
    targeted = {k: int(v) for k, v in re.findall(r'\b(\w+)=(\d+)',
               (pass_rows['targeted'] or [''])[0])}
    checks['published_frame_log_matches_frozen_result'] = (
        frame_rows == result.get('numeric_frames') and len(frame_rows) == metrics.get('published'))
    checks['normal_clock_and_period_are_50MHz_8ms'] = (
        metrics.get('clock_MHz') == prep.get('normal_clock_MHz') == 50 and
        metrics.get('period_us') == prep.get('normal_period_us') == 8000 and
        prep.get('normal_period_clocks') == 400000)
    runtimes = {name: load(probe / (name + '_runtime.json')) for name in TOOLS}
    final_runtime = load(probe / 'tool_runtime.json')
    checks['four_tool_exits_zero_and_ended'] = all(
        row.get('tool_exit') == 0 and row.get('ended_at') for row in runtimes.values())
    checks['last_tool_runtime_matches_archived_xsim_runtime'] = final_runtime == runtimes['3_xsim']
    identities = {}

    def add(pid, created, source):
        identities.setdefault((int(pid), float(created)), set()).add(source)

    for name, row in [('job.json', job), ('run_claim.json', claim),
                      ('tool_runtime.json', final_runtime), *[(name + '_runtime.json', row)
                      for name, row in runtimes.items()]]:
        for pkey, ckey in [('controller_pid', 'controller_created_epoch'),
                           ('wrapper_pid', 'wrapper_created_epoch')]:
            if pkey in row and ckey in row:
                add(row[pkey], row[ckey], name)
        for proc in row.get('processes', []):
            add(proc['pid'], proc['created_epoch'], name)
    add(wmi['pid'], datetime.fromisoformat(wmi['created_at']).timestamp(), 'WMI_launch.json')
    # Windows WMI timestamps can differ below one microsecond; group that record
    # with the already recorded controller identity, without accepting PID alone.
    grouped = {}
    for (pid, created), sources in identities.items():
        key = next((k for k in grouped if k[0] == pid and abs(k[1] - created) < 0.001), (pid, created))
        grouped.setdefault(key, set()).update(sources)
    processes = [identity_status(pid, created, sources) for (pid, created), sources in sorted(grouped.items())]
    checks['all_recorded_launch_tool_process_identities_exited'] = all(
        row['original_identity_exited'] is True for row in processes)
    hardware = [dict(relative_path=name, byte_identical=raw(previous / name) == raw(probe / name),
                     probe2_bytes=(probe / name).stat().st_size) for name in CORE]
    checks['all_seven_hardware_copies_byte_identical_to_probe1'] = all(r['byte_identical'] for r in hardware)

    def assets(root):
        return {p.relative_to(root).as_posix(): p for p in root.rglob('*')
            if p.is_file() and p.suffix.lower() in ASSET_SUFFIXES and
            p.relative_to(root).parts[0] != 'xsim.dir'}

    old_assets, new_assets = assets(previous), assets(probe)
    asset_rows = [dict(relative_path=name, byte_identical=raw(old_assets[name]) == raw(new_assets[name]),
                       probe2_bytes=new_assets[name].stat().st_size)
                  for name in sorted(old_assets.keys() & new_assets.keys())]
    checks['same_104_frozen_asset_names'] = old_assets.keys() == new_assets.keys() and len(new_assets) == 104
    checks['all_104_assets_byte_identical_to_probe1'] = (
        len(asset_rows) == 104 and all(r['byte_identical'] for r in asset_rows))
    old_tb = raw(previous / 'tb_ring_stride32_real4500.sv')
    new_tb = raw(probe / 'tb_ring_stride32_real4500.sv')
    before = b' integer normal_weight_reads=0,weight_frames=0,load_stops=0,clip_cases=0;'
    after = b' integer normal_weight_reads=0,weight_frames=0,load_stops=0,clip_cases=0,deadline_hits=0;'
    moved = old_tb.replace(before, after, 1)
    moved, removed = re.subn(rb'^ integer deadline_hits=0;\r?\n', b'', moved, count=1, flags=re.M)
    checks['TB_only_deadline_hits_declaration_moved_earlier'] = (
        old_tb.count(before) == 1 and removed == 1 and moved == new_tb)
    tb_diff = list(difflib.unified_diff(old_tb.decode('utf-8').splitlines(keepends=True),
        new_tb.decode('utf-8').splitlines(keepends=True), fromfile='probe1/TB', tofile='probe2/TB'))
    frozen = []
    for row in prep['frozen_inputs']:
        actual = metadata(Path(row['path']))
        frozen.append(dict(recorded=row, current=actual, matches=actual == row))
        captured.setdefault(str(Path(row['path']).resolve()), actual)
    checks['all_frozen_probe_inputs_metadata_still_match_preparation'] = all(r['matches'] for r in frozen)
    warning_lines = {name: [line.strip() for line in body.splitlines() if 'WARNING:' in line]
                     for name, body in logs.items()}
    sim_warnings = warning_lines['3_xsim']
    warning_counts = dict(total=len(sim_warnings),
        bias_truncation=sum('Data truncated' in s and 'bias.mem' in s for s in sim_warnings),
        ELU_truncation=sum('Data truncated' in s and 'elu.mem' in s for s in sim_warnings),
        ELU_too_many_words=sum('Too many words' in s and 'elu.mem' in s for s in sim_warnings))
    checks['initialization_warnings_fully_classified_not_zero'] = warning_counts == dict(
        total=10212, bias_truncation=984, ELU_truncation=9224, ELU_too_many_words=4)
    checks['one_xelab_task_index_sensitivity_warning'] = (
        len(warning_lines['2_xelab']) == 1 and '"index"' in warning_lines['2_xelab'][0] and
        'task "stop_load"' in warning_lines['2_xelab'][0])
    checks['no_xvlog_warnings'] = not warning_lines['0_xvlog'] and not warning_lines['1_xvlog']

    def words(name):
        return [int(token, 16) for token in text(probe / 'src/mem' / name).split()]

    bias, narrow_bias, elu, narrow_elu = [words(name) for name in
        ('bias.mem', 'bias_narrow.mem', 'elu.mem', 'elu13.mem')]
    checks['425_bias_low26_bits_match_narrow_asset'] = len(bias) == len(narrow_bias) == 425 and all(
        (a & ((1 << 26) - 1)) == (b & ((1 << 26) - 1)) for a, b in zip(bias, narrow_bias))
    checks['first2307_ELU_low13_bits_match_narrow_asset'] = len(elu) == 3073 and len(narrow_elu) == 2307 and all(
        (a & 8191) == (b & 8191) for a, b in zip(elu[:2307], narrow_elu))
    before_metadata = list(captured.values())
    after_metadata = [metadata(Path(row['path'])) for row in before_metadata]
    checks['all_read_input_metadata_unchanged_during_review'] = before_metadata == after_metadata
    review_result = dict(passed=all(checks.values()),
        recorded_at=datetime.now().astimezone().isoformat(), probe=str(probe), previous_probe=str(previous),
        checks=checks, normal=metrics, targeted=targeted, PASS_lines=pass_rows,
        numeric_published_frames=len(frame_rows), tool_exit_codes={name: r['tool_exit'] for name, r in runtimes.items()},
        process_identities=processes, recorded_identity_count=len(processes),
        seven_hardware_copies=hardware, frozen_assets=asset_rows,
        generated_xsim_memory_excluded_from_frozen_assets='xsim.dir/ring_stride32_real4500_snap/xsim.mem',
        TB_diff=tb_diff, frozen_inputs=frozen, tool_error_lines=errors,
        warning_counts=dict(xsim=warning_counts, xelab=len(warning_lines['2_xelab']),
            xvlog=[len(warning_lines[n]) for n in ('0_xvlog', '1_xvlog')]),
        xelab_warning=warning_lines['2_xelab'],
        initialization_warning_examples=dict(first=sim_warnings[:2], last=sim_warnings[-2:]),
        truncation_interpretation='Effective low bits match the corresponding narrow assets; full-file load warnings are retained, not counted as zero.',
        input_metadata_before=before_metadata, input_metadata_after=after_metadata,
        metadata_check_is_not_content_identity_proof=True,
        byte_equality_checks_use_direct_bytes_no_hash=True,
        existing_input_or_result_files_modified=False, new_simulation_started=False,
        SYN_PNR_started=False, programmed=False, deployment_accepted=False,
        arbitrary_initial_RAM_inductive_proof_claimed=False,
        scope='Independent terminal archive of finite paired observer/policy real4500 RTL simulation; no wholeG2, physical timing, CDC or vehicle acceptance',
        reproduction=dict(script=str(Path(__file__).resolve()), python='D:/Anaconda/python.exe',
            arguments=['-B', '-X', 'utf8', str(Path(__file__).resolve()), '--probe', str(probe),
                       '--previous', str(previous), '--out', '<new JSON path>']))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(review_result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8', newline='\n')
    return review_result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', type=Path, default=PROBE)
    parser.add_argument('--previous', type=Path, default=PREVIOUS)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    result = review(args.probe, args.previous, args.out)
    print(json.dumps(dict(passed=result['passed'], failed_checks=[k for k, v in result['checks'].items() if not v],
        numeric_published_frames=result['numeric_published_frames'], recorded_identity_count=result['recorded_identity_count'],
        warning_counts=result['warning_counts']), ensure_ascii=False))
