"""Read-only review of a completed three-clock Gowin timing report.

review_timing(build, out) writes one new JSON file, never a build or gate file.
It does not read a VO, run EDA, program hardware, or grant acceptance. Missing
report rows remain unknown. Run only after the build has stopped changing.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import json
import math
from pathlib import Path
import re

from audit_cdc_first_stages import sdc_fragment
from check_setup_endpoint_coverage import scan_report
from collect_pnr_result import collect
from review_fusion32_CDC52_final import parse_exception_delays


TARGETS = dict(clk50=50.0, rgmii_rxc=100.0, model25=25.0)
REPORT5000 = 'report_timing -setup -max_paths 5000 -max_common_paths 1'
NUMBER = r'-?\d+(?:\.\d+)?'
MASKED = {'clock_resource_plan.json', 'sfp_cdc_reference.json',
          'cdc_reference_audit.json'}


def normal(command):
    return ' '.join(command.split())


def status(checks):
    """False is an observed failure; None is unavailable, never a pass."""
    values = list(checks)
    return 'fail' if any(x is False for x in values) else (
        'unknown' if not values or any(x is None for x in values) else 'pass')


def metadata(path):
    if not path.exists():
        return dict(path=str(path), exists=False)
    stat = path.stat()
    return dict(path=str(path), exists=True, bytes=stat.st_size,
                mtime_ns=stat.st_mtime_ns)


class _ReportsOnlyPath:
    """Local view for collect(); no global pathlib monkeypatch or copies.

    Mask the current collector's optional netlist-audit entry points and reject
    direct VO reads through this view. Other reads are from the original build. No write
    methods are exposed. Collector acceptance/CDC results are discarded.
    """
    def __init__(self, path):
        self.path = Path(path)

    def __truediv__(self, part):
        return type(self)(self.path / part)

    def __str__(self):
        return str(self.path)

    def __fspath__(self):
        if self._masked():
            raise ValueError('Report-only collector cannot open ' + str(self.path))
        return str(self.path)

    def _masked(self):
        return self.path.suffix.lower() == '.vo' or self.path.name in MASKED

    def exists(self):
        return False if self._masked() else self.path.exists()

    def resolve(self):
        return self.path.resolve()

    def read_text(self, *args, **kwargs):
        if self._masked():
            raise ValueError('Report-only collector cannot read ' + str(self.path))
        return self.path.read_text(*args, **kwargs)


def _collector(build):
    try:
        data = collect(_ReportsOnlyPath(build))
        keys = ('errors', 'unrouted_nets', 'routing_passed', 'resources',
                'actual_pnr_global_freq_mhz', 'actual_synthesis_global_freq_mhz',
                'actual_pnr_replicate_resources', 'clocks',
                'setup_violations', 'hold_violations', 'per_clock_violations',
                'violations_outside_per_clock_summary',
                'reported_reset_pulse_checks')
        return dict(available=True, values={k: data.get(k) for k in keys},
                    physical_and_optional_CDC_audits_not_run=True,
                    collector_CDC_and_acceptance_fields_used=False)
    except Exception as exc:
        return dict(available=False, error=type(exc).__name__ + ': ' + str(exc),
                    collector_CDC_and_acceptance_fields_used=False)


def _section(text, heading, following):
    hits = list(re.finditer(r'^' + re.escape(heading) + r'\s*$', text, re.M))
    if len(hits) != 1:
        return None
    tail = text[hits[0].end():]
    end = re.search(r'^' + following, tail, re.M)
    return tail[:end.start()] if end else None


def _clocks(text, targets=TARGETS):
    summaries = defaultdict(list)
    pattern = (r'^\s*\d+[ \t]+(\S+)[ \t]+(\S+)[ \t]+(' + NUMBER +
               r')[ \t]+(' + NUMBER + r')MHz[ \t]+')
    for name, kind, period, frequency in re.findall(pattern, text, re.M):
        summaries[name].append(dict(type=kind, period_ns=float(period),
                                    frequency_mhz=float(frequency)))
    frequencies = defaultdict(list)
    pattern = (r'^\s*\d+[ \t]+(\S+)[ \t]+(' + NUMBER + r')\(MHz\)[ \t]+(' +
               NUMBER + r')\(MHz\)[ \t]+(\d+)[ \t]+(\S+)')
    for name, constraint, actual, levels, entity in re.findall(pattern, text, re.M):
        frequencies[name].append(dict(constraint_mhz=float(constraint),
            actual_fmax_mhz=float(actual), levels=int(levels), entity=entity))
    violations = defaultdict(lambda: defaultdict(list))
    for name, kind, tns, count in re.findall(
            r'^\s*(\S+)[ \t]+(setup|hold)[ \t]+(' + NUMBER +
            r')[ \t]+(\d+)[ \t]*$', text, re.M):
        violations[name][kind].append(dict(tns_ns=float(tns), violated_endpoints=int(count)))
    domains = {}
    for name, target in targets.items():
        summary = summaries[name][0] if len(summaries[name]) == 1 else None
        freq = frequencies[name][0] if len(frequencies[name]) == 1 else None
        checks = dict(clock_summary_present_once=True if summary else None,
                      fmax_summary_present_once=True if freq else None,
                      clock_frequency_matches_target=(summary['frequency_mhz'] == target if summary else None),
                      clock_period_matches_target=(math.isclose(summary['period_ns'], 1000.0 / target,
                          abs_tol=0.001) if summary else None),
                      fmax_constraint_matches_target=(freq['constraint_mhz'] == target if freq else None),
                      actual_fmax_meets_target=(freq['actual_fmax_mhz'] >= target if freq else None))
        counts = {}
        for kind in ('setup', 'hold'):
            rows = violations[name][kind]
            counts[kind] = rows[0] if len(rows) == 1 else None
            checks[kind + '_clean'] = (rows[0]['violated_endpoints'] == 0 and
                rows[0]['tns_ns'] == 0.0) if len(rows) == 1 else None
        domains[name] = dict(target_mhz=target, clock_summary=summary,
            fmax_summary=freq, violations=counts, checks=checks,
            status=status(checks.values()),
            duplicate_or_missing_row_counts=dict(clock_summary=len(summaries[name]),
                fmax_summary=len(frequencies[name]),
                setup=len(violations[name]['setup']), hold=len(violations[name]['hold'])))
    return domains, sorted(set(summaries) | set(frequencies) | set(violations))


def _global(text, domains):
    labels = dict(paths_analyzed='Paths Analyzed', endpoints_analyzed='Endpoints Analyzed',
        falling_endpoints='Falling Endpoints', setup_violated_endpoints='Setup Violated Endpoints',
        hold_violated_endpoints='Hold Violated Endpoints')
    result = {}
    for key, label in labels.items():
        found = re.findall(r'<Numbers of ' + re.escape(label) + r'>\s*:\s*(\d+)', text)
        result[key] = int(found[0]) if len(found) == 1 else None
    required_rows = [True if result[k] is not None else None for k in labels]
    result['status'] = status(required_rows + [result[k] == 0 if result[k] is not None else None
        for k in ('setup_violated_endpoints', 'hold_violated_endpoints')])
    result['violations_outside_three_domain_summary'] = {}
    for kind in ('setup', 'hold'):
        counts = [d['violations'][kind] for d in domains.values()]
        total = result[kind + '_violated_endpoints']
        result['violations_outside_three_domain_summary'][kind] = (
            total - sum(c['violated_endpoints'] for c in counts)
            if total is not None and all(c is not None for c in counts) else None)
    result['cross_clock_paths_may_be_excluded_from_per_clock_summary'] = True
    return result


def _coverage(tr, text, sdc):
    commands = re.findall(r'^\s*<Report Command>\s*:\s*(.+?)\s*$', text, re.M)
    actual = [c for c in commands if normal(c) == REPORT5000]
    result = dict(required_actual_command=REPORT5000,
        exact_command_occurrences_in_TR=len(actual),
        request_present_in_SDC=(sum(normal(c) == REPORT5000 for c in sdc.splitlines()) == 1
                                if sdc is not None else None),
        actual_5000_command_verified=(len(actual) == 1), status='unknown')
    try:
        report, endpoints = scan_report(tr)
        exact = [r for r in report['setup_reports'] if normal(r['command']) == REPORT5000]
        expected = report['summary_setup_violated_endpoints']
        unique5000 = exact[0]['unique_failed_endpoints'] if len(exact) == 1 else None
        complete5000 = (unique5000 == expected if unique5000 is not None and expected is not None else None)
        result.update(scan_report=report, unique_negative_setup_endpoints=endpoints,
            unique_failed_endpoints_in_5000_report=unique5000,
            actual_5000_report_covers_summary_under_printed_endpoint_grouping=complete5000,
            status=status([len(actual) == 1, complete5000,
                           report['status'] == 'endpoint_count_matches_summary'
                           if expected is not None else None]))
    except Exception as exc:
        result['error'] = type(exc).__name__ + ': ' + str(exc)
        result['status'] = status([len(actual) == 1, None])
    result['scope'] = ('Printed (To Clock including edge, To Node) unique negative endpoints; '
        'a matching count supports report coverage, not closure or alternate-path completeness. '
        'Printed -0.000 counts as negative, as in scan_report().')
    return result


def _constraints(build, text, sdc):
    reference_file = build / 'cdc_reference_audit.json'
    result = dict(reference=str(reference_file), expected_commands=52,
                  expected_max_delay_paths=26, status='unknown', paths=[])
    try:
        reference = json.loads(reference_file.read_text(encoding='utf-8-sig'))
        expected = [c for c in sdc_fragment(reference).splitlines() if c.startswith('set_')]
        paths = reference['paths']
        if len(set(map(normal, expected))) != 52 or not all(p.get('passed') for p in paths):
            raise ValueError('Reference must contain 26 individually passed paths and 52 unique commands')
        result['reference_reconstructed'] = True
    except Exception as exc:
        result['error'] = type(exc).__name__ + ': ' + str(exc)
        return result
    # 3.8 is the final section; accept its end of file explicitly.
    headings = list(re.finditer(r'^3\.8 Timing Constraints Report\s*$', text, re.M))
    section = text[headings[0].end():] if len(headings) == 1 else None
    active = re.findall(r'^\s*(TC_\S+)[ \t]+(\S+)[ \t]+(.+?)\s*$', section or '', re.M)
    by_command = defaultdict(list)
    for kind, state, command in active:
        by_command[normal(command)].append(dict(type=kind, state=state, command=command))
    sdc_counts = Counter(normal(c) for c in sdc.splitlines()) if sdc is not None else None
    rows = []
    for command in expected:
        reported = by_command[normal(command)]
        active_once = len(reported) == 1 and reported[0]['state'] == 'Actived'
        rows.append(dict(command=command, TR_rows=reported,
                         active_exactly_once=active_once if section is not None else None,
                         present_once_in_SDC=(sdc_counts[normal(command)] == 1
                                              if sdc_counts is not None else None)))
    expected_set = set(map(normal, expected))
    pattern = r'^\s*set_(false_path|clock_groups|multicycle_path|max_delay|min_delay)\b'
    extra_sdc = ([c.strip() for c in sdc.splitlines() if re.match(pattern, c) and
                  normal(c) not in expected_set] if sdc is not None else None)
    extra_tr = [dict(type=k, state=s, command=c) for k, s, c in active
        if re.match(pattern, c) and normal(c) not in expected_set]
    result.update(commands=rows, active_commands=sum(r['active_exactly_once'] is True for r in rows),
        unreviewed_exceptions_in_SDC=extra_sdc, unreviewed_exceptions_in_TR=extra_tr,
        status=status([r['active_exactly_once'] for r in rows] +
            [r['present_once_in_SDC'] for r in rows] +
            [not extra_sdc if extra_sdc is not None else None,
             not extra_tr if section is not None else None]))
    result['physical_endpoint_identity_revalidated'] = False
    try:
        measured = parse_exception_delays(text)
        result['strict_existing_26_delay_parser_succeeded'] = True
    except Exception as exc:
        measured = {}
        result['strict_existing_26_delay_parser_succeeded'] = False
        result['delay_parser_error'] = type(exc).__name__ + ': ' + str(exc)
    for path in paths:
        key = (path['source_register'], path['first_register'])
        record = measured.get(key)
        checks = [record['budget_ns'] == path['route_max_delay_ns'],
                  record['bounded_delay_passed']] if record else [None]
        result['paths'].append(dict(group=path['group'], source_register=key[0],
            first_register=key[1], second_register=path['second_register'],
            reference_budget_ns=path['route_max_delay_ns'], reported=record,
            status=status(checks)))
    result['max_delay_status'] = status(
        True if p['status'] == 'pass' else False if p['status'] == 'fail' else None
        for p in result['paths'])
    result['available_exact_delay_paths'] = sum(p['reported'] is not None for p in result['paths'])
    result['scope'] = ('Only the original reference 26 source-Q to first-D paths and '
        '52 exact commands. Actived is report evidence; physical endpoint identity, '
        'second-stage resolution/MTBF, related-clock channels and full CDC remain separate. '
        'The existing strict delay parser requires all 26 direct paths; any unsupported '
        'or partial delay section stays unknown rather than passing its available subset.')
    return result


def _reset_pulse(text):
    tables = {}
    specs = [('recovery', '3.1.3 Recovery Paths Table', r'3\.1\.4 '),
             ('removal', '3.1.4 Removal Paths Table', r'3\.2 '),
             ('minimum_pulse_width', '3.2 Minimum Pulse Width Table', r'3\.3 ')]
    for kind, title, following in specs:
        section = _section(text, title, following)
        rows = []
        for line in (section or '').splitlines():
            m = re.match(r'^\s*(\d+)\s+(' + NUMBER + r')\s+(.+?)\s*$', line)
            if m:
                rows.append(dict(index=int(m[1]), slack_ns=float(m[2]),
                    printed_negative_slack=m[2].startswith('-'), printed_row=line.strip()))
        # A present but empty/sample-free table supplies no numerical check.
        clean = (not any(r['printed_negative_slack'] for r in rows)) if rows else None
        tables[kind] = dict(available=section is not None, reported_rows=len(rows), rows=rows,
            worst_reported_slack_ns=min((r['slack_ns'] for r in rows), default=None),
            negative_reported_rows=sum(r['printed_negative_slack'] for r in rows),
            status=status([clean]),
            full_reset_or_board_IO_signoff=False)
    return dict(tables=tables, status=status(
        True if t['status'] == 'pass' else False if t['status'] == 'fail' else None
        for t in tables.values()), scope='Only printed recovery/removal/pulse-width table rows; limited reports do not prove all endpoints or reset sequencing.')


def review_timing(build, out, targets=None):
    build, out = Path(build).resolve(), Path(out).resolve()
    targets = TARGETS if targets is None else dict(targets)
    assert targets and all(isinstance(k, str) and float(v) > 0 for k, v in targets.items())
    if out == build or build in out.parents:
        raise ValueError('Output must be outside the frozen build')
    if out.exists():
        raise FileExistsError('Refuse to overwrite existing evidence: ' + str(out))
    tr = build / 'impl/pnr/g2_top.tr'
    inputs = [tr, build / 'impl/pnr/g2_top.rpt.txt', build / 'impl/pnr/cmd.do',
        build / 'impl/pnr/g2_top.log', build / 'gw_sh.log', build / 'manifest.json',
        build / 'impl/gwsynthesis/g2_top.prj', build / 'g2.sdc',
        build / 'cdc_reference_audit.json']
    before = [metadata(p) for p in inputs]
    result = dict(build=str(build), output=str(out),
        recorded_at=datetime.now().astimezone().isoformat(),
        timing_report=str(tr), TR_available=tr.exists(), status='unknown',
        programmed=False, new_EDA_started=False, VO_read=False,
        deployment_accepted=False, full_CDC_signoff=False,
        scope='Read-only actual three-clock TR review, never G2 or deployment acceptance')
    if not tr.exists():
        result.update(reason='Timing report unavailable; all timing/effectiveness/coverage checks unknown',
            domains={name:dict(target_mhz=target, status='unknown',
                clock_summary=None, fmax_summary=None,
                violations=dict(setup=None, hold=None)) for name,target in targets.items()},
            global_totals=None, original52_constraints=dict(status='unknown'),
            setup5000_endpoint_coverage=dict(status='unknown'),
            reset_pulse=dict(status='unknown'))
    else:
        text = tr.read_text(encoding='utf-8-sig', errors='replace')
        sdc_path = build / 'g2.sdc'
        sdc = sdc_path.read_text(encoding='utf-8-sig') if sdc_path.exists() else None
        result['collector_report_only'] = _collector(build)
        domains, names = _clocks(text, targets)
        result.update(domains=domains, actual_reported_clock_names=names,
            global_totals=_global(text, domains),
            setup5000_endpoint_coverage=_coverage(tr, text, sdc),
            original52_constraints=_constraints(build, text, sdc),
            reset_pulse=_reset_pulse(text))
        collector = result['collector_report_only']
        route = collector['values'].get('routing_passed') if collector['available'] else None
        result['routing_report_status'] = status([route])
        components = [d['status'] for d in domains.values()] + [
            result['routing_report_status'],
            result['global_totals']['status'], result['setup5000_endpoint_coverage']['status'],
            result['original52_constraints']['status'],
            result['original52_constraints'].get('max_delay_status', 'unknown'),
            result['reset_pulse']['status']]
        result['status'] = status(True if x == 'pass' else False if x == 'fail' else None for x in components)
    after = [metadata(p) for p in inputs]
    result['input_metadata_before'] = before
    result['input_metadata_after'] = after
    result['inputs_metadata_unchanged_during_read'] = before == after
    if before != after:
        result['status'] = 'unknown'
        result['reason'] = 'Inputs changed during reading; do not use this mixed snapshot'
    result['metadata_check_is_not_content_identity_proof'] = True
    result['input_files_modified'] = False
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8', newline='\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('build', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = review_timing(args.build, args.out)
    print(json.dumps({k: result[k] for k in ('build', 'TR_available', 'status',
        'inputs_metadata_unchanged_during_read', 'deployment_accepted')}, ensure_ascii=False))
