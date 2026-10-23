"""Collect vendor routing/congestion evidence without granting acceptance."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re


def collect(build):
    build = Path(build).resolve()
    pnr = build/'impl/pnr'
    report = pnr/'g2_top.rpt.txt'
    timing = pnr/'g2_top.tr'
    r = report.read_text(encoding='utf-8') if report.exists() else ''
    t = timing.read_text(encoding='utf-8') if timing.exists() else ''
    failures = re.findall(r'PR0004\).*?total\s+(\d+)\s+unrouted nets', r)
    congestion, fanout = [], []
    section = None
    commands = []
    for line in t.splitlines():
        if re.match(r'^\d+\.\d+ .*Report\s*$', line):
            section = 'congestion' if 'Route Congestions Report' in line else (
                'fanout' if 'High Fanout Nets Report' in line else None)
        if section and line.startswith('Report Command:'):
            commands.append(line.removeprefix('Report Command:'))
        if section == 'congestion':
            m = re.fullmatch(r'\s*(R\d+C\d+)\s+(\d+\.\d+)\s*', line)
            if m:
                congestion.append(dict(grid=m[1], congestion=float(m[2])))
        elif section == 'fanout':
            m = re.fullmatch(r'\s*(\d+)\s+(\S+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s*', line)
            if m:
                fanout.append(dict(fanout=int(m[1]), net=m[2],
                                   worst_slack_ns=float(m[3]), max_delay_ns=float(m[4])))
    def read_json(name):
        file = build/name
        return json.loads(file.read_text(encoding='utf-8')) if file.exists() else None
    return dict(recorded_at=datetime.now().astimezone().isoformat(), build=str(build),
                pnr_report_available=bool(r), timing_report_available=bool(t),
                reported_unrouted_nets=int(failures[-1]) if failures else None,
                congestion_grids=congestion, high_fanout_nets=fanout,
                actual_report_commands=commands,
                bsram_placement_available=(pnr/'g2_top.posp').exists(),
                runtime=read_json('tool_runtime.json'), job=read_json('job.json'),
                per_net_failure_list_available=False, deployment_accepted=False,
                scope='actual vendor-reported grids/fanout only; missing reports remain unknown; '
                      'congested grids and high fanout do not identify failed nets')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('build', type=Path)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    result = collect(a.build)
    a.out.write_text(json.dumps(result, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps(dict(unrouted=result['reported_unrouted_nets'],
                          congestion_grids=len(result['congestion_grids']),
                          high_fanout_nets=len(result['high_fanout_nets']))))
