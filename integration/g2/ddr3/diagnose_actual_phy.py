"""Summarize an original PHY transcript without controlling the simulator.

Calibration traffic is not application readback. Timing diagnostics before
calibration are retained and counted; their cause is deliberately not inferred.
The complete transcript remains the source for every summarized diagnostic.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import re


def summarize(path):
    lines = path.read_text(errors='replace').splitlines()
    calibrated = False
    counts = Counter()
    samples = {}
    progress = []
    calibration_line = None
    readback_line = None
    for number, line in enumerate(lines, 1):
        if 'CALIBRATED at' in line:
            calibrated = True
            calibration_line = number
        if 'ACTUAL C PHY PROBE PASSED' in line:
            readback_line = number
        if 'PROGRESS ' in line:
            fields = dict(re.findall(r'(\w+)=([^\s]+)', line))
            # Micron's global $timeformat prints ps even when the old label
            # says ns. Preserve that label and convert only an explicit unit.
            stamp = re.search(r'(?:ns|time)=([\d.]+)\s*(ps|ns|us)\b', line)
            if stamp:
                fields['printed_time_unit'] = stamp[2]
                fields['simulated_time_us'] = float(stamp[1]) * {
                    'ps': 1e-6, 'ns': 1e-3, 'us': 1}[stamp[2]]
            progress.append(dict(line=number, fields=fields))
        severity = ('error' if re.search(r'\bERROR:|\*\* (?:Error|Fatal)', line)
                    else 'warning' if re.search(r'\bWARNING:|\*\* Warning', line)
                    else None)
        if severity is None:
            continue
        timing = re.search(r'\b(t\w+) violation', line)
        tool = re.search(r'\b(vsim-\d+)\b', line)
        kind = (timing[1] if timing else tool[1] if tool else
                'reset_CKE' if 'CKE must be inactive' in line else
                'reset_minimum' if '200 us is required' in line else 'other')
        context = line
        if number < len(lines) and 'Time:' in lines[number]:
            context += ' ' + lines[number]
        stamp = re.search(r'(?:at time|Time:)\s*([\d.]+)\s*(ps|ns|us)\b', context)
        time_us = (float(stamp[1]) * {'ps': 1e-6, 'ns': 1e-3, 'us': 1}[stamp[2]]
                   if stamp else None)
        phase = ('after_calibration_marker' if calibrated else
                 'time_zero' if time_us == 0 else 'before_calibration_marker')
        key = (phase, severity, kind)
        counts[key] += 1
        record = dict(line=number, text=line, simulated_time_us=time_us)
        if key not in samples:
            samples[key] = dict(first=record)
        samples[key]['last'] = record
    return dict(
        transcript=str(path.resolve()), snapshot_lines=len(lines),
        calibration_observed=calibration_line is not None,
        calibration_marker_line=calibration_line,
        readback_success_marker_seen=readback_line is not None,
        readback_success_marker_line=readback_line,
        latest_progress=progress[-1] if progress else None,
        progress_samples=progress,
        diagnostics=[dict(phase=p, severity=s, kind=k, count=n, **samples[p, s, k])
                     for (p, s, k), n in sorted(counts.items())],
        unresolved_vendor_port_warning=any('Missing connection for port' in x for x in lines),
        strict_protocol_accepted=False,
        unconditional_service_bound=False,
        complete_UPDATE_5ms_verified=False,
        limitations=[
            'A live snapshot is not a final simulator result.',
            'Pre-calibration diagnostics are not silently accepted or proof of a board defect.',
            'A readback marker alone cannot establish protocol timing or worst-case service.',
        ])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('transcript', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = summarize(args.transcript)
    encoded = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(encoded + '\n', encoding='utf-8')
    print(json.dumps({k: result[k] for k in (
        'snapshot_lines', 'calibration_observed', 'readback_success_marker_seen',
        'latest_progress', 'unresolved_vendor_port_warning')}, ensure_ascii=False))
    print(json.dumps([{k: d[k] for k in ('phase', 'severity', 'kind', 'count')}
                      for d in result['diagnostics']], ensure_ascii=False))
