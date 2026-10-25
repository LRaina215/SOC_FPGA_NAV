"""Read completed local PNR reports; never launch a build or release joint gates."""
import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import re

from audit_pin_loads import CELLS
from measure_slf_row_banks import BSRAM


def section(text, title, following):
    start = re.search(r'^' + re.escape(title) + r'\s*$', text, re.M)
    assert start, title
    tail = text[start.end():]
    end = re.search(r'^' + re.escape(following), tail, re.M)
    assert end, following
    return tail[:end.start()]


def review(build):
    pnr = build / 'impl/pnr'
    timing = (pnr / 'g2_top.tr').read_text(encoding='utf-8')
    report = (pnr / 'g2_top.rpt.txt').read_text(encoding='utf-8')
    log = (build / 'physical.log').read_text(encoding='utf-8')
    runtime = json.loads((build / 'physical_runtime.json').read_text())
    assert runtime['tool_exit'] == 0 and not runtime['cancelled'], runtime
    assert 'Placement and routing completed' in log
    assert not re.search(r'\bERROR\s*\(|Error:', log)
    assert (pnr / 'g2_top.vo').is_file()
    counts = {}
    for name, title in [('setup', 'Setup Violated Endpoints'), ('hold', 'Hold Violated Endpoints'),
                        ('paths', 'Paths Analyzed'), ('endpoints', 'Endpoints Analyzed')]:
        counts[name] = int(re.search(r'<Numbers of ' + title + r'>:\s*(\d+)', timing)[1])
    clocks = [dict(name=m[1], required_mhz=float(m[2]), actual_mhz=float(m[3])) for m in
              re.finditer(r'^\s*\d+\s+(\S+)\s+([\d.]+)\(MHz\)\s+([\d.]+)\(MHz\)', timing, re.M)]
    assert len(clocks) == 1 and clocks[0]['name'] == 'clk50' and clocks[0]['required_mhz'] == 50
    assert re.search(r'TC_CLOCK\s+Actived\s+create_clock -name clk50 -period 20', timing)
    paths = {}
    for key, title, after in [('setup', '3.1.1 Setup Paths Table', '3.1.2 Hold Paths Table'),
                              ('hold', '3.1.2 Hold Paths Table', '3.1.3 Recovery Paths Table')]:
        rows = []
        for line in section(timing, title, after).splitlines():
            fields = line.split()
            if len(fields) == 9 and fields[0].isdigit():
                rows.append(dict(slack_ns=float(fields[1]), source=fields[2], destination=fields[3],
                                 source_clock=fields[4], destination_clock=fields[5], data_delay_ns=float(fields[8])))
        assert rows, key
        paths[key] = dict(worst_reported_slack_ns=min(x['slack_ns'] for x in rows),
                          reported_rows=len(rows), worst=rows[0],
                          row_memory_rows=[r for r in rows if 'u_rb/' in str(r) or 'rbq1' in str(r)])
    extras = {}
    for key, title, after in [('recovery', '3.1.3 Recovery Paths Table', '3.1.4 Removal Paths Table'),
                              ('removal', '3.1.4 Removal Paths Table', '3.2 Minimum Pulse Width Table'),
                              ('pulse', '3.2 Minimum Pulse Width Table', '3.3 Timing Report By Analysis Type')]:
        values = [float(x) for x in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s+\S+', section(timing, title, after), re.M)]
        extras[key] = dict(reported_rows=len(values), worst_slack_ns=min(values) if values else None)
    resources = {}
    for key in ('Logic', 'Register', 'CLS', 'BSRAM'):
        m = re.search(r'^\s*' + key + r'\s*\|\s*(\d+)/(\d+)', report, re.M)
        resources[key] = int(m[1]) if m else None
    resources['RAM16'] = int(re.search(r'--SSRAM\(RAM16\)\s*\|\s*(\d+)', report)[1])
    resources['LUT'] = int(re.search(r'\((\d+) LUT,', report)[1])
    physical_text=(pnr/'g2_top.vo').read_text(encoding='utf-8')
    primitive_counts=Counter(m[1] for m in CELLS.finditer(physical_text))
    assert sum(v for k,v in primitive_counts.items() if k.startswith('DFF'))==resources['Register']
    physical_bsram=sum(primitive_counts[k] for k in BSRAM)
    if resources['BSRAM'] is not None:assert resources['BSRAM']==physical_bsram
    resources['BSRAM']=physical_bsram
    glob = section(report, '6. Global Clock Signals', '7. Pinout')
    globals_used = []
    for line in glob.splitlines():
        if '| PRIMARY' in line or '| LW ' in line:
            fields = line.split('|')
            globals_used.append(dict(net=fields[0].strip(), resource=fields[1].strip()))
    return dict(build=str(build), report=str(pnr/'g2_top.rpt.txt'), timing_report=str(pnr/'g2_top.tr'),
                runtime=runtime, routing_completed=True,
                explicit_unrouted_count=None, routing_evidence='Natural successful routing completion, physical netlist and STA; no explicit unrouted count printed.',
                clocks=clocks, counts=counts, resources=resources, paths=paths, reset_pulse=extras,
                global_allocations=globals_used,
                internal_clock_50MHz_pass=bool(counts['setup']==counts['hold']==0 and clocks[0]['actual_mhz']>=50),
                fixture_external_IO_delays_constrained=False,
                joint_global_resource_occupancy_reproduced=False,
                all_row_read_paths_individually_listed=False,
                full_joint_acceptance=False)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--shared', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args=parser.parse_args()
    result=dict(recorded_at=datetime.now().astimezone().isoformat(),
                baseline=review(args.baseline), shared=review(args.shared),
                scope='Local complete-extractor context comparison only; original joint PNR unchanged.')
    result['physical_resource_delta']={k:result['shared']['resources'][k]-result['baseline']['resources'][k]
        for k in result['baseline']['resources'] if result['baseline']['resources'][k] is not None and result['shared']['resources'][k] is not None}
    result['full_joint_acceptance']=False
    args.out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:{'clocks':result[k]['clocks'],'counts':result[k]['counts'],'resources':result[k]['resources'],
                         'global_allocations':result[k]['global_allocations']} for k in ('baseline','shared')}))


if __name__=='__main__':main()
