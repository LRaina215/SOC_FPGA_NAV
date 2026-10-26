"""Classify a completed routed report without importing older path claims."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import json
from pathlib import Path
import re

from analyze_pnr53_failed_endpoints import category, meta, parse_path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('build', type=Path)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    assert not a.out.exists()
    coverage = a.build / 'actual_setup_endpoint_coverage.json'
    raw = json.loads(coverage.read_text(encoding='utf8'))
    summary, rows = raw
    assert summary['status'] == 'endpoint_count_matches_summary'
    failed = [r for r in rows if r['printed_negative_slack']]
    assert len(failed) == summary['summary_setup_violated_endpoints']
    assert len({(r['to_clock'], r['to_node']) for r in failed}) == len(failed)
    groups = defaultdict(list)
    for row in failed:
        groups[category(row['to_node'])].append(row)
    representatives = {k: min(v, key=lambda r: (r['slack_ns'], r['number']))
                       for k, v in groups.items()}
    wanted = {r['number'] for r in representatives.values()}
    tr = a.build / 'impl/pnr/g2_top.tr'
    before = meta(tr)
    texts = {}
    active = False
    current = None
    buf = []
    with tr.open(encoding='utf8') as f:
        for line in f:
            if line.rstrip() == '3.3.1 Setup Analysis Report':
                active = True
            if not active:
                continue
            if line.rstrip() == '3.3.2 Hold Analysis Report':
                if current in wanted:
                    texts[current] = ''.join(buf)
                break
            match = re.fullmatch(r'\s*Path(\d+)\s*', line)
            if match:
                if current in wanted:
                    texts[current] = ''.join(buf)
                current = int(match[1])
                buf = []
            if current in wanted:
                buf.append(line)
    assert wanted == set(texts) and before == meta(tr)
    details = {str(n): parse_path(t) for n, t in texts.items()}
    classes = []
    for name, rs in groups.items():
        rep = representatives[name]
        detail = details[str(rep['number'])]
        assert float(detail['summary']['Slack']) == rep['slack_ns']
        classes.append(dict(category=name, negative_unique_endpoints=len(rs),
                            worst_slack_ns=rep['slack_ns'], worst_path_number=rep['number'],
                            representative=rep, actual_path=detail))
    classes.sort(key=lambda r: r['worst_slack_ns'])
    result = dict(recorded_at=datetime.now().astimezone().isoformat(), build=str(a.build),
                  complete=True, read_only=True, EDA_started=False, VO_loaded=False,
                  actual_failed_endpoints=len(failed),
                  all_failed_endpoint_coverage_under_printed_vendor_key=True,
                  counts_by_actual_top=dict(Counter(r['to_node'].split('/')[0] for r in failed)),
                  classes=classes,
                  scope='Every negative ToClock/ToNode printed by this report; not every alternative path, IO or full functional signoff. Cell/route statistics apply to the class representative only.',
                  source_semantics_or_older_path_interpretation_imported=False,
                  input_bindings=[meta(coverage), before], deployment_accepted=False)
    a.out.mkdir()
    (a.out / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    for n, text in texts.items():
        (a.out / f'worst_class_path{n}.txt').write_text(text, encoding='utf8')
    print(json.dumps(dict(failed=len(failed), top=result['counts_by_actual_top'],
                         classes=[{k: r[k] for k in ('category', 'negative_unique_endpoints', 'worst_slack_ns', 'worst_path_number')} for r in classes]), ensure_ascii=False))


if __name__ == '__main__':
    main()
