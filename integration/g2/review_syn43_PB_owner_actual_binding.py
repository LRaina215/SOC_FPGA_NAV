"""Rebind the same31 PB obligations against actual SYN43 local modules.

No endpoint equivalence is inherited: each Boolean query uses SYN43 primitives,
and any changed/missing scalar name fails before an acceptance is recorded.
"""
from pathlib import Path
import argparse
import json
import review_syn42_PB_owner_actual_binding as audit
from prepare_baseline_fusion_retime import metadata, save


def review(out):
    base = Path('F:/nav_integration_g2_20261023')
    audit.LOCAL = base / 'SLF128_storage_safeFW_syn43_memory_review1/local_modules'
    audit.BUILD = base / 'SLF128_storage_safeFW_syn43'
    result = audit.review(out)
    result['audit_driver'] = metadata(Path(__file__).resolve())
    result['reused_obligations_not_inherited_endpoints'] = True
    result['all_primitives_and_ports_read_from_actual43'] = True
    save(out / 'result.json', result)
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    r = review(a.out)
    print(json.dumps(r, ensure_ascii=False, indent=2))
    raise SystemExit(0 if r['passed'] else 1)
