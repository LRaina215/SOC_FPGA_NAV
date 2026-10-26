"""Two physical comparisons of admitted SYN45; never select old PNR53/54.

This only adapts the already-used bounded-tool controller and its output paths.
The actual source/netlist admission must be delivered independently before
--prepare; starting either place never waits for the other place to finish.
"""
import argparse
from pathlib import Path

import run_slf128_storage_place_pair as engine
from prepare_baseline_fusion_retime import load, metadata

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE/'SLF128_measured_timing_syn45'
old_checked = engine.checked


def paths(place):
    assert place in (1, 2)
    name = f'SLF128_measured_timing_place{place}_pnr{54+place}'
    return BASE/name, BASE/(name+'_one_shot')


def checked(path):
    proof = load(path)
    assert Path(proof['source']).resolve() == SOURCE.resolve()
    assert proof['actual_complete_SYN45']
    assert proof['actual_NAV6_capture_and_consumer_mapping_review_passed']
    assert proof['actual_PID_unity_and_IR_stop_mapping_review_passed']
    assert proof['actual_action_preview_register_and_source_review_passed']
    assert proof['actual_ring_mapping_or_bound_retention_review_passed']
    assert proof['no_added_setup_or_whole_domain_exceptions']
    source_job = load(SOURCE/'job.json')
    assert source_job['phase'] == 'synthesis_complete' and source_job['tool_exit'] == 0
    assert metadata(SOURCE/'impl/gwsynthesis/g2_top.vg') == proof['actual_VG_binding']
    return old_checked(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--place', type=int, choices=(1, 2), required=True)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument('--prepare', type=Path, metavar='ACTUAL_ROOT_REVIEW')
    mode.add_argument('--start', action='store_true')
    args = ap.parse_args()
    engine.paths = paths
    engine.checked = checked
    if args.prepare:
        engine.prepare(args.prepare, args.place)
        print(str(paths(args.place)[0]))
    else:
        engine.start(args.place)


if __name__ == '__main__':
    main()
