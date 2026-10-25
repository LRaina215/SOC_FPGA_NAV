"""Run the unique PNR34 after the actual SYN25 preflight has passed.

The existing bounded PNR engine is configured in this private process only.
No old controller, source, profile or running implementation is modified.
PNR32 is the exact routed input/options reference; there are no new exceptions.
"""
import argparse
import json
from pathlib import Path

import run_baseline_single_factor_pnr as engine
from review_baseline_fusion_action_width_syn import bind_action_contract


ROOT = Path('F:/nav_integration_g2_20261023')
SOURCE = ROOT/'baseline_fusion_action_width_syn25'
PREPARED = ROOT/'baseline_fusion_action_width_prepared1'
ORIGIN = ROOT/'baseline_fusion_only_syn23'
BASELINE = ROOT/'baseline_fusion_only_pnr32'
OUTPUT = ROOT/'baseline_fusion_action_width_pnr34'
CANDIDATE = 'fusion_action_width'
ACTION = 'control/policy_action_map.v'


def check_source(profile):
    """Recheck frozen bytes and completed SYN, without another synthesis."""
    source, prepared = profile['source'], profile['prepared']
    assert source.resolve() == SOURCE.resolve()
    assert prepared.resolve() == PREPARED.resolve()
    assert profile['output'].resolve() == OUTPUT.resolve()
    job = engine.load_synthesis_job(source)
    manifest = engine.load(source/'manifest.json')
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0
    assert Path(job['source']).resolve() == prepared.resolve()
    assert Path(manifest['frozen_source_build']).resolve() == prepared.resolve()
    assert manifest['fusion_norm_pipeline'] and manifest['action_map_exact_widths']
    assert not manifest.get('pose_predecode') and not manifest['control_weights_changed']
    log = (source/'gw_sh.log').read_text(encoding='utf-8', errors='replace')
    assert 'GowinSynthesis finish' in log and '[100%] Generate report file' in log
    assert not any(line.startswith('ERROR') for line in log.splitlines())
    assert (source/'impl/gwsynthesis/g2_top.prj').is_file()
    assert (source/'impl/gwsynthesis/g2_top_syn_rsc.xml').is_file()
    proof = engine.load(prepared/'prepared_source_review.json')
    assert proof['passed'] and proof['modified_rtl_inputs'] == [ACTION]
    assert [proof[k] for k in ('unchanged_hardware_inputs', 'identical_assets',
                              'identical_headers')] == [62, 69, 31]
    bind_action_contract(source, prepared, proof)
    original, frozen, actual = [engine.hardware(root) for root in (ORIGIN, prepared, source)]
    assert original.keys() == frozen.keys() == actual.keys()
    assert sorted(k for k in original if not engine.equal_bytes(original[k], frozen[k])) == [ACTION]
    assert all(engine.equal_bytes(frozen[k], actual[k]) for k in frozen)
    for suffixes, count in [(engine.ASSETS, 69), ({'.h', '.vh'}, 31)]:
        old, prep, new = [engine.inputs(root, suffixes) for root in (ORIGIN, prepared, source)]
        assert old.keys() == prep.keys() == new.keys() and len(old) == count
        assert all(engine.equal_bytes(old[k], prep[k]) and engine.equal_bytes(old[k], new[k]) for k in old)
    for directory, count in [('fw', 61), ('firmware', 92)]:
        paths = [p for p in (ORIGIN/directory).rglob('*') if p.is_file()
                 and '__pycache__' not in p.relative_to(ORIGIN).parts]
        assert len(paths) == count
        for path in paths:
            relative = path.relative_to(ORIGIN)
            assert engine.equal_bytes(path, prepared/relative) and engine.equal_bytes(path, source/relative)
    for name in ('g2.cst', 'g2.sdc'):
        assert all(engine.equal_bytes(BASELINE/name, root/name) for root in (ORIGIN, prepared, source))
    assert (BASELINE/'g2.sdc').read_text().splitlines() == [
        'create_clock -name clk50 -period 20 [get_ports {clk50}]',
        'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]']
    assert 'CLOCK_LOC ' not in (source/'g2.cst').read_text()
    assert engine.load(BASELINE/'g2_gate.json')['routing_passed'] is True
    assert engine.equal_bytes(ORIGIN/'impl/gwsynthesis/g2_top.vg', BASELINE/'g2_top.vg')
    after = engine.load_synthesis_job(source)
    for key in ('phase', 'tool_exit', 'source', 'tool_pid', 'tool_created_epoch', 'synthesis_started_at'):
        assert after.get(key) == job.get(key), ('SYN identity changed', key)
    return manifest


def configure():
    # These bindings affect only this invocation's imported module namespace.
    engine.ORIGIN = ORIGIN
    engine.BASELINE = BASELINE
    engine.PROFILES = {CANDIDATE: dict(source=SOURCE, prepared=PREPARED, output=OUTPUT,
        changed=ACTION, offset=0, interval=13)}
    engine.check_source = check_source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--review', type=Path, required=True)
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    configure()
    review = args.review.resolve()
    if args.prepare_only:
        print(engine.prepare(engine.PROFILES[CANDIDATE], review))
        return 0
    return engine.run(CANDIDATE, review)


if __name__ == '__main__':
    raise SystemExit(main())
