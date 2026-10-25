"""Freeze one complete LF-only candidate. Never runs SYN, PNR or hardware.

Supported origins are the original correct-Lane2 SYN11 and the complete
shared-row SYN17. Both must have the exact LF source exercised by probe1.
The prepared manifest retains the origin/proof independently of the later
retry_frozen_synthesis parent directory. No hashes or dynamic tool actions.
"""
import argparse
import copy
from datetime import datetime
import json
from pathlib import Path
import re
import shutil

from prepare_lf_line_compact import transform, RELATIVE

ASSET_SUFFIXES = {'.hex', '.mem', '.bin', '.mif'}
ORIGINS = {
    'sfp_lane2_private_snapshot_syn11_retry1': (316, 10, 326, 314, 7, 321),
    'sfp_lane2_row_read_shared_syn17': (320, 13, 333, 318, 10, 328),
}


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def source_files(root, manifest):
    root = Path(root).resolve()
    files = [Path(name).resolve().relative_to(root).as_posix()
             for name in manifest['source_files']]
    assert len(files) == len(set(files)) == 68, 'expected the complete 68-input source'
    assert RELATIVE in files and all((root/name).is_file() for name in files)
    return files


def assets(root):
    root = Path(root)
    return {p.relative_to(root).as_posix(): p for p in root.rglob('*')
            if p.is_file() and p.suffix.lower() in ASSET_SUFFIXES
            and 'impl' not in p.relative_to(root).parts}


def proof_review(proof, original_text, require_mapped=False):
    """Read only completed results; pending mapped proof never becomes a pass."""
    proof = Path(proof).resolve()
    result = load(proof/'result.json')
    exercised = Path(result['source']).resolve()
    assert exercised.name == 'sfp_lane2_row_read_shared_syn17'
    assert (exercised/RELATIVE).read_text(encoding='utf-8') == original_text
    expected = transform(original_text)
    assert (proof/RELATIVE).read_text(encoding='utf-8') == expected
    assert result['modified_RTL_inputs'] == [RELATIVE]
    assert result['exact_inverse_transform'] and result['all2048_information_encodings_passed']
    assert (result['map_words'], result['slot_bits'], result['scan_N'], result['line_depth'],
            result['old_line_width'], result['new_line_information_width']) == (65536, 18, 256, 258, 19, 11)
    assert result['original_19bit_window_interface_retained']
    assert result['original_scan_arithmetic_precision_and_latency_retained']
    assert (proof/'unit/old.v').read_text(encoding='utf-8') == original_text.replace(
        'module slf_lf #(', 'module slf_lf_old #(', 1)
    assert (proof/'unit/new.v').read_text(encoding='utf-8') == expected.replace(
        'module slf_lf #(', 'module slf_lf_new #(', 1)
    assert (proof/'original_full_matcher/slf_lf.v').read_text(encoding='utf-8') == expected
    completed = result['functional_proofs']
    assert len(completed) >= 2
    for item, folder in zip(completed[:2], ('unit', 'original_full_matcher')):
        assert item['passed']
        assert all(r['tool_exit'] == 0 and not r['cancelled'] and r.get('ended_at')
                   for r in item['runtimes']), 'incomplete functional tool result'
        log = (proof/folder/'xsim.log').read_text(encoding='utf-8', errors='replace')
        assert 'TEST PASSED' in log and 'Fatal:' not in log and 'TEST FAILED' not in log
        assert all(line in log for line in item['result'])
    assert any(re.search(r'TEST PASSED LF .*runs=8 full_map_words=524288\b', line)
               for line in completed[0]['result'])
    assert 'original full SLF five-frame/245-candidate' in completed[1]['scope']
    syn = result['synthesis']
    assert syn['started'] and syn['completed'] and syn['runtime']['tool_exit'] == 0
    assert not syn['runtime']['cancelled'] and syn['runtime'].get('ended_at')
    assert (syn['actual_source_BSRAM'], syn['actual_wide_leaves'],
            syn['predicted_physical_BSRAM']) == (2, 1, 3)
    assert (proof/'isolated_syn/lf.v').read_text(encoding='utf-8') == expected.replace(
        'module slf_lf #(', 'module g2_top #(', 1)
    observed = {(row['kind'], tuple(sorted(row['parameters'].items()))) for row in syn['RAM']}
    assert len(observed) == 2
    assert sorted((r['kind'], r['parameters']['BIT_WIDTH_0'], r['parameters']['BIT_WIDTH_1'])
                  for r in syn['RAM']) == [('SDPB', '16', '16'), ('SDPX9B', '36', '36')]
    mapped = [item for item in completed if item.get('private_array_observation') is False
              and 'public pins and all backing words' in item.get('scope', '')]
    assert len(mapped) <= 1, 'unexpected duplicate mapped proof'
    mapped_passed = False
    if mapped:
        item = mapped[0]
        assert item['passed'] and all(r['tool_exit'] == 0 and not r['cancelled']
                                     and r.get('ended_at') for r in item['runtimes'])
        assert any(re.search(r'TEST PASSED MAPPED LF .*runs=8 full_map_words=524288\b', line)
                   for line in item['result'])
        assert Path(item['source_netlist']).resolve() == proof/'isolated_syn/impl/gwsynthesis/g2_top.vg'
        log = (proof/'vendor_public_interface/xsim.log').read_text(encoding='utf-8', errors='replace')
        assert all(line in log for line in item['result'])
        assert 'Fatal:' not in log and 'TEST FAILED' not in log
        mapped_passed = True
    if require_mapped:
        assert mapped_passed, 'mapped public-interface proof is not complete; do not start PNR'
    return dict(proof=str(proof), exact_LF_source_and_candidate_bound=True,
        unit_eight_runs_and_524288_words_passed=True,
        full_original_SLF_five_frames_245_candidates_passed=True,
        full_original_SLF_context_source=str(exercised), all2048_information_states_passed=True,
        actual_isolated_source_BSRAM=2, actual_isolated_wide_leaves=1,
        isolated_physical_BSRAM_prediction=3, isolated_geometry=syn['RAM'],
        mapped_public_interface_passed=mapped_passed,
        mapped_public_interface_status='completed_pass' if mapped_passed else 'pending_or_not_recorded',
        whole_joint_timing_or_physical_acceptance=False)


def origin_review(source):
    source = Path(source).resolve()
    assert source.name in ORIGINS, 'unsupported origin: requires frozen SYN11 or SYN17'
    manifest, job = load(source/'manifest.json'), load(source/'job.json')
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0
    assert not manifest.get('lf_line_compact') and not manifest['control_weights_changed']
    assert manifest['clock_core_mhz'] == 50 and manifest['device_version'] == 'C'
    for flag in ('snapshot_private_reset', 'slf_point_planes', 'slf_row_banks',
                 'observation_reuse_prefetch', 'bank_select_data_fanout',
                 'real_sfp', 'full_control_telemetry'):
        assert manifest[flag], flag
    shared = source.name == 'sfp_lane2_row_read_shared_syn17'
    assert bool(manifest.get('row_read_shared')) == shared
    assert bool(manifest.get('small_block_stores_selected')) == shared
    peer = source.parent/('sfp_lane2_private_snapshot_syn11_retry1' if shared
                          else 'sfp_lane2_row_read_shared_syn17')
    assert (source/RELATIVE).read_bytes() == (peer/RELATIVE).read_bytes(), 'origin LF sources differ'
    inv = load(source/'memory_inventory.json')
    counts = ORIGINS[source.name]
    assert (inv['expanded_bsram_count'], inv['wide32_36_count'],
            inv['predicted_split_bsram']) == counts[:3]
    lf = [r for r in inv['leaves'] if r['path'].startswith('g2_top/u_nav/u_match/u_lf/')]
    assert len(lf) == 4 and all(any(r['parameters'].get(k) in ('32', '36')
        for k in ('BIT_WIDTH', 'BIT_WIDTH_0', 'BIT_WIDTH_1')) for r in lf)
    return manifest, source_files(source, manifest), dict(origin=str(source), peer_origin=str(peer),
        both_frozen_LF_sources_byte_identical=True, base_source_BSRAM=counts[0],
        base_wide_leaves=counts[1], base_predicted_physical_BSRAM=counts[2],
        expected_source_BSRAM=counts[3], expected_wide_leaves=counts[4],
        expected_predicted_physical_BSRAM=counts[5], physical_is_prediction=True)


def prepare(source, out, proof):
    source, out, proof = map(lambda p: Path(p).resolve(), (source, out, proof))
    assert not out.exists() and source not in out.parents and out not in source.parents
    manifest, hardware, counts = origin_review(source)
    original = (source/RELATIVE).read_text(encoding='utf-8')
    evidence = proof_review(proof, original)
    data = assets(source)
    assert len(data) == 104
    headers = {p.relative_to(source).as_posix() for p in source.rglob('*') if p.is_file()
               and p.suffix.lower() in {'.vh', '.h'} and 'impl' not in p.relative_to(source).parts}
    extra = {'g2.cst', 'g2.sdc', 'ip/serdes.csr'}
    extra.update(p.relative_to(source).as_posix() for p in (source/'ip').iterdir()
                 if p.is_file() and p.suffix.lower() in {'.toml', '.ipc', '.mod', '.json'})
    out.mkdir(parents=True)
    for name in set(hardware) | set(data) | headers | extra:
        target = out/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source/name, target)
    (out/RELATIVE).write_text(transform(original), encoding='utf-8', newline='\n')
    changed = sorted(name for name in hardware if (source/name).read_bytes() != (out/name).read_bytes())
    assert changed == [RELATIVE]
    assert all(p.read_bytes() == (out/name).read_bytes() for name, p in data.items())
    assert all((source/name).read_bytes() == (out/name).read_bytes() for name in extra)

    def rewrite(value):
        if isinstance(value, str):
            return value.replace(source.as_posix(), out.as_posix()).replace(str(source), str(out))
        if isinstance(value, list):
            return [rewrite(v) for v in value]
        if isinstance(value, dict):
            return {k: rewrite(v) for k, v in value.items()}
        return value

    new_manifest = rewrite(copy.deepcopy(manifest))
    new_manifest.pop('effective_options', None)
    new_manifest.update(lf_line_compact=True, lf_line_compact_origin=str(source),
        lf_line_compact_proof=str(proof), lf_line_compact_only_RTL=RELATIVE,
        lf_line_compact_expected_counts=counts, programmed=False,
        complete_sfp_cdc_acceptance=False)
    original_tcl = (source/'run.tcl').read_text(encoding='utf-8')
    assert [x.strip() for x in original_tcl.splitlines() if x.strip().startswith('run ')] == ['run syn']
    (out/'run.tcl').write_text(rewrite(original_tcl), encoding='utf-8', newline='\n')
    (out/'manifest.json').write_text(json.dumps(new_manifest, indent=2)+'\n', encoding='utf-8')
    state = dict(passed=True, phase='prepared_complete_LF_only_candidate_no_build_started',
        prepared_at=datetime.now().astimezone().isoformat(), source=str(source), prepared=str(out),
        proof=str(proof), source_files=68, unchanged_hardware_inputs=67, unchanged_assets=104,
        modified_rtl_inputs=changed, copied_headers=len(headers),
        original_control_CPU_4500_weights_125Hz_capacity_and_stop_source_unchanged=True,
        original_IO_SDC_CSR_unchanged=True, origin_review=counts, completed_proof_review=evidence,
        mapped_proof_required_before_PNR=True, SYN_started=False, PNR_started=False,
        automatic_pnr=False, programmed=False, deployment_accepted=False,
        scope='Prepared frozen sources only; joint netlist, timing and physical RAM are not measured.')
    for filename, value in [('prepared_source_review.json', state), ('job.json', state)]:
        (out/filename).write_text(json.dumps(value, indent=2)+'\n', encoding='utf-8')
    return state


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('source', 'out', 'proof'):
        p.add_argument('--'+name, type=Path, required=True)
    a = p.parse_args()
    result = prepare(a.source, a.out, a.proof)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
