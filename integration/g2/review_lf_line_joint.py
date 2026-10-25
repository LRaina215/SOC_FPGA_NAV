"""Read-only full-SYN gate for an LF-only frozen candidate.

No constraint aliases, no inferred physical acceptance and no tool launch.
An unexpected source, primitive, initialization or named-driver change must
stop the caller and be examined; this reviewer implements no waiver/retry.
"""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path
import re

from audit_netlist_memories import inventory, BSRAM
from audit_routing_topology import primitive_directions
from audit_sfp_cdc import audit as audit_sfp
from continue_bank_select_once import bank_load
from prepare_lf_line_joint import load, source_files, assets, origin_review, proof_review
from prepare_lf_line_compact import transform, RELATIVE
from prepare_sfp_lane2 import verify_frozen, verify_synthesized_clock_pins
from revalidate_lidar_cdc import definitions, revalidate
from revalidate_observation_routing import driver_index, parameters, LIBRARY
from review_small_block_stores import parameter_reader, same_driver
from synthesis_connectivity import read

ROOT = Path(__file__).resolve().parents[2]
LF = 'u_nav/u_match/u_lf/'


def memory_key(row):
    return row['path'].removeprefix('g2_top/').replace('\\', '')


def direct_edges(cells):
    q = {}
    for cell in cells.values():
        if not cell['kind'].startswith('DFF') or 'CLK' not in cell['ports']:
            continue
        for net in cell['ports'].get('Q', []):
            assert net not in q, ('multiple DFF drivers', net)
            q[net] = cell
    edges = []
    for cell in cells.values():
        if not cell['kind'].startswith('DFF') or 'CLK' not in cell['ports']:
            continue
        for net in cell['ports'].get('D', []):
            source = q.get(net)
            if source and source['ports']['CLK'] != cell['ports']['CLK']:
                edges.append((source['name'], cell['name'], tuple(source['ports']['CLK']),
                              tuple(cell['ports']['CLK'])))
    return sorted(edges)


def review(source, baseline):
    source, baseline = Path(source).resolve(), Path(baseline).resolve()
    manifest, job = load(source/'manifest.json'), load(source/'job.json')
    assert manifest['lf_line_compact'] and job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0
    origin = Path(manifest['lf_line_compact_origin']).resolve()
    old_manifest, old_files, counts = origin_review(origin)
    files = source_files(source, manifest)
    assert files == old_files, 'complete input ordering/set changed'
    changed = sorted(name for name in files if (source/name).read_bytes() != (origin/name).read_bytes())
    assert changed == [RELATIVE], ('unexpected source changes', changed)
    assert (source/RELATIVE).read_text(encoding='utf-8') == transform((origin/RELATIVE).read_text(encoding='utf-8'))
    prepared = Path(job['source']).resolve()
    prep = load(prepared/'prepared_source_review.json')
    assert prep['passed'] and Path(prep['source']).resolve() == origin
    assert prep['modified_rtl_inputs'] == [RELATIVE] and prep['unchanged_hardware_inputs'] == 67
    assert prep['unchanged_assets'] == 104 and job['modified_rtl_inputs'] == []
    assert Path(manifest['frozen_source_build']).resolve() == prepared
    assert all((source/name).read_bytes() == (prepared/name).read_bytes() for name in files)
    for name in ('control_weights_changed', 'clock_core_mhz', 'device', 'device_version',
                 'real_sfp', 'full_control_telemetry', 'coverage_enabled', 'command_lease'):
        assert manifest[name] == old_manifest[name], ('contract flag changed', name)
    a, b = assets(origin), assets(source)
    assert a.keys() == b.keys() and len(a) == 104
    assert all(path.read_bytes() == b[name].read_bytes() for name, path in a.items())
    headers = [p.relative_to(origin) for p in origin.rglob('*') if p.is_file()
               and p.suffix.lower() in {'.vh', '.h'} and 'impl' not in p.relative_to(origin).parts]
    assert all((origin/p).read_bytes() == (source/p).read_bytes() for p in headers)
    for name in ('g2.cst', 'g2.sdc', 'ip/serdes.csr'):
        assert (source/name).read_bytes() == (origin/name).read_bytes(), name
    proof = Path(manifest['lf_line_compact_proof']).resolve()
    assert Path(prep['proof']).resolve() == proof
    proof_check = proof_review(proof, (origin/RELATIVE).read_text(encoding='utf-8'), require_mapped=True)

    assert baseline.name == 'sfp_lane2_observation_lw_pnr15'
    report = (baseline/'impl/pnr/g2_top.rpt.txt').read_text(encoding='utf-8')
    assert re.search(r'PR0004\).*total 184 unrouted nets', report)
    assert re.search(r'BSRAM\s+\|\s+326/340', report)
    original = origin.parent/'sfp_lane2_private_snapshot_syn11_retry1'
    assert (baseline/'g2_top.vg').read_bytes() == (original/'impl/gwsynthesis/g2_top.vg').read_bytes()
    assert (source/'ip/serdes.csr').read_bytes() == (baseline/'ip/serdes.csr').read_bytes()

    old, new = inventory(origin), inventory(source)
    expected = (counts['expected_source_BSRAM'], counts['expected_wide_leaves'],
                counts['expected_predicted_physical_BSRAM'])
    actual = (new['expanded_bsram_count'], new['wide32_36_count'], new['predicted_split_bsram'])
    assert actual == expected, ('full LF mapping differs from prediction', actual, expected)
    assert actual[2] <= 336 and new['primitive_types']['GTR12_QUAD'] == 1
    old_vg, new_vg = origin/'impl/gwsynthesis/g2_top.vg', source/'impl/gwsynthesis/g2_top.vg'
    before, after = read(old_vg), read(new_vg)
    old_defs, new_defs = definitions(old_vg), definitions(new_vg)
    old_params, new_params = parameter_reader(old_defs), parameter_reader(new_defs)
    old_nonlf = {memory_key(r): r for r in old['leaves'] if not memory_key(r).startswith(LF)}
    new_nonlf = {memory_key(r): r for r in new['leaves'] if not memory_key(r).startswith(LF)}
    assert old_nonlf.keys() == new_nonlf.keys(), 'non-LF RAM added/removed/renamed'
    for name, row in old_nonlf.items():
        assert (row['primitive'], row['parameters']) == (new_nonlf[name]['primitive'], new_nonlf[name]['parameters']), ('RAM geometry changed', name)
        assert old_params(before[name]) == new_params(after[name]), ('non-LF RAM INIT changed', name)
    new_lf = [after[memory_key(r)] for r in new['leaves'] if memory_key(r).startswith(LF)]
    assert len(new_lf) == 2
    isolated_vg = proof/'isolated_syn/impl/gwsynthesis/g2_top.vg'
    isolated = read(isolated_vg)
    isolated_params = parameter_reader(definitions(isolated_vg))
    signature = lambda cells, params: Counter((c['kind'], tuple(sorted(params(c).items()))) for c in cells)
    expected_lf = [c for c in isolated.values() if c['kind'] in BSRAM]
    assert signature(new_lf, new_params) == signature(expected_lf, isolated_params), 'joint LF geometry/INIT differs from unique isolated SYN'
    assert all(c['ports']['CLKA'] == c['ports']['CLKB'] == ['clk50_d'] and
               c['ports']['RESET'] == ['GND'] for c in new_lf), 'LF RAM clock/reset changed'
    assert all(old['by_top_instance'][name] == new['by_top_instance'][name] for name in ('u_soc', 'u_control'))

    directions = primitive_directions(LIBRARY)
    plan = load(ROOT/'integration/g2/clock_resource_plan_lane2_observation_lw.json')
    names = [r['net'] for r in plan['promote']+plan['prefer_global']]+plan['local']
    assert len(names) == len(set(names)) == 14
    cst = (baseline/'g2.cst').read_text(encoding='utf-8')
    assert re.findall(r'^CLOCK_LOC "([^"]+)"', cst, re.M) == names
    a, b = driver_index(before, directions), driver_index(after, directions)
    route = []
    for net in names+['clk50_d', 'rgmii_rxc_d', 'h_rx_clk', 'h_tx_clk']:
        assert len(a.get(net, [])) == len(b.get(net, [])) == 1, ('driver missing/nonunique', net)
        x, y = a[net][0], b[net][0]
        same_driver(x, y, old_params(x), new_params(y))
        route.append(dict(net=net, driver=y['name'], kind=y['kind'], inputs_and_INIT_identical=True))
    rejected = []
    x = a[names[0]][0]
    for mutation in ('input', 'driver', 'INIT'):
        y, px, py = copy.deepcopy(x), old_params(x), dict(old_params(x))
        if mutation == 'input':
            pin = next(p for p in y['ports'] if directions[y['kind']][p] == 'input')
            y['ports'][pin] = ['GND' if y['ports'][pin] != ['GND'] else 'VCC']
        elif mutation == 'driver':
            y['name'] += '__unproved_alias'
        else:
            py['INIT'] = "16'h0" if py.get('INIT') != "16'h0" else "16'h1"
        try:
            same_driver(x, y, px, py)
        except AssertionError:
            rejected.append(mutation)
        else:
            raise AssertionError(('invalid driver accepted', mutation))
    lane = verify_frozen(source, origin, manifest['sfp_lane2_from_build'])
    lane['actual_synthesized_clock_pins'] = verify_synthesized_clock_pins(after)
    lidar = revalidate(before, after, load(baseline/'cdc_reference_audit.json'),
                       directions, old_defs, new_defs)
    assert not lidar['proved_reset_aliases'] and lidar['changed_reset_pin_connections'] == 0
    sfp = audit_sfp(new_vg, True)
    assert sfp['passed'] and len(sfp['paths']) == 51
    assert json.loads(json.dumps(sfp['paths'])) == load(baseline/'sfp_cdc_reference.json')['paths']
    endpoints = set()
    for rows in (load(baseline/'cdc_reference_audit.json')['paths'], sfp['paths']):
        for row in rows:
            endpoints.update(row[key] for key in ('source_register', 'first_register', 'second_register'))
    for name in endpoints:
        assert before[name] == after[name] and old_params(before[name]) == new_params(after[name]), ('known CDC endpoint/INIT changed', name)
    old_edges, new_edges = direct_edges(before), direct_edges(after)
    assert old_edges == new_edges and len(old_edges) >= 77, 'new/changed explicit direct CDC'
    clock_set = lambda cells: sorted({tuple(c['ports']['CLK']) for c in cells.values()
        if c['kind'].startswith('DFF') and 'CLK' in c['ports']})
    assert clock_set(before) == clock_set(after), 'register clock-domain set changed'
    selectors = bank_load(after)
    assert selectors['maximum'] <= 128
    row_prefix = 'u_nav/u_match/u_ex/u_rb/'
    old_row = {n: c for n, c in before.items() if n.startswith(row_prefix)}
    new_row = {n: c for n, c in after.items() if n.startswith(row_prefix)}
    assert old_row == new_row and old_row, 'SLF extraction row implementation changed'
    assert all(old_params(c) == new_params(new_row[n]) for n, c in old_row.items()), 'SLF extraction row INIT changed'
    assert sum(c['kind'] == 'RAM16SDP4' for c in new_row.values()) == 250
    for prefix in ('u_nav/u_match/u_cand/', 'u_nav/u_match/u_res/'):
        a_store = {n: c for n, c in before.items() if n.startswith(prefix)}
        b_store = {n: c for n, c in after.items() if n.startswith(prefix)}
        assert a_store and a_store == b_store
        assert all(old_params(c) == new_params(b_store[n]) for n, c in a_store.items())

    sdc = (baseline/'g2.sdc').read_text(encoding='utf-8')
    assert len(re.findall(r'^set_false_path\b.*-hold', sdc, re.M)) == 77
    assert len(re.findall(r'^set_max_delay\b', sdc, re.M)) == 77
    assert sorted(float(v) for v in re.findall(r'^create_clock\b.*?-period\s+(\S+)', sdc, re.M)) == [8, 8, 10, 20]
    for name in (original, origin):
        assert (source/'g2.sdc').read_bytes() == (name/'g2.sdc').read_bytes()
    summary, prior = load(source/'synthesis_summary.json'), load(origin/'synthesis_summary.json')
    assert not summary['errors'] and summary['actual_synthesis_global_freq_mhz'] == 50
    val = lambda data, key: int(data['synthesis_hierarchy'][0].get(key, '0').split('(')[0])
    dsp = ('T_MULTALU27X18', 'T_MULT12X12', 'T_MULT27X36', 'T_MULTADDALU12X12')
    assert all(val(summary, key) == val(prior, key) for key in dsp), 'DSP resources changed'
    delta = {key: val(summary, key)-val(prior, key) for key in
             ('T_Lut', 'T_Register', 'T_Alu', 'T_Ssram', 'T_Bsram')+dsp}
    return dict(passed=True, source=str(source), baseline=str(baseline), origin=str(origin),
        proof=str(proof), modified_rtl_inputs=changed, unchanged_hardware_inputs=67, unchanged_assets=104,
        source_BSRAM=actual[0], wide_leaves=actual[1], predicted_physical_BSRAM=actual[2],
        source_bsram=actual[0], predicted_physical_bsram=actual[2], physical_is_prediction=True,
        origin_resource_counts=counts, non_LF_RAM_geometry_and_INIT_preserved=len(old_nonlf),
        actual_LF_source_BSRAM=2, actual_LF_wide_leaves=1,
        LF_geometry_and_INIT_match_unique_isolated_SYN=True, completed_proof_review=proof_check,
        driver_checks=route, strict_targets=14, true_clocks=4, unproved_aliases=0,
        negative_driver_checks_rejected=rejected, lane2=lane, lidar26=lidar, host51=sfp,
        explicit_direct_crossings=len(new_edges), all_explicit_direct_crossings_identical=True,
        known_constraints=154, hold_only=77, max_delay=77, clock_periods_ns=[20, 10, 8, 8],
        register_clock_domain_set_unchanged=True, SLF_row_RAM16_preserved=250,
        map_selector_max_load=selectors['maximum'], resource_delta=delta,
        original_control_CPU_4500_weights_125Hz_capacity_and_stop_source_unchanged=True,
        no_DDR_in_candidate=True, physical_CDC_timing_PCS_reset_IO_and_extra43_signoff=False,
        actual_global_resource_allocation_verified=False, deployment_accepted=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--out', type=Path)
    a = p.parse_args()
    result = review(a.source, a.baseline)
    encoded = json.dumps(result, indent=2)+'\n'
    if a.out:
        a.out.write_text(encoded, encoding='utf-8')
    print(encoded)


if __name__ == '__main__':
    main()
