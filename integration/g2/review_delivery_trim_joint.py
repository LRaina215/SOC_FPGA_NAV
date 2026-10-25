"""Read-only pre-PNR review for explicitly scoped delivery trade-off candidates.

The profile is a scope declaration, not a waiver. Actual frozen bytes, completed
targeted proofs, primitive parameters, clocks and known CDC connections are read
again. A successful review permits an exploratory implementation run only; it
does not accept reduced performance, timing, hardware or vehicle deployment.
No synthesis, PNR, simulator, hashes or hardware operations are started here.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import re

from audit_netlist_memories import BSRAM
from audit_routing_topology import primitive_directions
from audit_sfp_cdc import audit as audit_sfp
from prepare_lf_line_joint import assets
from prepare_sfp_lane2 import verify_synthesized_clock_pins
from revalidate_lidar_cdc import definitions, revalidate
from revalidate_observation_routing import driver_index, LIBRARY
from review_lf_line_joint import direct_edges
from review_small_block_stores import parameter_reader
from synthesis_connectivity import read


CLOCKS = ('clk50_d', 'rgmii_rxc_d', 'h_rx_clk', 'h_tx_clk')
CPU_IMAGE_ASSETS = {'ram_b0.hex', 'ram_b1.hex', 'ram_b2.hex', 'ram_b3.hex', 'ram_w.hex',
                    'firmware/nav_fw.bin', 'firmware/ram_b0.hex', 'firmware/ram_b1.hex',
                    'firmware/ram_b2.hex', 'firmware/ram_b3.hex', 'firmware/ram_w.hex'}
PROTECTED_FLAGS = ('device', 'device_version', 'clock_core_mhz',
                   'control_weights_changed', 'real_sfp', 'command_lease')
PROTECTED_INPUTS = ('fixed4500_weight_rom.v', 'fc_v4_top.v',
                    'command_lease_guard.v', 'nav_command_units.v',
                    'nav/rtl/plan/safety_guard.v',
                    'nav/rtl/soc/third_party/picorv32.v', 'sfp_host_mac.v',
                    'sfp_management.v', 'ip/serdes.v', 'ip/giga_serial_ethernet.v')
PROTECTED_HIERARCHIES = ('u_control/model_thread/', 'u_sfp/', 'u_soc/u_cpu/',
                        'u_nav/g_support.u_support/', 'u_nav/u_safety/')


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def value(data, dotted):
    for part in dotted.split('.'):
        data = data[int(part)] if isinstance(data, list) else data[part]
    return data


def resolve(base, path):
    p = Path(path)
    return p.resolve() if p.is_absolute() else (Path(base)/p).resolve()


def names(root, manifest):
    result = []
    for name in manifest.get('source_files', []):
        p = Path(name)
        result.append(p.resolve().relative_to(root).as_posix() if p.is_absolute() else p.as_posix())
    assert len(result) == len(set(result)), 'duplicate input file'
    assert all((root/name).is_file() for name in result), 'missing frozen input'
    return result


def netlist(root):
    candidates = (root/'impl/gwsynthesis/g2_top.vg', root/'g2_top.vg')
    result = next((p for p in candidates if p.is_file()), None)
    assert result, ('missing actual complete netlist', str(root))
    if all(p.is_file() for p in candidates):
        assert candidates[0].read_bytes() == candidates[1].read_bytes(), 'two candidate netlists disagree'
    return result


def declared(profile, primary, alternate=None):
    result = profile.get(primary, profile.get(alternate, []) if alternate else [])
    assert isinstance(result, list), ('profile field must be a list', primary)
    return result


def within(name, prefixes):
    return any(name == p.rstrip('/') or name.startswith(p.rstrip('/')+'/') for p in prefixes)


def scopes(profile):
    result = declared(profile, 'allowed_changed_prefixes', 'allowed_trimmed_modules')
    assert result and all(isinstance(p, str) and p.strip('/') for p in result)
    # A blanket root/control/nav scope would turn the source contract into a waiver.
    assert not set(p.rstrip('/') for p in result) & {'g2_top', 'u_nav', 'u_control', 'u_sfp', 'u_soc'}
    for p in result:
        assert not any(within(p.rstrip('/'), [guard]) or within(guard.rstrip('/'), [p])
                       for guard in PROTECTED_HIERARCHIES), ('protected hierarchy in trim scope', p)
    return result


def review_proofs(profile, prepared, candidate, changed, frozen_netlist, changed_firmware=()):
    records = profile.get('proofs', [])
    if not records and profile.get('source_difference_proof'):
        records = [{'path': profile['source_difference_proof'], 'required_true_fields': ['passed']}]
    assert records and isinstance(records, list), 'no completed, source-bound targeted proof'
    bindings, checks = set(), []
    for item in records:
        proof_path = resolve(prepared, item.get('path', item.get('result_file')))
        result = load(proof_path)
        required = item.get('required_true_fields', item.get('result_required_true_fields', ['passed']))
        assert required and all(value(result, f) is True for f in required), ('targeted proof not passed', str(proof_path), required)
        for runtime in result.get('runtimes', []):
            assert runtime.get('tool_exit') == 0 and runtime.get('cancelled') is False and runtime.get('ended_at'), ('incomplete proof process', str(proof_path))
        actual_bindings = item.get('source_bindings') or result.get('source_bindings', [])
        for binding in actual_bindings:
            relative = binding['candidate']
            actual = resolve(candidate, relative)
            exercised = resolve(proof_path.parent, binding['proof_source'])
            assert actual.is_file() and exercised.is_file()
            assert actual.read_bytes() == exercised.read_bytes(), ('proof source differs from candidate', relative, str(exercised))
            bindings.add(relative)
        # Netlist-only transformations may prove all outside module text and
        # primitive pins by direct comparison in this reviewer instead of simulating.
        checks.append(dict(path=str(proof_path), required_true_fields=required,
                           exact_source_bindings=actual_bindings))
    if not frozen_netlist:
        needed = set(changed) | set(changed_firmware)
        covered = set(profile.get('proof_covered_inputs', []))
        # Coverage cannot be self-declared: an exercised transformed file must
        # be recorded for every changed RTL input. Firmware assets are separate.
        assert needed <= bindings, ('changed RTL lacks exact exercised-source binding', sorted(needed-bindings), sorted(covered))
    return checks


def inventory_cells(cells, params):
    rows = []
    for name, cell in cells.items():
        if cell['kind'] not in BSRAM:
            continue
        p = params(cell)
        wide = any(p.get(k) in ('32', '36') for k in ('BIT_WIDTH', 'BIT_WIDTH_0', 'BIT_WIDTH_1'))
        rows.append(dict(name=name, primitive=cell['kind'], parameters=p, wide=wide))
    return dict(source_BSRAM=len(rows), wide_leaves=sum(r['wide'] for r in rows),
                predicted_physical_BSRAM=len(rows)+sum(r['wide'] for r in rows),
                leaves=rows)


def changed_cells(before, after, old_params, new_params):
    added = sorted(after.keys()-before.keys())
    removed = sorted(before.keys()-after.keys())
    changed = sorted(n for n in before.keys() & after.keys()
                     if before[n] != after[n] or old_params(before[n]) != new_params(after[n]))
    return dict(added=added, removed=removed, changed=changed)


def review(source, baseline, *, prepared_only=False):
    source, baseline = Path(source).resolve(), Path(baseline).resolve()
    assert baseline.name == 'sfp_lane2_observation_lw_pnr15', 'unreviewed physical constraint/CDC reference'
    baseline_report = (baseline/'impl/pnr/g2_top.rpt.txt').read_text(encoding='utf-8')
    assert re.search(r'PR0004\).*total 184 unrouted nets', baseline_report)
    assert re.search(r'BSRAM\s+\|\s+326/340', baseline_report)
    manifest = load(source/'manifest.json')
    frozen = bool(manifest.get('frozen_netlist'))
    job = load(source/'job.json') if (source/'job.json').is_file() else {}
    prepared = source if frozen or prepared_only else Path(job['source']).resolve()
    profile_path = prepared/'trim_profile.json'
    assert profile_path.is_file(), ('missing explicit trim profile', str(profile_path))
    profile = load(profile_path)
    assert profile.get('schema_version', 1) == 1
    origin = resolve(prepared, profile.get('origin', profile.get('source_base')))
    original_manifest = load(origin/'manifest.json')
    original_job = load(origin/'job.json')
    assert original_job['phase'] == 'synthesis_complete' and original_job['tool_exit'] == 0
    kind = profile['kind']
    if kind == 'rtl_source':
        kind = 'rtl'
    assert kind in ('rtl', 'frozen_netlist') and frozen == (kind == 'frozen_netlist')
    if not frozen and not prepared_only:
        assert job.get('phase') == 'synthesis_complete' and job.get('tool_exit') == 0
        assert job.get('modified_rtl_inputs') == [], 'joint SYN must compile the prepared source without another transform'
        assert Path(manifest['frozen_source_build']).resolve() == prepared
    prepfile = prepared/'prepared_source_review.json'
    if not prepfile.is_file():
        prepfile = prepared/'prepared_review.json'
    prep = load(prepfile)
    assert prep.get('passed') is True
    assert Path(prep.get('source', prep.get('source_base'))).resolve() == origin
    allowed = scopes(profile)
    tradeoffs = profile.get('tradeoffs', profile.get('removed_functions'))
    assert tradeoffs and profile.get('deployment_accepted', False) is False
    assert not manifest.get('control_weights_changed')
    for flag in PROTECTED_FLAGS:
        assert manifest.get(flag) == original_manifest.get(flag), ('core contract flag changed', flag)
    assert manifest['clock_core_mhz'] == 50 and manifest['device_version'] == 'C' and manifest['real_sfp']
    assert not profile.get('cuts_stop_protection', False)

    oldfiles, newfiles = names(origin, original_manifest), names(source, manifest)
    source_differences = []
    firmware_changes = []
    if newfiles:
        prepared_manifest = load(prepared/'manifest.json')
        assert set(newfiles) == set(names(prepared, prepared_manifest))
        assert all((source/n).read_bytes() == (prepared/n).read_bytes() for n in newfiles)
        added = sorted(set(newfiles)-set(oldfiles)); removed = sorted(set(oldfiles)-set(newfiles))
        source_differences = sorted(n for n in set(oldfiles) & set(newfiles)
                                    if (source/n).read_bytes() != (origin/n).read_bytes())
        assert source_differences == sorted(declared(profile, 'modified_rtl_inputs')), ('RTL difference outside profile', source_differences)
        assert added == sorted(declared(profile, 'added_rtl_inputs'))
        assert removed == sorted(declared(profile, 'removed_rtl_inputs'))
        assert source_differences == sorted(prep['modified_rtl_inputs'])
        for name in PROTECTED_INPUTS:
            if name in oldfiles:
                assert name in newfiles and (source/name).read_bytes() == (origin/name).read_bytes(), ('protected input changed', name)
        from trim_peripheral_rtl import (trim_oled, trim_tx_shadow, trim_console_storage,
                                         OLED_RELATIVE, TX_SHADOW_RELATIVE, CONSOLE_RELATIVE)
        for name, transform in ((OLED_RELATIVE, trim_oled),
                                (TX_SHADOW_RELATIVE, trim_tx_shadow),
                                (CONSOLE_RELATIVE, trim_console_storage)):
            if name in source_differences:
                assert (source/name).read_text(encoding='utf-8') == transform((origin/name).read_text(encoding='utf-8')), ('peripheral change exceeds reviewed exact transform', name)
        # Control algorithm/weights sources are retained; the explicit OLED
        # boundary files and its instantiating wrapper may change only by proof.
        for name in oldfiles:
            if name.startswith('control/') and name not in {'control/oled.v', 'control/oled_driver.v', 'control/oled_front.v'}:
                assert name in newfiles and (source/name).read_bytes() == (origin/name).read_bytes(), ('control algorithm input changed', name)
        old_headers = {p.relative_to(origin).as_posix():p for p in origin.rglob('*')
                       if p.is_file() and p.suffix.lower() in {'.vh', '.h'}
                       and 'impl' not in p.relative_to(origin).parts}
        new_headers = {p.relative_to(source).as_posix():p for p in source.rglob('*')
                       if p.is_file() and p.suffix.lower() in {'.vh', '.h'}
                       and 'impl' not in p.relative_to(source).parts}
        assert old_headers.keys() == new_headers.keys(), 'frozen include-header set changed'
        for name,p in old_headers.items():
            if p.read_bytes() != new_headers[name].read_bytes():
                assert name.startswith('fw/') and name in profile.get('firmware_modified_inputs', []), ('hardware/include header changed without dedicated review', name)
        if profile.get('firmware_modified_inputs') or declared(profile, 'allowed_changed_assets', 'allowed_asset_changes'):
            old_fw = {p.relative_to(origin).as_posix():p for p in (origin/'fw').rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'}
            new_fw = {p.relative_to(source).as_posix():p for p in (source/'fw').rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'}
            assert old_fw.keys() == new_fw.keys(), 'frozen firmware source set changed'
            firmware_changes = sorted(n for n,p in old_fw.items() if p.read_bytes() != new_fw[n].read_bytes())
            assert firmware_changes == sorted(profile.get('firmware_modified_inputs', [])), ('undeclared firmware source difference', firmware_changes)

    oldassets = assets(origin)
    assert len(oldassets) == 104, ('unexpected original asset set', len(oldassets))
    candidate_assets = assets(source)
    inherited = frozen and not candidate_assets
    if inherited:
        candidate_assets = oldassets
    assert candidate_assets.keys() == oldassets.keys(), '104-asset set changed'
    asset_changes = sorted(n for n in oldassets if oldassets[n].read_bytes() != candidate_assets[n].read_bytes())
    assert asset_changes == sorted(declared(profile, 'allowed_changed_assets', 'allowed_asset_changes')), ('undeclared asset change', asset_changes)
    if asset_changes:
        assert not inherited and profile.get('firmware_provenance'), 'changed firmware lacks build provenance'
        # A new CPU navigation image is allowed, original RL/control weight
        # assets are not. Explicit paths make this distinction reviewable.
        assert set(asset_changes) <= CPU_IMAGE_ASSETS, ('non-CPU-image asset changed', asset_changes)
        bound_assets = profile['firmware_provenance'].get('output_bindings', [])
        assert {b['candidate'] for b in bound_assets} == set(asset_changes)
        for binding in bound_assets:
            assert candidate_assets[binding['candidate']].read_bytes() == resolve(prepared, binding['compiled_output']).read_bytes()
    proof_checks = review_proofs(profile, prepared, source, source_differences, frozen, firmware_changes)
    if asset_changes:
        firmware_proof_flags = ('global_initialization_radius_and_relocation_radius_retained',
            'generated_inverse_indices_match_literal_candidate_feeds',
            'all_batches_within_actual_hardware_capacity',
            'slow_forward_turn_and_reverse_categories_retained', 'failed_HAL_returns_zero_command')
        assert any(all(load(p['path']).get(f) is True for f in firmware_proof_flags)
                   for p in proof_checks), 'navigation firmware lacks completed capacity/index/radius/motion-category proof'
        provenance = profile['firmware_provenance']
        assert provenance['RAM_bytes'] == 131072 and provenance['stack_reserve_bytes'] >= 6144
        stack = load(resolve(prepared, provenance['stack_analysis']))
        assert stack['complete'] and stack['static_bound_bytes'] <= provenance['stack_reserve_bytes']
        elf = resolve(prepared, provenance['compiled_elf']).read_bytes()
        assert elf[:5] == b'\x7fELF\x01' and int.from_bytes(elf[18:20], 'little') == 243, 'CPU image did not come from RV32 ELF'

    for relative in ('g2.cst', 'g2.sdc', 'ip/serdes.csr'):
        assert (source/relative).read_bytes() == (baseline/relative).read_bytes(), ('constraint/IO/CSR differs from reviewed physical reference', relative)
    sdc = (source/'g2.sdc').read_text(encoding='utf-8')
    assert len(re.findall(r'^set_false_path\b.*-hold', sdc, re.M)) == 77
    assert len(re.findall(r'^set_max_delay\b', sdc, re.M)) == 77
    assert sorted(float(v) for v in re.findall(r'^create_clock\b.*?-period\s+(\S+)', sdc, re.M)) == [8, 8, 10, 20]
    assert '/LANE2_PCS_RX_O_FABRIC_CLK' in sdc and '/LANE2_PCS_TX_O_FABRIC_CLK' in sdc
    assert '/LANE0_PCS_' not in sdc and not re.search(r'^\s*set_clock_groups\b', sdc, re.M)
    if prepared_only:
        return dict(passed=True, profile_id=profile.get('profile_id', profile.get('profile_name')),
            source=str(source), prepared=str(prepared), origin=str(origin), baseline=str(baseline),
            modified_rtl_inputs=source_differences, modified_firmware_inputs=firmware_changes,
            unchanged_asset_count=104-len(asset_changes), asset_changes=asset_changes,
            completed_source_bound_proofs=proof_checks, tradeoffs=tradeoffs,
            source_and_compiled_CPU_image_checks_passed=True,
            known_constraints=154, hold_only=77, max_delay=77,
            clock_periods_ns=[20,10,8,8],
            review_scope='frozen prepared sources, compiled CPU image and completed targeted proofs only',
            actual_joint_synthesis_netlist_resources_clock_drivers_and_CDC_reviewed=False,
            exploratory_PNR_precheck_passed=False, synthesis_or_PNR_started=False,
            deployment_accepted=False, programmed=False)
    oldvg, newvg = netlist(origin), netlist(source)
    before, after = read(oldvg), read(newvg)
    olddefs, newdefs = definitions(oldvg), definitions(newvg)
    oldparams, newparams = parameter_reader(olddefs), parameter_reader(newdefs)
    changes = changed_cells(before, after, oldparams, newparams)
    allowed_init = declared(profile, 'allowed_firmware_init_prefixes') if asset_changes else []
    outside = []
    firmware_init = []
    for category, entries in changes.items():
        for name in entries:
            if within(name, allowed):
                continue
            if category == 'changed' and within(name, allowed_init):
                assert before[name] == after[name], ('firmware init exemption changed a connection', name)
                x, y = oldparams(before[name]), newparams(after[name])
                assert {k:v for k,v in x.items() if not k.startswith('INIT_RAM')} == {k:v for k,v in y.items() if not k.startswith('INIT_RAM')}
                firmware_init.append(name)
            else:
                outside.append(dict(category=category, name=name,
                    before=before.get(name), after=after.get(name)))
    assert not outside, ('explicit primitive/name/connection/INIT changed outside permitted scopes; do not guess automatic aliases', outside[:12], len(outside))
    oled_release = None
    if frozen and within('u_control/display', allowed):
        oled_cells = [c for n,c in after.items() if n.startswith('u_control/display/')]
        assert len(oled_cells) == 2 and all(c['kind'] == 'VCC' for c in oled_cells), 'OLED replacement must release both open-drain pins'
        for top_port, cell_name in (('sck', 'sck_iobuf'), ('sda', 'sda_iobuf')):
            io = after[cell_name]
            assert io['kind'] == 'IOBUF' and io['ports']['IO'] == [top_port] and io['ports']['I'] == ['GND']
            assert any(c['ports'] == {'V': io['ports']['OEN']} for c in oled_cells), 'OLED output enable is not driven to release'
        oled_release = True
    memory = inventory_cells(after, newparams)
    old_memory = inventory_cells(before, oldparams)
    memory_scopes = declared(profile, 'allowed_memory_change_prefixes')
    if not memory_scopes:
        memory_scopes = allowed if frozen else []
    old_memories = {r['name']: r for r in old_memory['leaves']}
    new_memories = {r['name']: r for r in memory['leaves']}
    preserved_memories = 0
    for name in old_memories.keys() | new_memories.keys():
        if within(name, memory_scopes):
            continue
        assert name in old_memories and name in new_memories, ('non-trim memory leaf added/removed', name)
        x, y = old_memories[name], new_memories[name]
        if name in firmware_init:
            assert x['primitive'] == y['primitive']
            assert {k:v for k,v in x['parameters'].items() if not k.startswith('INIT_RAM')} == {k:v for k,v in y['parameters'].items() if not k.startswith('INIT_RAM')}
        else:
            assert x == y, ('non-trim RAM geometry/INIT changed', name)
        preserved_memories += 1
    assert memory['predicted_physical_BSRAM'] <= 336, ('wide-split resource budget exceeded', memory['predicted_physical_BSRAM'])
    for field in ('source_BSRAM', 'wide_leaves', 'predicted_physical_BSRAM'):
        expected = profile.get('expected_'+field)
        if expected is not None:
            assert memory[field] == expected, ('RAM does not match prepared expectation', field, memory[field], expected)
    assert sum(c['kind'] == 'GTR12_QUAD' for c in after.values()) == 1
    directions = primitive_directions(LIBRARY)
    olddrivers, newdrivers = driver_index(before, directions), driver_index(after, directions)
    route_names = re.findall(r'^CLOCK_LOC "([^"]+)"', (source/'g2.cst').read_text(encoding='utf-8'), re.M)
    assert len(route_names) == len(set(route_names)) == 14
    driver_checks = []
    for net in route_names+list(CLOCKS):
        x, y = olddrivers.get(net, []), newdrivers.get(net, [])
        assert len(x) == len(y) == 1, ('named clock/global net absent or nonunique; inspect actual new driver', net, x, y)
        assert x[0] == y[0] and oldparams(x[0]) == newparams(y[0]), ('named driver/input/INIT changed; no unproved alias', net, x[0], y[0])
        driver_checks.append(dict(net=net, name=y[0]['name'], kind=y[0]['kind'], identical_inputs_and_INIT=True))
    clockpins = verify_synthesized_clock_pins(after)
    oldclockset = {tuple(c['ports']['CLK']) for c in before.values() if c['kind'].startswith('DFF') and 'CLK' in c['ports']}
    newclockset = {tuple(c['ports']['CLK']) for c in after.values() if c['kind'].startswith('DFF') and 'CLK' in c['ports']}
    assert newclockset == oldclockset, ('register clock domain set changed', sorted(newclockset-oldclockset), sorted(oldclockset-newclockset))
    lidar_ref = load(baseline/'cdc_reference_audit.json')
    lidar = revalidate(before, after, lidar_ref, directions, olddefs, newdefs)
    host = audit_sfp(newvg, synthesis=True)
    assert host['passed'] and len(host['paths']) == 51
    assert json.loads(json.dumps(host['paths'])) == load(baseline/'sfp_cdc_reference.json')['paths']
    for rows in (lidar_ref['paths'], host['paths']):
        for row in rows:
            for role in ('source_register', 'first_register', 'second_register'):
                name = row[role]
                assert oldparams(before[name]) == newparams(after[name]), ('known CDC endpoint INIT changed', name)
    oldedges, newedges = set(direct_edges(before)), set(direct_edges(after))
    removed_edges, added_edges = sorted(oldedges-newedges), sorted(newedges-oldedges)
    assert not added_edges and not removed_edges, ('explicit direct CDC changed; targeted review is required', added_edges, removed_edges)
    summary = load(source/'synthesis_summary.json') if (source/'synthesis_summary.json').is_file() else None
    if not frozen:
        assert summary and not summary['errors'] and summary['actual_synthesis_global_freq_mhz'] == 50
    def primitive_resources(cells):
        counts = Counter(c['kind'] for c in cells.values())
        return dict(explicit_cells=len(cells), FF=sum(n for k,n in counts.items() if k.startswith('DFF')),
                    LUT=sum(n for k,n in counts.items() if k.startswith('LUT')),
                    source_BSRAM=sum(n for k,n in counts.items() if k in BSRAM))
    before_resources, after_resources = primitive_resources(before), primitive_resources(after)
    return dict(passed=True, profile_id=profile.get('profile_id', profile.get('profile_name')),
        kind=kind, source=str(source), prepared=str(prepared), origin=str(origin), baseline=str(baseline),
        tradeoffs=tradeoffs, modified_rtl_inputs=source_differences,
        modified_firmware_inputs=firmware_changes,
        unchanged_asset_count=104-len(asset_changes), asset_changes=asset_changes,
        inherited_original_assets=inherited, completed_source_bound_proofs=proof_checks,
        OLED_actual_open_drain_release_verified=oled_release,
        allowed_changed_prefixes=allowed,
        explicit_netlist_changes={k:len(v) for k,v in changes.items()}, changed_cells=changes,
        outside_allowed_scopes_identical=True, source_BSRAM=memory['source_BSRAM'],
        original_explicit_primitive_resources=before_resources,
        candidate_explicit_primitive_resources=after_resources,
        primitive_resource_delta={k:after_resources[k]-v for k,v in before_resources.items()},
        source_bsram=memory['source_BSRAM'], wide_leaves=memory['wide_leaves'],
        predicted_physical_BSRAM=memory['predicted_physical_BSRAM'],
        predicted_physical_bsram=memory['predicted_physical_BSRAM'],
        physical_is_prediction=True, actual_memory_leaves=memory['leaves'],
        non_trim_RAM_leaves_preserved=preserved_memories,
        allowed_memory_change_prefixes=memory_scopes,
        bound_firmware_INIT_changes=firmware_init,
        driver_checks=driver_checks, strict_targets=14, true_clocks=4,
        unproved_aliases=0, actual_Lane2_clock_pins=clockpins,
        lidar26=lidar, host51=host, explicit_direct_crossings=len(newedges),
        changed_direct_crossings=dict(added=added_edges, removed=removed_edges),
        known_constraints=154, hold_only=77, max_delay=77,
        clock_periods_ns=[20,10,8,8], register_clock_domains_unchanged=True,
        original_4500_weights_125Hz_CPU_hardware_SFP_and_stop_protection_retained=True,
        complete_navigation_performance_accepted=False,
        physical_CDC_timing_PCS_reset_IO_and_extra43_signoff=False,
        actual_global_resource_allocation_verified=False,
        exploratory_PNR_precheck_only=True, deployment_accepted=False, programmed=False)


def review_prepared(source, baseline):
    """Source-only gate; deliberately cannot authorize PNR without real SYN."""
    return review(source, baseline, prepared_only=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--prepared-only', action='store_true')
    args = parser.parse_args()
    try:
        result = review(args.source, args.baseline, prepared_only=args.prepared_only)
    except (AssertionError, KeyError, FileNotFoundError, TypeError) as error:
        result = dict(passed=False, source=str(args.source.resolve()),
                      baseline=str(args.baseline.resolve()), error=str(error),
                      deployment_accepted=False)
        if args.out:
            args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 2
    encoded = json.dumps(result, ensure_ascii=False, indent=2)+'\n'
    if args.out:
        args.out.write_text(encoded, encoding='utf-8')
    print(encoded)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
