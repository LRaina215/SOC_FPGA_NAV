"""Prepare the reviewed OLED-only routing diagnostic without running tools.

The frozen SYN16 netlist and the completed, correct-Lane2 PNR19 constraints are
the input. Only the unique OLED module definition is replaced. Its two output
enables are driven high, releasing the existing open-drain top-level IOBUFs.
All other explicit cells, parameters and pins are retained at this boundary.

This file has no synthesis, placement, routing or programming entry point.
Navigation, control, complete telemetry and SFP are outside the allowed cuts.
"""
import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import re
import shutil

from audit_netlist_memories import MODULE, INST, BSRAM
from audit_routing_topology import primitive_directions
from audit_sfp_cdc import audit as audit_sfp
from prepare_sfp_lane2 import verify_synthesized_clock_pins
from revalidate_lidar_cdc import revalidate
from revalidate_observation_routing import LIBRARY, driver_index, parameters
from synthesis_connectivity import read, DECL


TARGET = 'u_control/display'
OUTPUTS = {'IIC_SCL_6': 1, 'IIC_SDA_6': 1}
FORBIDDEN_CUTS = ('u_control/model_thread', 'u_control/telemetry_thread',
                 'u_nav/g_host_extension.u_control_tx', 'u_nav/g_support.u_support',
                 'u_nav/u_tr', 'u_nav/u_match', 'u_nav/u_map', 'u_sfp')
CLOCKS = ('clk50_d', 'rgmii_rxc_d', 'h_rx_clk', 'h_tx_clk')


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2),
                    encoding='utf-8', newline='\n')


def hierarchy(modules):
    paths, multiplicity = {}, Counter()
    def walk(kind, prefix=''):
        assert prefix not in paths, ('duplicate hierarchy path', prefix)
        paths[prefix] = kind
        multiplicity[kind] += 1
        for child, name in INST.findall(modules[kind]):
            if child in modules:
                walk(child, (prefix+'/'+name.lstrip('\\')).strip('/'))
    walk('g2_top')
    return paths, multiplicity


def release_oled(kind, body):
    end = re.search(r'\)\s*;', body)
    assert end and not DECL.search(body[:end.start()]), 'unrecognized module header'
    declarations = [(m[0], *m.groups()) for m in DECL.finditer(body)
                    if m[1] in ('input', 'output', 'inout')]
    assert not any(d == 'inout' for _, d, *_ in declarations)
    outputs = {name: (hi, lo) for _, d, hi, lo, name in declarations if d == 'output'}
    assert outputs == {name: (None, None) for name in OUTPUTS}, ('OLED boundary changed', outputs)
    rows = ['module '+kind+' ('+body[:end.start()]+');']
    rows.extend(text for text, *_ in declarations)
    for index, (name, level) in enumerate(OUTPUTS.items()):
        assert level == 1
        rows.append(f'  VCC diagnostic_oled_release_{index} (\n    .V({name})\n);')
    return '\n'.join(rows+['endmodule'])


def resources(cells):
    kinds = Counter(c['kind'] for c in cells.values())
    return dict(explicit_primitives=len(cells), by_primitive=dict(kinds),
                explicit_DFF=sum(n for k, n in kinds.items() if k.startswith('DFF')),
                explicit_LUT=sum(n for k, n in kinds.items() if k.startswith('LUT')),
                source_BSRAM=sum(n for k, n in kinds.items() if k in BSRAM),
                physical_resources='unmeasured; PNR may prune and move other logic')


def prepare(source, reference, out):
    source, reference, out = (Path(p).resolve() for p in (source, reference, out))
    assert not out.exists(), 'one-shot preparation refuses an existing output'
    assert all(out != p and p not in out.parents and out not in p.parents
               for p in (source, reference)), 'output must be separate from frozen inputs'
    source_vg = source/'impl/gwsynthesis/g2_top.vg'
    frozen_bytes = source_vg.read_bytes()
    assert frozen_bytes == (reference/'g2_top.vg').read_bytes(), 'PNR19 did not use this source'
    source_manifest = json.loads((source/'manifest.json').read_text(encoding='utf-8'))
    assert source_manifest['real_sfp'] and source_manifest['full_control_telemetry']
    assert source_manifest['small_block_stores_selected'] and not source_manifest.get('row_read_shared')
    assert source_manifest['sfp_lane2_input_checks']['lane'] == 'Q0 Lane2'
    runtime = json.loads((reference/'tool_runtime.json').read_text(encoding='utf-8'))
    assert runtime.get('tool_exit') == 1 and runtime.get('cancelled') is False
    assert runtime['max_runtime_s'] == runtime['max_no_observable_progress_s'] == 0
    report = (reference/'impl/pnr/g2_top.rpt.txt').read_text(encoding='utf-8')
    assert re.search(r'PR0004\).*total 355 unrouted nets', report)
    assert re.search(r'BSRAM\s+\|\s+333/340', report)

    original = frozen_bytes.decode('utf-8')
    matches = list(MODULE.finditer(original))
    modules = {m[1]: m[2] for m in matches}
    assert len(modules) == len(matches), 'duplicate module definitions'
    paths, multiplicity = hierarchy(modules)
    kind = paths[TARGET]
    assert kind == 'oled' and multiplicity[kind] == 1, 'shared OLED definition requires instance cloning'
    target, = [m for m in matches if m[1] == kind]
    changed = original[:target.start()]+release_oled(kind, target[2])+original[target.end():]
    changed_modules = dict(MODULE.findall(changed))
    assert {k: v for k, v in modules.items() if k != kind} == {
        k: v for k, v in changed_modules.items() if k != kind}, 'outside module/INIT text changed'
    assert not (set(FORBIDDEN_CUTS) & {TARGET})

    out.mkdir(parents=True, exist_ok=False)
    (out/'impl/gwsynthesis').mkdir(parents=True)
    (out/'impl/gwsynthesis/g2_top.vg').write_text(changed, encoding='utf-8', newline='\n')
    shutil.copyfile(out/'impl/gwsynthesis/g2_top.vg', out/'g2_top.vg')
    before, after = read(source_vg), read(out/'g2_top.vg')
    prefix = TARGET+'/'
    removed = {n: c for n, c in before.items() if n.startswith(prefix)}
    added = {n: c for n, c in after.items() if n.startswith(prefix)}
    outside = lambda cells: {n: c for n, c in cells.items() if not n.startswith(prefix)}
    assert removed and len(added) == 2 and all(c['kind'] == 'VCC' for c in added.values())
    assert outside(before) == outside(after), 'outside explicit cell/pin changed'
    for port, cell_name in (('sck', 'sck_iobuf'), ('sda', 'sda_iobuf')):
        c = after[cell_name]
        expected = 'IIC_SCL_6' if port == 'sck' else 'IIC_SDA_6'
        assert c['kind'] == 'IOBUF' and c['ports']['I'] == ['GND']
        assert c['ports']['IO'] == [port] and c['ports']['OEN'] == [expected]
        assert any(c['ports'] == {'V': [expected]} for c in added.values()), 'OLED must release, not pull low'

    old_cst = (reference/'g2.cst').read_bytes()
    lines = old_cst.splitlines(keepends=True)
    removed_cst = [line for line in lines
                   if re.match(rb'\s*CLOCK_LOC\s+"'+TARGET.encode()+rb'/', line)]
    # OLED owns no clock allocation; do not release unrelated global resources.
    assert not removed_cst
    (out/'g2.cst').write_bytes(b''.join(line for line in lines if line not in removed_cst))
    assert (out/'g2.cst').read_bytes() == old_cst
    for relative in ('g2.sdc', 'cdc_reference_audit.json', 'sfp_cdc_reference.json', 'ip/serdes.csr'):
        (out/relative).parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(reference/relative, out/relative)
        assert (out/relative).read_bytes() == (reference/relative).read_bytes()
    sdc = (out/'g2.sdc').read_text(encoding='utf-8')
    assert len(re.findall(r'^create_clock\b', sdc, re.M)) == 4
    assert len(re.findall(r'^set_false_path\b.*-hold$', sdc, re.M)) == 77
    assert len(re.findall(r'^set_max_delay\b', sdc, re.M)) == 77
    assert 'LANE2_PCS_RX_O_FABRIC_CLK' in sdc and 'LANE2_PCS_TX_O_FABRIC_CLK' in sdc
    assert 'LANE0_PCS_' not in sdc and not re.search(r'^\s*set_clock_groups\b', sdc, re.M)
    directions = primitive_directions(LIBRARY)
    old_drivers, new_drivers = driver_index(before, directions), driver_index(after, directions)
    clocks = []
    route_names = re.findall(r'^CLOCK_LOC "([^"]+)"', old_cst.decode('utf-8'), re.M)
    for net in list(CLOCKS)+route_names:
        assert len(old_drivers[net]) == len(new_drivers[net]) == 1, ('nonunique driver', net)
        old, new = old_drivers[net][0], new_drivers[net][0]
        assert old == new and parameters(old, modules) == parameters(new, changed_modules)
        clocks.append(dict(net=net, driver=new['name'], kind=new['kind']))
    assert len(route_names) == 14
    physical_clock_pins = verify_synthesized_clock_pins(after)
    lidar_reference = json.loads((out/'cdc_reference_audit.json').read_text(encoding='utf-8'))
    lidar = revalidate(before, after, lidar_reference, directions, modules, changed_modules)
    host = audit_sfp(out/'g2_top.vg', synthesis=True)
    assert host['passed'] and len(host['paths']) == 51
    assert json.loads(json.dumps(host['paths'])) == json.loads(
        (out/'sfp_cdc_reference.json').read_text(encoding='utf-8'))['paths']
    for relative in ('impl/gwsynthesis/g2_top.prj', 'effective_options.tcl'):
        shutil.copyfile(source/relative, out/relative)
    manifest = dict(source_manifest)
    manifest.pop('effective_options', None)
    manifest.update(source=str(source), frozen_netlist=True, diagnostic_only=True,
        deployment_accepted=False, programmed=False, max_runtime_minutes=0,
        max_no_progress_minutes=0, source_files=[],
        scope='OLED-only frozen-netlist diagnostic; not accepted for SRAM/Flash or vehicle use',
        routing_ablation=dict(profile='01_oled_only', target=TARGET, type=kind,
            original_synthesis=str(source), physical_reference=str(reference),
            reference_unrouted=355, reference_physical_BSRAM=333,
            boundary_constants=OUTPUTS, function_loss='OLED display and its I2C traffic only',
            control_navigation_full_telemetry_and_SFP_cut=False,
            input_identity='all other module text including INIT and all explicit outside cell pins identical',
            removed_clock_constraints=[x.decode('utf-8') for x in removed_cst]))
    write_json(out/'manifest.json', manifest)
    review = dict(passed=True, prepared_at=datetime.now().astimezone().isoformat(),
        build=str(out), source=str(source), reference=str(reference), diagnostic_only=True,
        programmed=False, synthesis_started=False, pnr_started=False,
        permanent_deployment_acceptance=False, profile='01_oled_only',
        module_unique=True, module_definition_multiplicity=multiplicity[kind],
        target=TARGET, boundary_constants=OUTPUTS,
        top_IOBUF_open_drain_release_proved=True,
        removed=resources(removed), replacement=resources(added),
        outside_explicit_cells=len(outside(before)), outside_cell_pins_identical=True,
        outside_module_text_and_parameters_identical=True,
        original_IO_CST_SDC_CSR_bytes_identical=True, removed_CLOCK_LOC=[],
        original_four_clocks_and_fourteen_route_drivers=clocks,
        actual_Lane2_clock_pins=physical_clock_pins, lidar_CDC=lidar,
        host_CDC=dict(passed=True, paths=51, exact_reference_equal=True),
        timing_constraints=dict(clock_periods_ns=[20, 10, 8, 8],
                                hold_only=77, max_delay=77, original_bytes_unchanged=True),
        limits=dict(total_PNR_seconds=0, silent_PNR_seconds=0),
        not_removed=list(FORBIDDEN_CUTS),
        interpretation='A pass locates a routable peripheral-reduced configuration, not the failed-net identity. '
                       'A fail does not exonerate OLED because placement and pruning change. '
                       'Physical resources, actual global allocation, timing, CDC, IO and function still require review.')
    write_json(out/'prepared_review.json', review)
    return review


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--reference-pnr', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    review = prepare(a.source, a.reference_pnr, a.out)
    print(json.dumps({k: review[k] for k in ('passed', 'build', 'profile', 'removed',
        'outside_explicit_cells', 'original_IO_CST_SDC_CSR_bytes_identical',
        'synthesis_started', 'pnr_started', 'permanent_deployment_acceptance')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
