"""Admit exploratory PNR from completed actual memory and clock reviews.

This closes source/mapper entry contracts. It never accepts physical timing,
arbitrary live debug reconfiguration, the pending P11 command chain, or a car.
"""
from pathlib import Path
from datetime import datetime
import argparse
import json
import re

from audit_cdc_first_stages import sdc_fragment
from prepare_baseline_fusion_retime import load, metadata, save

BASE = Path('F:/nav_integration_g2_20261023')
LIBRARY = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
REFERENCE = BASE/'baseline_fusion32_CDC52_pnr35/cdc_reference_audit.json'


def admit(build, memory, clocks, operation, output):
    assert not output.exists(), 'Preserve prior admission results'
    job = load(build/'job.json')
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0
    resources = load(memory/'result.json')
    assert resources['passed'] and resources['complete']
    assert resources['actual_32_CEB_BLKSEL_read_gates_passed']
    operational = load(operation)
    assert operational['passed'] and operational['complete'] and operational['exploratory_PNR_entry_passed']
    assert operational['deployment_accepted'] is False
    assert operational['source_operation_contracts']
    assert not operational['current43_demonstrated_unresolved_public_numeric_or_reachable_collision_counterexamples']
    proof_paths = operational['proof_paths']
    assert Path(proof_paths['actual43_memories']).resolve() == (memory/'result.json').resolve()
    lf = load(Path(proof_paths['actual43_LF_trace']))
    weight = load(Path(proof_paths['actual43_weights']))
    local = load(Path(proof_paths['actual43_LF_trig_touch']))
    assert lf['passed'] and lf['trace_complete'] and lf['scan_run_cycles'] == 16900
    assert not lf['collision_events']
    assert all(r['read_active_equals_not_reset_and_run'] == 'UNSAT' and
               r['all66_observable_data_lanes_bound'] for r in lf['run_prefetch_actual_array_gates'])
    assert weight['passed'] and any('all40704=1' in s and 'idle_hold=81409' in s for s in weight['markers'])
    assert local['passed'] and local['LF_packing']['passed'] and local['touch_output_retiming']['passed']
    assert all(x['retiming']['proved'] for x in local['trig'])
    assert metadata(lf['actual_module']['path']) == lf['actual_module']
    assert metadata(weight['actual_module']['path']) == weight['actual_module']
    clock = load(clocks/'result.json')
    assert clock['passed'] and Path(clock['new']).resolve() == build.resolve()
    for key in ('actual_two_clock_roots_only', 'actual_original26_proved',
                'actual_FIFO_reset_types_proved', 'actual_IR_partition_proved',
                'actual_200FF_lease_partition_proved'):
        assert clock[key], key
    assert all(v['passed'] for v in clock['sections'].values())
    for row in resources['inputs']+clock['input_bindings']:
        assert metadata(row['path']) == row, row
    # Supplement the actual26 structural review with all78 INIT roles. A
    # default is read from the unmodified official macro, not presumed zero.
    cdc = load(clocks/'actual_original26_and_CDC52.json')
    reference = load(REFERENCE)
    text = LIBRARY.read_text(encoding='utf8')
    init_checks = []
    for row in cdc['rows']:
        for role, old_role in (('source', 'source_register'), ('first', 'first_register'),
                               ('second', 'second_register')):
            cell = row[role]
            matches = [x['actual_source_first_second'][old_role] for x in reference['paths']
                       if x['actual_source_first_second'][old_role]['primitive']['ports']['Q'] == cell['ports']['Q']]
            assert len(matches) == 1, (role, cell)
            defaults = re.findall(r'module\s+'+re.escape(cell['kind'])+
                                   r'\b[\s\S]*?parameter\s+INIT\s*=\s*([^;]+)', text)
            assert len(defaults) == 1
            actual = row[role+'_reset']['parameters'].get('INIT', defaults[0].strip())
            assert actual == matches[0]['effective_INIT'], (role, cell, actual, matches)
            init_checks.append(dict(role=role, name=cell['name'], kind=cell['kind'],
                                    actual_INIT=actual, reference_INIT=matches[0]['effective_INIT']))
    assert len(init_checks) == 78
    radar_path = clocks/'actual_cdc_reference_audit.json'
    radar = load(radar_path)
    assert radar['passed'] and len(radar['paths']) == 26
    commands = [s for s in sdc_fragment(radar).splitlines() if s.startswith('set_')]
    reviewed = [s for s in (clocks/'actual_SYN42_CDC52_fragment.sdc').read_text().splitlines()
                if s.startswith('set_')]
    assert commands == reviewed and len(commands) == 52
    top = resources['synthesis_resources']
    number = lambda key: int(re.match(r'^\d+', top[key])[0])
    lut, ff = number('T_Lut'), number('T_Register')
    physical = resources['split1_physical_prediction']
    assert resources['actual_source_BSRAM'] == number('T_Bsram')
    assert physical == number('T_Bsram')+resources['actual_wide_leaves']
    assert physical <= 300 and lut < 45825 and ff < 31120
    dependencies = [Path(__file__).resolve(), memory/'result.json', operation,
                    clocks/'result.json', clocks/'actual_original26_and_CDC52.json',
                    clocks/'actual_SYN42_CDC52_fragment.sdc', radar_path,
                    LIBRARY, REFERENCE, build/'impl/gwsynthesis/g2_top.vg',
                    build/'g2.cst', build/'g2.sdc', build/'manifest.json',
                    Path(__file__).with_name('run_slf128_storage_place_pair.py')]
    dependencies += [Path(v['path']) for v in clock['sections'].values()]
    for row in operational['all_input_bindings']+operational['inherited_source_proof_metadata']:
        assert metadata(row['path']) == row, row
        dependencies.append(Path(row['path']))
    dependencies += [Path(x['path']) for x in resources['inputs']+clock['input_bindings']]
    paths = {str(p.resolve()): p.resolve() for p in dependencies}
    result = dict(passed=True, at=datetime.now().astimezone().isoformat(),
        source=str(build.resolve()), SYN_resource_gate_passed=True,
        actual_storage_INIT_and_read_gate_legal=True,
        memory_legality_scope='Reviewed source operation contracts and mapped port/INIT/retiming; known LF drain and weight counterexamples closed in this actual VG. Not arbitrary live reconfiguration or full vehicle signoff.',
        source_operation_contracts=operational['source_operation_contracts'],
        actual_two_clocks_and_stop_review_passed=True,
        original26_revalidated_after_FIFO_reset_change=True, actual78_INIT_checks=init_checks,
        radar26_file=str(radar_path), predicted_physical_BSRAM=physical,
        physical_BSRAM_measured=False, actual_SYN_LUT=lut, actual_SYN_FF=ff,
        pending_deployment_items=operational['remaining_deployment_checks']+[
            'Both actual place options unrouted0 and all target setup/hold0',
            'Actual52 Actived, physical payload/CDC resolution/MTBF',
            'Reset recovery/removal/assertion pulse and source-synchronous IO budgets',
            'P11-39 safe owner/arrival-age implementation, busy CPU and physical stop',
            'Multi-seed128 P95/no-loss and >=300s patrol with error curves'],
        bindings=[metadata(paths[k]) for k in sorted(paths)],
        PNR_exploration_authorized=True, deployment_accepted=False, programmed=False)
    save(output, result)
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build', type=Path, required=True)
    p.add_argument('--memory-review', type=Path, required=True)
    p.add_argument('--clock-review', type=Path, required=True)
    p.add_argument('--operation-review', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    r = admit(a.build, a.memory_review, a.clock_review, a.operation_review, a.out)
    print(json.dumps(dict(passed=True, LUT=r['actual_SYN_LUT'], FF=r['actual_SYN_FF'],
                         predicted_physical_BSRAM=r['predicted_physical_BSRAM'],
                         bindings=len(r['bindings']), deployment_accepted=False)))
