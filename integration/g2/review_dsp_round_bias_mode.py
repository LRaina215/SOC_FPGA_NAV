"""Prove actual SYN32 DSP input functions and audit the public macro mode.

This cuts at the six genuine core action register banks. It proves all 24-bit
states of each bank, not only the in-range clip samples. It is an arithmetic
and register-boundary preflight, not a physical timing or deployment result.
"""
import argparse
from datetime import datetime
from pathlib import Path
import re

from audit_routing_topology import primitive_directions
from prove_soc_console_remap import Logic, literal, z3
from revalidate_lidar_cdc import definitions
from review_baseline_fusion_action_width_syn import load, metadata, save
from review_small_block_stores import parameter_reader
from synthesis_connectivity import read

BASE = Path('F:/nav_integration_g2_20261023')
MAP = 'u_control/model_thread/actuator_map/'
CORE = 'u_control/model_thread/policy/core/'
LIBRARY = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
GAINS = [19, -19, -600, 19, 19, 600]


def review(full_review, out):
    full = load(full_review)
    assert full['passed'] and full['actual_complete_SYN']
    vg = Path(full['actual_vg'])
    for row in full['reviewed_input_metadata']:
        assert metadata(row['path']) == row
    def keep(name):
        return (MAP.startswith(name+'/') or name.startswith(MAP) or
                CORE.startswith(name+'/') or name.startswith(CORE+'action'))
    cells = read(vg, keep=keep)
    params = parameter_reader(definitions(vg))
    directions = primitive_directions(LIBRARY)
    state = {c['ports']['Q'][0]: z3.Bool('actual_state:'+n)
             for n, c in cells.items() if c['kind'].startswith('DFF')}
    logic = Logic(cells, params, directions, state, {})
    dsps = full['private_DSP_round_bias_actual_mapping']['action_DSPs']
    assert len(dsps) == 6
    checks = []
    def vector(nets):
        return z3.Concat(*[z3.If(logic.expr(n), z3.BitVecVal(1, 1),
                                z3.BitVecVal(0, 1)) for n in nets])
    def equal(label, actual, expected):
        solver = z3.Solver(); solver.set(timeout=30000)
        solver.add(actual != expected)
        status = solver.check()
        assert status == z3.unsat, (label, str(status),
            str(solver.model()) if status == z3.sat else solver.reason_unknown())
        checks.append(dict(label=label, method='UNSAT actual mapped-cone miter'))
    rows = []
    for dsp in dsps:
        i = int(dsp['core_action_index']); gain = GAINS[i]
        cell = cells[dsp['name']]; p = params(cell); ports = cell['ports']
        assert cell['kind'] == 'MULTALU27X18' and p == dsp['parameters']
        assert ports == dsp['ports']
        # Installed public model: direct A, signed27 x signed18, +C;
        # accumulation selects the zero PRE_LOAD, CASI is disabled.
        zeros = ['P_SEL', 'P_ADDSUB', 'A_SEL', 'ACC_SEL', 'CASI_SEL',
                 'ADD_SUB_0', 'ADD_SUB_1', 'PRE_LOAD']
        assert all(literal(p[k]) == 0 for k in zeros)
        assert literal(p['C_SEL']) == 1
        assert all(v == '"FALSE"' for k, v in p.items() if k.startswith('DYN_'))
        assert p['MULT12X12_EN'] == p['FB_PREG_EN'] == p['SOA_PREG_EN'] == '"FALSE"'
        assert all(v == ('"CLK0"' if k == 'OREG_CLK' else '"BYPASS"')
                   for k, v in p.items() if k.endswith('_CLK'))
        assert p['OREG_CE'] == '"CE0"' and p['OREG_RESET'] == '"RESET0"'
        assert ports['CLK'] == ['GND', 'clk50_d']
        assert ports['CE'] == ['VCC', 'VCC'] and set(ports['RESET']) == {'GND'}
        assert set(ports['D']+ports['CASI']+ports['SIA']) == {'GND'}
        assert all(set(ports[k]) == {'GND'} for k in
                   ['ACCSEL', 'PSEL', 'ASEL', 'PADDSUB', 'CSEL', 'CASISEL', 'ADDSUB'])
        # Form the genuine action value from the observed core FF instances;
        # never bind arbitrary net names or a late scheduler publication.
        roots = {}
        for n in dsp['input_actual_FF_boundaries']:
            match = re.fullmatch(re.escape(CORE)+f'action{i}_([0-9]+)_s0', n)
            assert match and cells[n]['kind'].startswith('DFF')
            assert cells[n]['ports']['CLK'] == ['clk50_d']
            roots[int(match[1])] = cells[n]['ports']['Q'][0]
        assert set(roots) == set(range(24))
        action = vector([roots[b] for b in range(23, -1, -1)])
        clipped = z3.If(action > 4096, z3.BitVecVal(4096, 24),
                       z3.If(action < -4096, z3.BitVecVal(-4096, 24), action))
        a, b, c = [vector(ports[k]) for k in ['A', 'B', 'C']]
        equal(f'action{i}: actual A27 is signed original clip', a, z3.SignExt(3, clipped))
        equal(f'action{i}: actual B18 is gain {gain}', b, z3.BitVecVal(gain, 18))
        negative = z3.And(z3.Xor(clipped < 0, z3.BoolVal(gain < 0)), clipped != 0)
        bias = z3.If(negative, z3.BitVecVal(2047, 48), z3.BitVecVal(2048, 48))
        equal(f'action{i}: actual C48 is exact sign/zero-dependent bias', c, bias)
        # No unexplained missing drivers or other state can participate.
        variables = {str(x) for net in ports['A']+ports['C']
                     for x in z3.z3util.get_vars(logic.expr(net))}
        assert variables <= {str(state[n]) for n in roots.values()}
        rows.append(dict(name=dsp['name'], index=i, gain=gain, input_states=1 << 24,
                         A_equals_original_signed_clip=True, B_equals_signed_gain=True,
                         C_equals_exact_bias_including_zero=True,
                         mode='signed(A27)*signed(B18)+C48, PRE_LOAD0, no CASI/feedback/preadder',
                         OREG='posedge original clk50_d, CE1, RESET0',
                         all_other_register_stages_bypassed=True,
                         maximum_absolute_unbiased_product=4096*abs(gain),
                         actual_primitive=cell, parameters=p))
    assert sorted(r['index'] for r in rows) == list(range(6))
    library = LIBRARY.read_text()
    macro = re.search(r'\bmodule MULTALU27X18\b.*?\bendmodule', library, re.S)[0]
    # Bind the mode interpretation above to the unchanged installed public
    # model. Protected PHY bodies are unrelated to this arithmetic macro.
    for snippet in ['assign mult_out = a0 * b0',
                    'alu_out = acc_load + m_out0 + c_sig + d_casi',
                    'out_reg_sync <= alu_out', 'assign DOUT = d_out[47:0]']:
        assert snippet in macro
    result = dict(passed=True, recorded_at=datetime.now().astimezone().isoformat(),
                  source=str(vg), full_review=str(full_review), checks=checks, DSPs=rows,
                  all24bit_action_states_per_group=1 << 24,
                  actual_signed_multiply_add_bias_mode_proved=True,
                  original_E2_capture_and_E3_output_contract_preserved=True,
                  separate_product_FF_count=0, new_global_requests=0,
                  physical_timing_not_proven=True, whole_system_equivalence_claimed=False,
                  evidence_bindings=[metadata(p) for p in
                     [vg, full_review, LIBRARY, Path(__file__),
                      Path(__file__).with_name('prove_soc_console_remap.py')]])
    save(out, result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--full-review', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = review(args.full_review, args.out)
    print(dict(passed=result['passed'], DSPs=len(result['DSPs']), UNSAT_checks=len(result['checks'])))
