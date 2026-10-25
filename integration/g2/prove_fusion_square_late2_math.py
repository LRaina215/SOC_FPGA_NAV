"""Independent small mathematical proof; no RTL/tool build or queue mutation."""
from pathlib import Path
from datetime import datetime
import json
import argparse
import math
import random
import re
import time
import sys

import psutil
sys.path.insert(0, 'F:/nav_integration_g2_20261023/routing_proof_dependencies')
import z3

OUT = None
SOURCE = Path('F:/nav_integration_g2_20261023/baseline_fusion_only_syn23/control')
CANDIDATE = Path('F:/nav_integration_g2_20261023/fusion_square_pipeline2_candidate1/imu_fusion.v')
MASK = (1 << 64)-1


def meta(path):
    s = path.stat()
    return dict(path=str(path), bytes=s.st_size, mtime_ns=s.st_mtime_ns)


def prove(label, assumptions, mismatch):
    solver = z3.Solver()
    solver.set(timeout=5000)
    solver.add(*assumptions, mismatch)
    started = time.perf_counter()
    status = solver.check()
    result = dict(label=label, status=str(status), passed=status == z3.unsat,
                  elapsed_ms=(time.perf_counter()-started)*1000)
    if status == z3.sat:
        result['counterexample'] = str(solver.model())
    elif status == z3.unknown:
        result['reason'] = solver.reason_unknown()
    assert result['passed'], result
    return result


def step_bv(state):
    rem, acc, bitv, cnt = state
    take = z3.UGE(rem, acc+bitv)
    return (z3.If(take, rem-(acc+bitv), rem),
            z3.If(take, z3.LShR(acc, 1)+bitv, z3.LShR(acc, 1)),
            z3.LShR(bitv, 2), cnt+1)


def abs24(bits):
    bits &= (1 << 24)-1
    return (-bits if bits & (1 << 23) else bits) & ((1 << 24)-1)


def model(value, late2=False):
    # State immediately after accepted start. The delayed input is captured at
    # cnt1, two edges later. Both retain the separate cnt32 completion edge.
    rem, acc, bitv, cnt = (0 if late2 else value), 0, 1 << 62, 0
    trace = []
    while cnt < 32:
        if late2 and cnt < 2:
            if cnt == 1:
                rem = value
            acc >>= 1
        elif rem >= acc+bitv:
            rem -= acc+bitv
            acc = (acc >> 1)+bitv
        else:
            acc >>= 1
        bitv >>= 2
        cnt += 1
        trace.append((rem & MASK, acc & MASK, bitv, cnt))
    return acc & 0xffffffff, trace, 33


def engine_edge(state, *, reset, start, value, late2):
    busy, done, cnt, rem, bitv, acc, root = state
    if reset:
        return (0, 0, 0, 0, 0, 0, 0)
    done = 0
    if not busy:
        if start:
            busy, cnt, acc, rem, bitv = 1, 0, 0, (0 if late2 else value), 1 << 62
    elif cnt == 32:
        busy, done, root = 0, 1, acc & 0xffffffff
    elif late2 and cnt < 2:
        if cnt == 1:
            rem = value
        acc, bitv, cnt = 0, bitv >> 2, cnt+1
    else:
        if rem >= acc+bitv:
            rem, acc = rem-(acc+bitv), (acc >> 1)+bitv
        else:
            acc >>= 1
        bitv, cnt = bitv >> 2, cnt+1
    return (busy, done, cnt, rem, bitv, acc, root)


def main():
    global OUT, SOURCE, CANDIDATE
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--source',type=Path,default=SOURCE)
    parser.add_argument('--candidate',type=Path,default=CANDIDATE)
    args=parser.parse_args()
    OUT=args.out.resolve();SOURCE=args.source.resolve();CANDIDATE=args.candidate.resolve()
    assert not OUT.exists(), 'preserve existing proof output; choose a new directory'
    OUT.mkdir(parents=True)
    me = psutil.Process()
    me.nice(psutil.IDLE_PRIORITY_CLASS)
    me.cpu_affinity([16])
    fusion = (SOURCE/'imu_fusion.v').read_text(encoding='utf-8')
    sqrt = (SOURCE/'imu_isqrt.v').read_text(encoding='utf-8')
    abs_source = re.search(r'    function \[23:0\] abs24;.*?    endfunction', fusion, re.S)[0]
    assert 'input signed [23:0] v;' in abs_source
    assert 'begin abs24 = v[23] ? -v : v; end' in abs_source
    assert 'reg [63:0] norm_square_x=0,norm_square_y=0,norm_square_z=0;' in fusion
    assert 'wire [63:0] sqrt_value=norm_square_x+norm_square_y+norm_square_z;' in fusion
    for coordinate in ('x', 'y', 'z'):
        assert f'norm_square_{coordinate}<=abs24(eg{coordinate}) * abs24(eg{coordinate});' in fusion
    assert "bitv <= 64'd1 << 62;" in sqrt and "cnt == 6'd32" in sqrt
    assert 'root <= acc[31:0];' in sqrt
    candidate = CANDIDATE.read_text(encoding='utf-8')
    helper = candidate[candidate.index('module imu_fusion_isqrt_late2'):]
    reset_block = 'busy <= 0; done <= 0; cnt <= 0; rem <= 0; bitv <= 0; acc <= 0; root <= 0;'
    normalize = lambda s: ' '.join(s.split())
    assert normalize(reset_block) in normalize(sqrt) and normalize(reset_block) in normalize(helper)
    assert "else if(cnt<6'd2)" in helper and "if(cnt==6'd1) rem<=value;" in helper
    assert "acc<=0;bitv<=bitv>>2;cnt<=cnt+1'b1;" in helper
    assert 'norm_abs_valid<=0;norm_product_valid<=0;' in candidate
    assert 'norm_product_valid<=norm_abs_valid;' in candidate

    x = z3.BitVec('signed24_input_bits', 24)
    returned_unsigned24 = z3.If(z3.Extract(23, 23, x) == 1, -x, x)
    signed32 = z3.SignExt(8, x)
    mathematical_abs32 = z3.If(signed32 < 0, -signed32, signed32)
    proofs = [prove('all_signed24_patterns_abs_return_zero_extension_equals_mathematical_abs', [],
                    z3.ZeroExt(8, returned_unsigned24) != mathematical_abs32),
              prove('unsigned_abs24_maximum_is_2_pow23_including_minimum_signed_input', [],
                    z3.UGT(returned_unsigned24, z3.BitVecVal(1 << 23, 24)))]
    a, b, c = z3.Ints('abs_a abs_b abs_c')
    bound = 3*(1 << 46)
    domain = [v >= 0 for v in (a, b, c)]+[v <= 1 << 23 for v in (a, b, c)]
    proofs.append(prove('three_exact_squares_sum_maximum_3_pow46', domain,
                        a*a+b*b+c*c > bound))
    assert bound < 1 << 48 < 1 << 60
    assert abs24(0x800000) == 1 << 23
    assert 3*abs24(0x800000)**2 == bound

    value = z3.BitVec('original_accepted_value', 64)
    reference0 = (value, z3.BitVecVal(0, 64), z3.BitVecVal(1 << 62, 64), z3.BitVecVal(0, 6))
    reference1 = step_bv(reference0)
    reference2 = step_bv(reference1)
    delayed2 = (value, z3.BitVecVal(0, 64), z3.BitVecVal(1 << 58, 64), z3.BitVecVal(2, 6))
    proofs.append(prove('general_value_below_2_pow60_reference_first_two_steps_equal_delayed_capture_cnt2',
                        [z3.ULT(value, z3.BitVecVal(1 << 60, 64))],
                        z3.Or(*[a != b for a, b in zip(reference2, delayed2)])))
    proofs.append(prove('fusion_range_is_strictly_inside_first_two_empty_iteration_domain',
                        [z3.ULE(value, z3.BitVecVal(bound, 64))],
                        z3.Or(z3.UGE(value, z3.BitVecVal(1 << 62, 64)),
                              z3.UGE(value, z3.BitVecVal(1 << 60, 64)))))

    rng = random.Random(20261026)
    vectors = [(x, y, z) for x, y, z in [
        (0, 0, 0), (0x800000, 0x800000, 0x800000),
        (0x7fffff, 0x7fffff, 0x7fffff), (0x800001, 0xffffff, 1)]]
    vectors += [tuple(rng.randrange(1 << 24) for _ in range(3)) for _ in range(4096)]
    for triple in vectors:
        n = sum(abs24(v)**2 for v in triple)
        old_root, old_trace, old_done = model(n)
        new_root, new_trace, new_done = model(n, late2=True)
        assert n <= bound and old_root == new_root == math.isqrt(n)
        assert old_trace[1:] == new_trace[1:] and old_done == new_done == 33
    generic = [0, 1, 2, 3, 4, (1 << 48)-1, (1 << 60)-1]
    generic += [rng.randrange(1 << 60) for _ in range(4096)]
    for n in generic:
        old_root, old_trace, old_done = model(n)
        new_root, new_trace, new_done = model(n, late2=True)
        assert old_root == new_root == math.isqrt(n)
        assert old_trace[1:] == new_trace[1:] and old_done == new_done == 33

    # Public control alignment under interrupted work. Exhaust each possible
    # reset edge through completion, then restart with a fresh bounded request.
    reset_cases = 0
    control_edges = 0
    for interrupt_edge in range(36):
        old_state = new_state = (0, 0, 0, 0, 0, 0, 0)
        delayed_value = 0
        pending = {}
        for edge in range(85):
            reset = edge == interrupt_edge
            start = edge in (0, 1, 2, interrupt_edge+1, interrupt_edge+2, 74)
            n = (bound - edge*109) & ((1 << 48)-1)
            if reset:
                pending.clear()
            if edge in pending:
                delayed_value = pending.pop(edge)
            if start and not reset and not old_state[0]:
                pending[edge+2] = n
            old_state = engine_edge(old_state, reset=reset, start=start, value=n, late2=False)
            new_state = engine_edge(new_state, reset=reset, start=start, value=delayed_value, late2=True)
            assert (old_state[0],old_state[1],old_state[2],old_state[6]) == (new_state[0],new_state[1],new_state[2],new_state[6])
            if old_state[0] and old_state[2] >= 2:
                assert old_state == new_state
            if reset:
                assert old_state == new_state == (0,0,0,0,0,0,0)
            control_edges += 1
        reset_cases += 1

    counterexamples = []
    for n in (1 << 60, 1 << 62, MASK):
        old_root, _, old_done = model(n)
        new_root, _, new_done = model(n, late2=True)
        assert old_root == math.isqrt(n) and old_root != new_root and old_done == new_done
        counterexamples.append(dict(value=n, value_hex=hex(n), original_root=old_root,
                                    delayed2_root=new_root, done_cycle_after_accepted_start=old_done))
    result = dict(passed=True, recorded_at=datetime.now().astimezone().isoformat(),
        proof_layer='Independent mathematical/bit-vector state-transition proof, not compiled candidate RTL or physical timing acceptance',
        source_bindings=[meta(SOURCE/'imu_fusion.v'), meta(SOURCE/'imu_isqrt.v')],
        actual_abs24_source=abs_source, proofs=proofs,
        exact_sum_maximum=bound, maximum_attained_by_three_signed_minimum_inputs=True,
        fusion_sum_strictly_below_2_pow48=True, necessary_generic_bound_for_two_empty_steps='0 <= original accepted value < 2^60',
        state_after_two_iterations=dict(rem='original accepted value', acc=0, bitv=1 << 58, cnt=2, busy=1, done=0, root='unchanged prior root'),
        suffix_induction='Identical cnt2 state and identical subsequent deterministic transitions imply identical root and done for all remaining iterations.',
        late2_timing=dict(fusion_abs_capture_edge='E (original st4/st9)', product_capture_edge='E+1',
                         original_norm_square_capture_edge='E+2', sqrt_start_accepted_edge='E+1',
                         skip_cnt0_edge='E+2', capture_real_value_and_skip_cnt1_edge='E+3',
                         state_matches_original_cnt2_edge='E+3', done_and_root_edge='E+34',
                         done_edges_after_accepted_start=33),
        prerequisites=['The value sampled at cnt1 is exactly the reference value accepted two edges earlier.',
                       'Input magnitudes/products are unsigned zero-extended before multiplication and no truncation is introduced.',
                       'Reset/start/busy/cnt progression and transitions from cnt2 onward are preserved in the actual candidate.',
                       'The changed helper must be scoped to this bounded fusion caller; arbitrary 64-bit callers are not equivalent.'],
        signed24_triples_checked=len(vectors), additional_bounded_unsigned_values_checked=len(generic),
        reset_start_control_source_contracts_checked=True,
        reset_interruption_phases_checked=reset_cases, reset_start_control_edges_checked=control_edges,
        reset_argument='Both actual helpers have identical asynchronous reset assignments for every helper state/output. Fusion resets both valid stages and sqrt_start, so an interrupted pipeline cannot launch or complete stale work; after a fresh accepted start the same two-step relation re-establishes. Finite edge checks supplement this reset transition inspection, not a full fusion state-space proof.',
        arbitrary_64bit_counterexamples=counterexamples,
        candidate_expected_path=str(CANDIDATE), candidate_exists=CANDIDATE.exists(),
        candidate_RTL_equivalence_proved_by_this_script=False,
        physical_or_board_acceptance=False, synthesis_or_PNR_started=False,
        execution_policy=dict(priority=int(me.nice()), cpu_affinity=me.cpu_affinity()))
    if CANDIDATE.exists():
        result['candidate_metadata_observed_not_RTL_proof'] = meta(CANDIDATE)
    (OUT/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(passed=result['passed'], exact_sum_maximum=bound,
                         proof_statuses=[p['status'] for p in proofs],
                         finite_values_checked=len(vectors)+len(generic),
                         counterexamples=counterexamples), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
