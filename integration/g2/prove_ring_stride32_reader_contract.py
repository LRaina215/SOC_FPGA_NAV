"""Independent finite-address induction with universally arbitrary RAM words.

No RTL frontend or simulator is invoked. Exact actual source anchors bind a
small mathematical transition model. RAM data are opaque symbolic terms, so
word equality is parametric in every value, X/Z pattern and memory aliasing.
Only legal known-valued pointer/control states and the paired caller contract
are covered; this is not a whole-RTL formal-equivalence claim.
"""
from pathlib import Path
from datetime import datetime
from itertools import product
import argparse
import json
import re
import time

BASE = Path('F:/nav_integration_g2_20261023')
OLD = BASE / 'SLF128_storage_safeFW_syn43'
NEW = BASE / 'SLF128_ring_stride32_prepared1'
FILES = ('control/rl_observation_v5.v', 'control/rl_policy_tick_v5.v', 'control/mem_policy.v')


def meta(path):
    s = path.stat()
    return dict(path=str(path.resolve()), bytes=s.st_size, mtime_ns=s.st_mtime_ns)


def normalized(text):
    text = re.sub(r'//[^\n]*|/\*.*?\*/', '', text, flags=re.S)
    return re.sub(r'\s+', '', text)


def line(path, anchor):
    t = path.read_text(encoding='utf8')
    assert t.count(anchor) == 1, (str(path), anchor)
    return dict(path=str(path), line=t[:t.index(anchor)].count('\n') + 1, anchor=anchor)


def bind_sources():
    old = {p: (OLD / p).read_text(encoding='utf8') for p in FILES}
    new = {p: (NEW / p).read_text(encoding='utf8') for p in FILES}
    op, np = old[FILES[0]], new[FILES[0]]
    # Shared producer arithmetic, clipping function, reset and COMMIT are exact.
    port = '    input wire frame_read_peek_first,frame_read_advance,\n'
    assert np.count(port) == 1
    prefix = '    generate if (OBS_RING_STORAGE != 0)'
    assert op[:op.index(prefix)] == np[:np.index(prefix)].replace(port, '')
    writer = '        always @(posedge clk) begin\n            if(rst_n && lane_write)'
    assert op.count(writer) == np.count(writer) == 1
    assert op[op.index(writer):] == np[np.index(writer):]
    # The complete original sequential scheduler/diagnostic tail is unchanged.
    sequential = '    always @(posedge clk or negedge rst_n) begin'
    assert old[FILES[1]][old[FILES[1]].index(sequential):] == new[FILES[1]][new[FILES[1]].index(sequential):]
    # Owner only adds the two direct narrow connections, not a new enable.
    owner = new[FILES[2]]
    wire = '    wire observation_peek_first,observation_read_advance;\n'
    connection = '\n        .frame_read_peek_first(observation_peek_first),.frame_read_advance(observation_read_advance),'
    assert owner.count(wire) == 1 and owner.count(connection) == 2
    assert owner.replace(wire, '').replace(connection, '') == old[FILES[2]]
    a, b = normalized(op), normalized(np)
    assert "reg signed" not in b  # whitespace normalization actually used below
    assert "frame_mem[0:167]" in a and "frame_mem[0:191]" in b
    for term in ("write_address=work_slot*8'd28+lane;",
                 "read_address=read_slot*8'd28+read_lane;",
                 "read_lane=frame_read_index-segment*8'd28;",
                 "frame_read_data=frame_read_index>=168?24'sd0:segment==0?clip_current(raw_word):raw_word;"):
        assert term in a, term
    assert 'read_slot=segment==0?r4:segment==1?r0:segment==2?r1:segment==3?r2:segment==4?r3:r4;' in a
    assert "segment=frame_read_index<28?0:frame_read_index<56?1:frame_read_index<84?2:frame_read_index<112?3:frame_read_index<140?4:5;" in a
    assert "write_address={work_slot,lane};" in b
    assert "read_address=frame_read_peek_first?{live4,5'd0}:{read_slot,read_lane};" in b
    assert "frame_read_data=frame_read_peek_first?clip_current(raw_word):read_segment<6?(read_segment==0?clip_current(raw_word):raw_word):24'sd0;" in b
    cursor = '''always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin read_segment<=3'd6;read_lane<=0;read_slot<=0;
        end else if(frame_freeze && initialized) begin
          read_segment<=0;read_lane<=1;read_slot<=live4;
        end else if(frame_read_advance) begin
          if(read_segment<6) begin
            if(read_lane==27) begin read_lane<=0;read_segment<=read_segment+1'b1;
              case(read_segment)
                0:read_slot<=frozen0;1:read_slot<=frozen1;2:read_slot<=frozen2;
                3:read_slot<=frozen3;4:read_slot<=frozen4;
                default:read_slot<=read_slot;
              endcase
            end else read_lane<=read_lane+1'b1;
          end
        end
      end'''
    assert normalized(cursor) in b, 'actual cursor transition differs from the mathematical binding'
    schedule = dict((int(x), int(y)) for x, y in re.findall(r'([0-4]):read_slot<=frozen([0-4]);', b))
    assert schedule == {i: i for i in range(5)}
    policy = normalized(new[FILES[1]])
    advance = '''always @* begin
      frame_read_advance=1'b0;
      if(!rst_n) begin end
      else if(!enable) begin end
      else if(!sensor_ok || fault!=0) begin end
      else begin
        if(!(tick_count==PERIOD_CLKS-1 && state!=IDLE)) begin
          case(state)
            LOAD:if(OBS_RING_STORAGE!=0) begin
              if(load_index!=167) frame_read_advance=1'b1;
            end
          endcase
        end
      end
    end'''
    assert normalized(advance) in policy
    assert 'frame_read_peek_first=OBS_RING_STORAGE!=0&&state==SNAPSHOT;' in policy
    assert 'frame_freeze=OBS_RING_STORAGE!=0&&state==SNAPSHOT&&frame_valid&&run_ok&&!deadline_missed;' in policy
    assert policy.count('if(load_index!=167)snapshot_word<=frame_read_data;') == 1
    assert 'valid<=1;initialized<=1;state<=IDLE;' in normalized(op)
    # Full source inventory: no unnoticed fourth RTL change.
    def paths(root):
        m = json.loads((root / 'manifest.json').read_text(encoding='utf8'))
        result = {Path(x).resolve().relative_to(root.resolve()).as_posix(): Path(x) for x in m['source_files']}
        assert len(result) == 63
        return result
    old_inputs, new_inputs = paths(OLD), paths(NEW)
    assert old_inputs.keys() == new_inputs.keys()
    changed = sorted(p for p in old_inputs if old_inputs[p].read_bytes() != new_inputs[p].read_bytes())
    assert changed == sorted(FILES)
    anchors = [line(NEW / FILES[0], 'wire [7:0] read_address='),
               line(NEW / FILES[0], 'read_segment<=0;read_lane<=1;read_slot<=live4;'),
               line(NEW / FILES[0], 'case(read_segment)'),
               line(NEW / FILES[1], 'assign frame_read_peek_first='),
               line(NEW / FILES[1], "if(load_index!=167) snapshot_word<=frame_read_data;")]
    bindings = [meta(root / rel) for root in (OLD, NEW) for rel in FILES]
    bindings += [meta(root / 'manifest.json') for root in (OLD, NEW)]
    return schedule, bindings, anchors, changed


ORDER = (4, 0, 1, 2, 3, 4)


def raw(old_address):
    # Free variable for the corresponding arbitrary old RAM word. Equality
    # here means identical expressions for ALL assignments, not a test value.
    return ('arbitrary_RAM_word', old_address)


def old_word(index, pointers):
    if index >= 168:
        return ('zero',)
    segment, lane = divmod(index, 28)
    term = raw(pointers[ORDER[segment]] * 28 + lane)
    return ('identical_clip_function', term) if segment == 0 else term


def new_word(cursor, peek, live4):
    segment, lane, slot = cursor
    if peek:
        slot, lane = live4, 0
    if 0 <= slot < 6 and 0 <= lane < 28:
        term = raw(slot * 28 + lane)  # assumed memory-bijection invariant
    else:
        term = ('independent_new_hole_word', slot * 32 + lane)
    if peek or segment == 0:
        return ('identical_clip_function', term)
    return term if segment < 6 else ('zero',)


def step(cursor, pointers, schedule, reset=False, freeze=False, initialized=True, advance=False):
    if reset:
        return (6, 0, 0)
    if freeze and initialized:
        return (0, 1, pointers[4])
    segment, lane, slot = cursor
    if advance and segment < 6:
        if lane == 27:
            return segment + 1, 0, pointers[schedule[segment]] if segment in schedule else slot
        return segment, lane + 1, slot
    return cursor


def prove(out):
    assert not out.exists()
    began = time.monotonic()
    schedule, bindings, anchors, changed = bind_sources()
    count = dict(pointer_tuples=0, first_word=0, prefetch_words=0, induction_steps=0, final_sentinel=0,
                 reset_states=0, hold_states=0, freeze_overrides=0, qualification_cases=0,
                 write_read_pairs=0, padding_nonalias_pairs=0)
    boundary = {k: 0 for k in (27, 55, 83, 111, 139, 167)}
    # Base and induction step, with frozen pointers fully arbitrary/repeated.
    for pointers in product(range(6), repeat=5):
        count['pointer_tuples'] += 1
        assert old_word(0, pointers) == new_word((6, 0, 0), True, pointers[4])
        count['first_word'] += 1
        cursor = step((6, 0, 0), pointers, schedule, freeze=True)
        assert cursor == (0, 1, pointers[4])
        for k in range(1, 168):
            segment, lane = divmod(k, 28)
            assert cursor == (segment, lane, pointers[ORDER[segment]])
            assert old_word(k, pointers) == new_word(cursor, False, pointers[4])
            assert 0 <= cursor[1] < 28 and 0 <= cursor[2] < 6
            count['prefetch_words'] += 1
            if k in boundary:
                boundary[k] += 1
            cursor = step(cursor, pointers, schedule, advance=True)
            if k < 167:
                s2, l2 = divmod(k + 1, 28)
                assert cursor == (s2, l2, pointers[ORDER[s2]])
                count['induction_steps'] += 1
        assert cursor == (6, 0, pointers[4])
        assert step(cursor, pointers, schedule, advance=False) == cursor
        assert old_word(168, pointers) == new_word(cursor, False, pointers[4]) == ('zero',)
        count['final_sentinel'] += 1
    # Hold/reset for every bit pattern of the11 explicit cursor registers.
    for cursor in product(range(8), range(32), range(8)):
        assert step(cursor, (0,) * 5, schedule) == cursor
        assert step(cursor, (0,) * 5, schedule, reset=True, freeze=True, advance=True) == (6, 0, 0)
        count['hold_states'] += 1
        count['reset_states'] += 1
        for live4 in range(6):
            pointers = (0, 0, 0, 0, live4)
            assert step(cursor, pointers, schedule, freeze=True, advance=True) == (0, 1, live4)
            count['freeze_overrides'] += 1
    # Independently read the old procedural priority as a decision tree, then
    # compare its capture permission with the new canonical advance predicate.
    for rst, enable, sensor, fault, last_tick, state, index in product(
            range(2), range(2), range(2), range(8), range(2), range(5), range(168)):
        old_accept = False
        if rst:
            if not enable:
                pass
            elif not sensor or fault:
                pass
            elif last_tick and state != 0:
                pass
            elif state == 2:
                old_accept = index != 167
        new_advance = bool(rst and enable and sensor and fault == 0 and
                           not (last_tick and state != 0) and state == 2 and index != 167)
        assert bool(old_accept) == new_advance
        count['qualification_cases'] += 1
    # A single arbitrary write preserves the memory relation at ANY read site.
    # Symbolic W is independent of every initial RAM variable. Numeric address
    # equality selects W in both memories, otherwise both keep the same atom.
    sites = [(s, lane) for s in range(6) for lane in range(28)]
    for ws, wl in sites:
        old_w, new_w = ws * 28 + wl, ws * 32 + wl
        for rs, rl in sites:
            old_r, new_r = rs * 28 + rl, rs * 32 + rl
            assert (old_w == old_r) == (new_w == new_r)
            old_term = ('arbitrary_write_W',) if old_w == old_r else raw(old_r)
            new_term = ('arbitrary_write_W',) if new_w == new_r else raw(old_r)
            assert old_term == new_term
            count['write_read_pairs'] += 1
        for hs, hl in product(range(6), range(28, 32)):
            assert new_w != hs * 32 + hl
            count['padding_nonalias_pairs'] += 1
    # Repetition/aliasing cannot turn history into current: clipping depends
    # on logical segment, never on equality of physical slot IDs.
    repeated = (3,) * 5
    assert old_word(0, repeated)[0] == 'identical_clip_function'
    assert old_word(140, repeated) == raw(3 * 28)
    clip = lambda v: min(409600, max(-409600, v))
    assert clip(500000) == 409600 and 500000 != clip(500000)
    for row in bindings:
        assert meta(Path(row['path'])) == row, ('input changed during proof', row)
    result = dict(passed=True, phase='independent_reader_contract_induction_complete',
        recorded_at=datetime.now().astimezone().isoformat(), elapsed_s=time.monotonic() - began,
        source=str(OLD), candidate=str(NEW), actual_changed_RTL=changed, bindings=bindings, actual_source_anchors=anchors,
        method='All finite legal pointer/address states plus exact opaque symbolic-term equality; no sampled RAM data, noZ3 dependency, noRTL frontend',
        counts=count, total_accepted_word_equalities=count['first_word'] + count['prefetch_words'],
        all_six_boundary_counts=boundary,
        source_text_checks=dict(producer_arithmetic_clipping_reset_commit_exact=True,
            original_RAM_writer_and_all_pointer_COMMIT_freeze_logic_exact=True,
            default_flat_branch_exact=True, complete_policy_sequential_tail_exact=True,
            owner_only_two_direct_narrow_signals=True, all_other60_RTL_byte_identical=True),
        invariant='Before accepted prefetch logicalk1..167: segment=floor(k/28), lane=k%28, slot=frozen[4,0,1,2,3,4][segment]. Atfreeze frozen=old live and cursor=(0,1,old live4).',
        memory_relation='For slot0..5/lane0..27, N[32*slot+lane]=O[28*slot+lane]. Every old word is an independent opaque value; new holes unconstrained. A common arbitrary write preserves this relation, hence induction over any finite producer sequence.',
        all_value_argument='Opaque RAM terms may be assigned ANY24bit value or4-state pattern. Identical source clip function is kept as an uninterpreted deterministic operation; matching terms prove equality for every assignment, including repeated slots and unknown contents. No RAM is read, initialized or forced by this proof.',
        accepted_freeze_precondition='Paired policy stateSNAPSHOT/frame_valid/run_ok/no-deadline; original observer valid is assigned with initialized in the exact unchanged COMMIT branch. Known reset/state/caller contract assumed.',
        LOAD167='Original if(load_index!=167) capture remains exact; new advance0. Segment6 sentinel is not consumed; both private data functions return0 at the old logical168 sentinel.',
        stop_and_reset='With no reset, noacceptedfreeze and noadvance every11bit cursor state holds. When disable/STOP actually asserts observer reset, reset wins and sets6/0/0; it is incorrect to require allSTOP types to leave cursorFF unchanged.',
        clip_alias_witness=dict(all_five_pointers=3, newest_raw=500000, current=409600, newest_history=500000),
        no_simulation_started=True, no_EDA_started=True, RTL_written=False, whole_RTL_formal_equivalence=False,
        limits=['Manual exact-source semantic binding; not parsing/elaborating completeVerilog',
                'Known legal control/pointer values, initialized/caller contract; arbitraryX/Z control/reset not universally quantified',
                'Internal speculative data outside accepted paired transactions may differ',
                'No wholeG2 reachability/timing/CDC/IO/actuator or physical stop claim'])
    out.mkdir(parents=True)
    (out / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    return result


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args()
    r = prove(a.out)
    print(json.dumps(dict(passed=r['passed'], counts=r['counts'], total_accepted_word_equalities=r['total_accepted_word_equalities'], elapsed_s=r['elapsed_s']), ensure_ascii=False))
