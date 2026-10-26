"""Prepare, then read one completed SYN45 NAV/PID mapping. Never launch EDA.

Default is a source-bound plan only. --review requires natural SYN completion,
the original tool identity to have exited and the joint 2/4GiB read admission.
Generated LUT/FF suffixes are not correspondences. Register roles use their
actual connected semantic Q net, followed by full transition/INIT/clock proof.
Unknown roles and SAT/timeout results remain failures in the saved report.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import json
from pathlib import Path
import re
import traceback

import psutil

from prepare_baseline_fusion_retime import metadata, same_bytes
from prepare_nav_command_predecode import derive, PULSES
from plan_PNR53_PID_unity_bypass import ORIGINAL_TARGETS, UNITY_TARGETS
from synthesis_connectivity import read
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from review_stage3_slf128_memories import LIBRARY
from audit_routing_topology import primitive_directions
from prepare_no_sfp_clock_plan import construct_shared_logic
from prove_soc_console_remap import z3, literal
from retry_frozen_synthesis import memory_status

BASE = Path('F:/nav_integration_g2_20261023')
BASELINE = BASE/'SLF128_ring_stride32_syn44'
CANDIDATE = BASE/'SLF128_measured_timing_SYN45_prepared1'
BUILD = BASE/'SLF128_measured_timing_syn45'
NAV = 'nav/rtl/top/nav_subsystem.v'
CORE = 'control4500_core.v'
ACTION = ('control/policy_action_map.v', 'control/rl_policy_tick_v5.v', 'control/mem_policy.v')
COMMANDS = ((0x700c, 0), (0x700c, 1), (0x700c, 2), (0x2070, None), (0x3004, 0), (0x3004, 1))
RESET_PINS = ('RESET', 'CLEAR', 'SET', 'PRESET')
COMB = re.compile(r'LUT[1-6]|MUX2_LUT\w*|ALU|INV|IBUF|BUFG|GND|VCC')


def load(p): return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p, v): p.write_text(json.dumps(v, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
def files(root):
    ps = [Path(p).relative_to(root).as_posix() for p in load(root/'manifest.json')['source_files']]
    assert len(ps) == len(set(ps)) == 63
    return {r: root/r for r in ps}


def source_gate(a):
    old, new = files(a.baseline), files(a.candidate)
    assert old.keys() == new.keys()
    changed = {r for r in old if not same_bytes(old[r], new[r])}
    assert changed in ({NAV, CORE}, {NAV, CORE, *ACTION}), ('unexpected RTL changes', sorted(changed))
    nav, consumers = derive(old[NAV].read_text(encoding='utf8'))
    assert new[NAV].read_text(encoding='utf8') == nav
    core = old[CORE].read_text(encoding='utf8')
    assert core.count(ORIGINAL_TARGETS) == 1
    assert new[CORE].read_text(encoding='utf8') == core.replace(ORIGINAL_TARGETS, UNITY_TARGETS, 1)
    pid = a.candidate/'control/motor520_closed_loop.v'
    assert same_bytes(pid, a.baseline/pid.relative_to(a.candidate))
    text = pid.read_text(encoding='utf8')
    for s in ('TARGET_LIMIT_MMPS=2000', 'RAMP_STEP_MMPS=100',
              'if(!reset_n || !enable || (!CLOSED_LOOP_AT_ZERO && target_mmps==0))',
              'assign pwm=reset_n && enable && active_magnitude!=0 && counter<active_magnitude;'):
        assert s in text, ('PID expected source role changed', s)
    m = load(a.candidate/'manifest.json')
    assert m['NAV_command_predecode'] and m['PID_unity_reachable_bypass']
    assert bool(m.get('private_action_preview')) == (set(ACTION) <= changed)
    if set(ACTION) <= changed:
        # The root composition receipt proves this independent action overlay.
        receipt = load(a.candidate/'measured_timing_source_preparation.json')
        assert receipt['action_source_contract']['passed']
    assets = []
    for root in (a.baseline, a.candidate):
        assets.append({p.relative_to(root).as_posix(): p for p in root.rglob('*')
            if p.is_file() and 'impl' not in p.relative_to(root).parts
            and p.suffix.lower() in {'.mem', '.hex', '.mif', '.bin'}})
    assert assets[0].keys() == assets[1].keys() and len(assets[0]) == 104
    assert all(same_bytes(assets[0][n], p) for n, p in assets[1].items())
    proofs = [BASE/'NAV_predecode_source_relation_review2/result.json',
              BASE/'NAV_command_predecode_bus_probe2/result.json',
              BASE/'PID_unity_actual_ModelSim_review1/result.json']
    assert all(load(p)['passed'] for p in proofs)
    return dict(passed=True, exact_NAV_source_transform=True, exact_PID_unity_source_transform=True,
        all_unchanged_RTL_byte_identical=63-len(changed), changed_RTL=sorted(changed),
        all104_assets_byte_identical=True, original_53572B_FW=True,
        private_action_preview_selected=bool(m.get('private_action_preview')),
        private_action_actual_mapping_is_separate_gate=bool(m.get('private_action_preview')),
        six_source_consumer_substitutions=consumers,
        bindings=[metadata(p) for p in new.values()]+[metadata(p) for p in proofs])


def natural_complete(build):
    p = build/'job.json'; j = load(p)
    assert j['phase'] == 'synthesis_complete' and j['tool_exit'] == 0, ('SYN not naturally complete', j.get('phase'))
    pid, created = j.get('tool_pid'), j.get('tool_created_epoch')
    if pid and psutil.pid_exists(pid):
        proc = psutil.Process(pid)
        assert created is not None and abs(proc.create_time()-created) > .001, ('original tool still exists', pid, created)
    s = memory_status()
    assert s.avail_phys >= 2*1024**3 and s.avail_page >= 4*1024**3, ('joint graph admission', s.avail_phys, s.avail_page)
    return dict(job=metadata(p), original_tool_pid=pid, original_tool_created_epoch=created,
        original_tool_exited=True, physical_bytes=s.avail_phys, commit_bytes=s.avail_page)


class Reviewer:
    def __init__(self, a):
        self.a = a; self.checks = []; self.failures = []; self.notes = []
        self.av = a.baseline/'impl/gwsynthesis/g2_top.vg'; self.bv = a.vg or a.build/'impl/gwsynthesis/g2_top.vg'
        # Top arbiter and source-CSR qualifiers can be optimized across the SoC
        # and bridge boundaries. Include their actual drivers too: otherwise
        # con_read_needed and c_we, or s_addr and u_addr/c_addr, become unrelated
        # free inputs and produce an artificial acceptance counterexample.
        self.ca = read(self.av); self.cb = read(self.bv)
        self.ma = definitions(self.av); self.mb = definitions(self.bv)
        self.pa = parameter_reader(self.ma); self.pb = parameter_reader(self.mb)
        self.dirs = primitive_directions(LIBRARY)
        self.la, self.lb, self.boundaries = construct_shared_logic(self.ca, self.cb, self.pa, self.pb, self.dirs)
        self.cone_drivers = {}; self.cone_cache = {}
        self.qa = self.qs(self.ca); self.qb = self.qs(self.cb)
        self.pairs = {}; self.state_bindings = []
        # Same semantic connected Q role is allowed even if a generated suffix
        # changed, but only with type/effective INIT/clock equality. Its complete
        # actual transition is checked in the requested consumer partition.
        for q in self.qa.keys() & self.qb.keys():
            x, y = self.qa[q], self.qb[q]
            if self.compatible(x, y):
                sym = z3.Bool('semantic_Q:'+q)
                self.la.q_symbols[q] = self.lb.q_symbols[q] = sym
                self.pairs[x['name']] = y['name']
                self.state_bindings.append(dict(role=q, old=x['name'], new=y['name'],
                    before=self.shape(x, self.pa), after=self.shape(y, self.pb),
                    type_change_only_with_both_reset_controls_literal_GND=x['kind']!=y['kind']))
        for n in self.ca.keys() & self.cb.keys():
            x, y = self.ca[n], self.cb[n]
            if x['kind'].startswith('DFF') and self.shape(x, self.pa) == self.shape(y, self.pb):
                self.pairs.setdefault(n, n)
        self.la.memo = {'GND': z3.BoolVal(False), 'VCC': z3.BoolVal(True)}
        self.lb.memo = {'GND': z3.BoolVal(False), 'VCC': z3.BoolVal(True)}

    @staticmethod
    def qs(cells):
        d = {}
        for c in cells.values():
            if c['kind'].startswith('DFF'):
                q, = c['ports']['Q']; assert q not in d, ('duplicate Q role', q)
                d[q] = c
        return d

    @staticmethod
    def shape(c, p):
        return dict(kind=c['kind'], clock=c['ports'].get('CLK'),
            INIT=literal(p(c).get('INIT', "1'b1" if c['kind'] in ('DFFPE', 'DFFSE') else "1'b0")))

    def compatible(self, x, y):
        a, b = self.shape(x, self.pa), self.shape(y, self.pb)
        if a == b: return True
        # A tied-low asynchronous CLEAR and a tied-low synchronous RESET have
        # no event behavior to differ. Full CE/D/hold next-state is still
        # proved; this is not permission to replace a live asynchronous reset.
        return ({x['kind'],y['kind']} == {'DFFCE','DFFRE'}
            and a['clock'] == b['clock'] and a['INIT'] == b['INIT']
            and all(c['ports'].get('CLEAR' if c['kind']=='DFFCE' else 'RESET') == ['GND'] for c in (x,y)))

    def prove(self, label, mismatch, assumptions=()):
        s = z3.Solver(); s.set(timeout=self.a.timeout_ms)
        s.add(*assumptions); s.add(z3.simplify(mismatch)); result = s.check()
        row = dict(label=label, result=str(result).upper(), assumptions=[str(x) for x in assumptions])
        self.checks.append(row)
        if result != z3.unsat:
            row['counterexample_or_unknown'] = str(s.model()) if result == z3.sat else s.reason_unknown()
            self.failures.append(row)
            return False
        return True

    @staticmethod
    def vector(bits):
        xs = [z3.If(x, z3.BitVecVal(1, 1), z3.BitVecVal(0, 1)) for x in bits]
        return z3.Concat(*xs) if len(xs)>1 else xs[0]

    def role(self, side, q):
        qs = self.qa if side == 'old' else self.qb
        assert q in qs, ('semantic Q not found; inspect actual driver, do not guess suffix', side, q)
        return qs[q]

    def support(self, cells, nets):
        """Exact structural cone up to actual FF/opaque/external boundaries."""
        key = (id(cells), tuple(nets))
        if key in self.cone_cache: return self.cone_cache[key]
        if id(cells) not in self.cone_drivers:
            drivers = defaultdict(list)
            for c in cells.values():
                for pin, ns in c['ports'].items():
                    if self.dirs[c['kind']].get(pin) == 'output':
                        for n in ns: drivers[n].append(c)
            self.cone_drivers[id(cells)] = drivers
        drivers = self.cone_drivers[id(cells)]
        seen, primitives, frontier = set(), set(), set()
        def visit(n):
            if n in seen or n in ('VCC', 'GND'): return
            seen.add(n); ds = drivers.get(n, [])
            if not ds: frontier.add(n); return
            assert len(ds) == 1, ('nonunique cone driver', n)
            c = ds[0]
            if not COMB.fullmatch(c['kind']): frontier.add(n); return
            primitives.add(c['name'])
            for p, ns in c['ports'].items():
                if self.dirs[c['kind']].get(p) == 'input':
                    for x in ns: visit(x)
        for n in nets: visit(n)
        result = dict(primitives=sorted(primitives), frontier=sorted(frontier),
            primitive_kinds=dict(Counter(cells[n]['kind'] for n in primitives)))
        self.cone_cache[key] = result
        return result

    def nav(self):
        start = len(self.failures); details = []
        wr = self.role('old', 'u_nav/rf_wr')
        aa = [self.role('old', f'u_nav/a_q[{i}]') for i in reversed(range(2, 16))]
        dd = {i: self.role('old', f'u_nav/wd_q[{i}]') for i in range(3)}
        wr_next = self.la.next_state(wr)
        addr_next = self.vector([self.la.next_state(c) for c in aa]+[z3.BoolVal(False)]*2)
        addr_now = self.vector([self.la.one(c, 'Q') for c in aa]+[z3.BoolVal(False)]*2)
        new_pulses = set()
        for name, (code, bit) in zip(PULSES, COMMANDS):
            c = self.role('new', 'u_nav/'+name); new_pulses.add(c['name'])
            assert self.shape(c, self.pb)['INIT'] == 0
            assert c['ports']['CLK'] == wr['ports']['CLK'] == ['clk50_d']
            wanted = z3.And(wr_next, addr_next == code)
            relation = z3.And(self.la.one(wr, 'Q'), addr_now == code)
            if bit is not None:
                wanted = z3.And(wanted, self.la.next_state(dd[bit]))
                relation = z3.And(relation, self.la.one(dd[bit], 'Q'))
            ok = self.prove('NAV '+name+' full actual next-state == original accepted command', self.lb.next_state(c) != wanted)
            details.append(dict(role=name, actual=c, parameters=self.pb(c),
                expected_command_hex=f'{code:04x}', payload_bit=bit, full_transition_passed=ok,
                D_cone=self.support(self.cb, c['ports']['D']),
                reset_pins={p:c['ports'][p] for p in RESET_PINS if p in c['ports']}))
            # This relation, established by the above one-step proof and cold
            # reset0, is the only new state invariant used at old consumers.
            self.lb.memo[c['ports']['Q'][0]] = relation
        matched, unmatched, movements = [], [], []
        pulse_qs = {'u_nav/'+n for n in PULSES}
        old_address_qs = {'u_nav/a_q['+str(i)+']' for i in range(2, 16)}
        consumer_prefixes = ('u_nav/u_match/', 'u_nav/u_map/', 'u_nav/u_xt/', 'u_nav/u_support/')
        for n, x in self.ca.items():
            if not x['kind'].startswith('DFF') or not n.startswith(consumer_prefixes): continue
            if n not in self.pairs:
                unmatched.append(dict(old=n, Q=x['ports']['Q'])); continue
            y = self.cb[self.pairs[n]]
            ok = self.prove('NAV retained consumer transition '+n, self.la.next_state(x) != self.lb.next_state(y))
            matched.append(dict(old=n, new=y['name'], passed=ok))
            for pin in ('CE', *RESET_PINS):
                if pin not in x['ports'] or pin not in y['ports']: continue
                u, v = self.support(self.ca, x['ports'][pin]), self.support(self.cb, y['ports'][pin])
                old_hit = sorted(old_address_qs & set(u['frontier']))
                new_hit = sorted(pulse_qs & set(v['frontier']))
                if old_hit or new_hit:
                    movements.append(dict(old=n, new=y['name'], pin=pin, old_address_Qs=old_hit,
                        new_address_Qs=sorted(old_address_qs & set(v['frontier'])), new_pulse_Qs=new_hit,
                        old_logic_cells=len(u['primitives']), new_logic_cells=len(v['primitives'])))
        removed = [r for r in movements if r['old_address_Qs'] and not r['new_address_Qs'] and r['new_pulse_Qs']]
        # Do not demand every NAV CE lose its address: configuration writes and
        # other original commands are still legitimately decoded downstream.
        all_reached = set().union(*(set(r['new_pulse_Qs']) for r in movements)) if movements else set()
        functional = len(self.failures) == start and len(details) == 6
        return dict(mapped_NAV_pulse_contract_passed=functional,
            NAV_decode_removed_from_selected_consumer_CE=bool(removed) and all_reached == pulse_qs,
            actual_six_registers=details, consumer_register_transitions=matched,
            unmatched_consumer_registers=unmatched, control_pin_frontier_changes=movements,
            address_removed_endpoints=removed, six_pulses_reach_actual_consumer_controls=sorted(all_reached),
            full_NAV_sequential_equivalence=False,
            original_other_command_or_config_address_decode_is_allowed=True,
            first_edge_and_cold_start_scope='The new pulse/old RF-command relation is established after the first shared edge, with actual INIT0 and original reset contract; arbitrary inconsistent pre-first-edge extra FF states are not accepted.')

    def pid(self):
        start = len(self.failures); ranges = []; model_bindings = []; sides = []
        # Cut the public model wheel register outputs at their real Q drivers.
        # Missing unused logical bits are not invented as physical aliases.
        for side in ('left', 'right'):
            model = z3.BitVec('reachable_model_'+side, 32)
            ranges += [model >= -600, model <= 600]
            bindings = []
            for tag, qs, logic in (('old', self.qa, self.la), ('new', self.qb, self.lb)):
                found = []
                for q, c in qs.items():
                    m = re.fullmatch(r'u_control/model_'+side+r'_mmps\[(\d+)\]', q)
                    if not m: continue
                    bit = int(m[1]); assert 0 <= bit < 32
                    assert c['name'].startswith('u_control/model_thread/'), ('model output has new writer', c)
                    assert c['ports']['CLK'] == ['clk50_d']
                    logic.memo[q] = z3.Extract(bit, bit, model) == 1
                    found.append(dict(bit=bit, actual=c, parameters=(self.pa if tag=='old' else self.pb)(c)))
                assert {r['bit'] for r in found} >= set(range(11)), ('missing low model roles', tag, side)
                bindings.append(dict(side=tag, actual_present_model_bits=found,
                    no_unobserved_physical_high_bit_alias_guessed=True))
            model_bindings.append(dict(wheel=side, bindings=bindings))
            oldref = [self.role('old', f'u_control/{side}_pid_reference[{i}]') for i in reversed(range(32))]
            newref = [self.role('new', f'u_control/{side}_pid_reference[{i}]') for i in reversed(range(32))]
            reference = self.vector([self.lb.one(c, 'Q') for c in newref])
            local_ranges = ranges+[reference >= -600, reference <= 600]
            wanted = z3.If(model > reference+100, reference+100,
                z3.If(model < reference-100, reference-100, model))
            actual = self.vector([self.lb.one(c, 'D') for c in newref])
            transition_ok = self.prove(side+' unity PID full ramp-register next-state',
                z3.Or(*[self.la.next_state(x) != self.lb.next_state(y) for x, y in zip(oldref, newref)]), ranges)
            accepted_differences = []
            for i, c in zip(reversed(range(32)), newref):
                reset = next((self.lb.one(c,p) for p in RESET_PINS if p in c['ports']), z3.BoolVal(False))
                capture = z3.And(self.lb.one(c,'CE','VCC'), z3.Not(reset))
                accepted_differences.append(z3.And(capture, self.lb.one(c,'D') != (z3.Extract(i,i,wanted)==1)))
            # D on a disabled/reset edge is a synthesis don't-care. The exact
            # public capture condition is mandatory, not an arbitrary weakening
            # of the model values. CE/reset and original next-state are checked
            # independently above and same-cycle IR stop below.
            d_ok = self.prove(side+' actual accepted ramp D == direct reachable model next_reference',
                z3.Or(*accepted_differences), local_ranges)
            evaluations = None
            if not d_ok:
                s = z3.Solver(); s.set(timeout=self.a.timeout_ms); s.add(*local_ranges,z3.Or(*accepted_differences))
                if s.check() == z3.sat:
                    m=s.model(); evaluations={n:str(m.eval(v,model_completion=True)) for n,v in
                        [('model',model),('reference',reference),('actual_D',actual),('expected_D',wanted)]}
            oldcone = self.support(self.ca, [n for c in oldref for n in c['ports']['D']])
            newcone = self.support(self.cb, [n for c in newref for n in c['ports']['D']])
            ir = lambda n: bool(re.search(r'(?:ir_|remote_thread/|telemetry_thread/.*emergency)', n))
            old_ir = [n for n in oldcone['frontier'] if ir(n)]
            new_ir = [n for n in newcone['frontier'] if ir(n)]
            sides.append(dict(wheel=side, actual32_reference_FF_shapes=[self.shape(c,self.pb) for c in newref],
                complete_ramp_transition_equal=transition_ok, actual_direct_model_ramp_D_proved=d_ok,
                actual_D_capture_condition='per actual bit CE && !RESET/CLEAR/SET/PRESET; reset/set checked separately',
                failed_D_evaluations=evaluations,
                old_D_cone=oldcone, new_D_cone=newcone, old_IR_state_frontier=old_ir, new_IR_state_frontier=new_ir,
                target_amplification_selection_removed=d_ok and bool(old_ir) and not new_ir))
        # Actual same-cycle stop, not absence of IR from the entire PID cone.
        ir_old = self.role('old', 'u_control/ir_emergency_stop')
        ir_new = self.role('new', 'u_control/ir_emergency_stop')
        assert self.shape(ir_old,self.pa) == self.shape(ir_new,self.pb)
        ir_preserved = self.prove('IR latch complete original next-state', self.la.next_state(ir_old) != self.lb.next_state(ir_new))
        ir_value = self.lb.one(ir_new,'Q')
        stops = []
        for side in ('left','right'):
            for i in range(32):
                c = self.role('new', f'u_control/{side}_pid_reference[{i}]')
                reset = next((self.lb.one(c,p) for p in RESET_PINS if p in c['ports']), None)
                assert reset is not None
                stops.append(self.prove(side+f' PID reference bit{i} same-cycle IR reset', z3.Not(reset), [ir_value]))
        pwm = []
        for n in ('M1_PWM_d','M2_PWM_d','M1_DIR1_d','M1_DIR2_d','M2_DIR1_d','M2_DIR2_d'):
            assert self.lb.drivers.get(n), ('real motor output missing', n)
            pwm.append(self.prove(n+' IR latch gates real motor output without new state', self.lb.expr(n), [ir_value]))
        # The unchanged IR and PID/PWM source plus this mapped public ramp and
        # direct output-stop proof does not prove every PID private DSP state.
        functional = len(self.failures) == start and all(r['complete_ramp_transition_equal'] and r['actual_direct_model_ramp_D_proved'] for r in sides)
        return dict(mapped_PID_reachable_contract_passed=functional,
            PID_target_amplification_removed=all(r['target_amplification_selection_removed'] for r in sides),
            IR_immediate_stop_preserved=ir_preserved and all(stops) and all(pwm),
            wheels=sides, actual_public_model_register_boundaries=model_bindings,
            model_range_assumption='Both original/public action-map outputs are reset0, hold, or exact clip(action)*+-600 rounded; known reachable values stay[-600,600]. The independently selected action-preview mapping must close this same public register invariant.',
            target_limit2000_ramp_step100_bound_to_unchanged_PID_source=True,
            ramp_range_invariant='Original reference register resets0 and on every accepted sample moves by at most100 toward a model target in[-600,600]; hold/reset/fault branches cannot leave[-600,600]. The direct-D check uses this invariant; old/new complete ramp next-state equality is also checked without the ramp bound.',
            whole_PID_private_state_or_DSP_sequential_equivalence=False,
            arbitrary32bit_model_or_unknown_output_identity_claimed=False,
            direct_stop_scope='Existing IR latch asserted -> original same-cycle ramp reset inputs and combinational actual PWM/DIR outputs low; no new stop latency or clock exception.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,default=BASELINE); p.add_argument('--candidate',type=Path,default=CANDIDATE)
    p.add_argument('--build',type=Path,default=BUILD); p.add_argument('--vg',type=Path)
    p.add_argument('--out',type=Path,required=True); p.add_argument('--review',action='store_true')
    p.add_argument('--timeout-ms',type=int,default=20000)
    a = p.parse_args(); assert not a.out.exists(); a.out.mkdir(parents=True)
    result = dict(at=datetime.now().astimezone().isoformat(), stage='prepare_only',
        baseline=str(a.baseline), candidate=str(a.candidate), build=str(a.build),
        EDA_started=False, RTL_changed=False, actual_graph_parsed=False, deployment_accepted=False,
        mapped_NAV_pulse_contract_passed=False, NAV_decode_removed_from_selected_consumer_CE=False,
        mapped_PID_reachable_contract_passed=False, PID_target_amplification_removed=False,
        IR_immediate_stop_preserved=False, combined_actual_goal_passed=False,
        actual_mapping_pending=True, physical_timing_or_routing_accepted=False)
    try:
        result['source_gate'] = source_gate(a)
        result['plan'] = dict(
            actual_NAV_roles=['u_nav/'+n for n in PULSES],
            acceptance_reference='Original actual rf_wr/a_q[15:2]/wd_q[2:0] complete next-state;6 new actualFF full next-state,INIT0,originalclk50; no generated LUT name matching.',
            consumer_goal='Registered pulse frontier reaches real match/map/XT control pins; count original address FF dependencies removed on corresponding pins. Other configuration/address decodes remain legal.',
            PID_roles='Public model left/right FF output bits -> left/right_pid_reference[31:0] actual next_reference D plus CE/reset; direct signed32 model/ramp equation with original step100.',
            stop_goal='Actual IR latch complete transition; IR=1 forces same-cycle reference reset and actual PWM/DIR outputs low. IR must remain in enable/stop cones.',
            mandatory_manual_differences=['Missing/merged/inverted semantic Q roles: inspect actual connected drivers; never whitelist an auto suffix.',
                'Any new FF kind/INIT/CLK/reset polarity or actual SAT/unknown; preserve first failure.',
                'IR dependencies remaining in target-only ramp D vs legitimate dependencies in enable/reset.',
                'Unmatched consumer FFs or opaque memory/DSP boundaries are not full sequential equivalence.',
                'Action preview actual six DSP boundary/public output invariant is a separate gate, as are all storage INIT/CDC and physical STA.'],
            run=f'D:/Anaconda/python.exe -X utf8 integration/g2/review_measured_NAV_PID_syn45.py --review --out {BASE}/SYN45_NAV_PID_actual_review1')
        if a.review:
            result['admission'] = natural_complete(a.build)
            before = metadata(a.vg or a.build/'impl/gwsynthesis/g2_top.vg')
            r = Reviewer(a); result['actual_graph_parsed'] = True
            result['state_boundary_audit'] = r.boundaries
            result['NAV'] = r.nav(); result['PID'] = r.pid()
            for key in ('mapped_NAV_pulse_contract_passed','NAV_decode_removed_from_selected_consumer_CE'):
                result[key] = result['NAV'][key]
            for key in ('mapped_PID_reachable_contract_passed','PID_target_amplification_removed','IR_immediate_stop_preserved'):
                result[key] = result['PID'][key]
            result.update(stage='actual_readonly_review_complete', actual_mapping_pending=False,
                checks=r.checks, failures=r.failures, actual_input=before,
                actual_partition_counts_before=dict(Counter(c['kind'] for c in r.ca.values())),
                actual_partition_counts_after=dict(Counter(c['kind'] for c in r.cb.values())))
            assert before == metadata(a.vg or a.build/'impl/gwsynthesis/g2_top.vg')
            save(a.out/'actual_semantic_state_correspondences.json', r.state_bindings)
            result['combined_actual_goal_passed'] = all(result[k] for k in (
                'mapped_NAV_pulse_contract_passed','NAV_decode_removed_from_selected_consumer_CE',
                'mapped_PID_reachable_contract_passed','PID_target_amplification_removed','IR_immediate_stop_preserved'))
    except Exception:
        result['error'] = traceback.format_exc()
        result['stage'] = 'actual_or_source_review_blocked_preserved'
        if 'r' in locals():
            result.update(checks=r.checks, failures=r.failures)
    save(a.out/'result.json', result)
    print(json.dumps({k:result[k] for k in ('stage','actual_graph_parsed','combined_actual_goal_passed')},ensure_ascii=False))
    if 'error' in result: print(result['error'])
    return 0 if 'error' not in result and (not a.review or result['combined_actual_goal_passed']) else 1


if __name__ == '__main__': raise SystemExit(main())
