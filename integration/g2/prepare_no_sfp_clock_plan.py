"""Locate routing-request signals in already parsed real PHY-off netlists.

This module never reads a VG or starts a tool. The joint reviewer supplies the
two actual cell graphs and parameter readers. Correspondences come from real
primitive instances or their surviving sink pins, never generated net names.
Boolean checks use corresponding actual state/memory boundaries; this is an
implementation preflight, not complete sequential or physical signoff.
"""
from collections import Counter
import re

from prove_soc_console_remap import Logic, literal, z3

REQUEST = re.compile(r'^CLOCK_LOC "([^"]+)" (.+);\s*$', re.M)


def construct_shared_logic(before, after, oldparams, newparams, directions):
    qa, qb, sa, sb = {}, {}, {}, {}
    matched_ff = []
    def initial(cell, params):
        return literal(params(cell).get('INIT',
            "1'b1" if cell['kind'] in ('DFFPE', 'DFFSE') else "1'b0"))
    for name in before.keys() | after.keys():
        a, b = before.get(name), after.get(name)
        if a and a['kind'].startswith('DFF'):
            qa[a['ports']['Q'][0]] = z3.Bool('old_state:'+name)
        if b and b['kind'].startswith('DFF'):
            qb[b['ports']['Q'][0]] = z3.Bool('new_state:'+name)
        if (a and b and a['kind'].startswith('DFF') and a['kind'] == b['kind']
                and a['ports'].get('CLK') == b['ports'].get('CLK')
                and initial(a, oldparams) == initial(b, newparams)):
            shared = z3.Bool('actual_state_instance:'+name)
            qa[a['ports']['Q'][0]] = qb[b['ports']['Q'][0]] = shared
            matched_ff.append(name)
    combinational = lambda kind: bool(re.fullmatch(r'LUT[1-6]', kind) or
        kind.startswith('MUX2_LUT') or kind in ('ALU', 'INV', 'IBUF', 'BUFG', 'GND', 'VCC'))
    opaque_matched = 0
    for name in before.keys() | after.keys():
        a, b = before.get(name), after.get(name)
        share = bool(a and b and a['kind'] == b['kind'] and
            not a['kind'].startswith('DFF') and not combinational(a['kind'])
            and oldparams(a) == newparams(b) and
            all(a['ports'].get(p) == b['ports'].get(p)
                for p in set(a['ports']) | set(b['ports']) if 'CLK' in p))
        if share:
            opaque_matched += 1
        for cell, symbols, side in ((a, sa, 'old'), (b, sb, 'new')):
            if cell is None or cell['kind'].startswith('DFF') or combinational(cell['kind']):
                continue
            for pin, nets in cell['ports'].items():
                if directions[cell['kind']].get(pin) == 'output':
                    for i in range(len(nets)):
                        symbols[(name, pin, i)] = z3.Bool(
                            ('actual_opaque_instance:' if share else side+'_opaque:')+
                            name+':'+pin+':'+str(i))
    return (Logic(before, oldparams, directions, qa, sa),
            Logic(after, newparams, directions, qb, sb),
            dict(state_boundary='same actual primitive instance/kind/INIT/CLK',
                 shared_state_primitives=len(matched_ff),
                 shared_opaque_primitives=opaque_matched,
                 full_sequential_equivalence_claimed=False))


def plan(before, after, oldparams, newparams, directions, *, original_cst,
         logic_old=None, logic_new=None, removed_sfp_management_page_contract=False):
    requests = list(REQUEST.finditer(original_cst))
    assert len(requests) == 14, 'expected the original measured 14 routing requests'
    assert len({m[1] for m in requests}) == 14
    if logic_old is None and logic_new is None:
        logic_old, logic_new, boundaries = construct_shared_logic(
            before, after, oldparams, newparams, directions)
    else:
        assert logic_old is not None and logic_new is not None
        boundaries = dict(supplied_by_joint_reviewer=True,
                          full_sequential_equivalence_claimed=False)
    old_targets = {m[1] for m in requests}
    loads = {net: [] for net in old_targets}
    for name, cell in before.items():
        for pin, nets in cell['ports'].items():
            if directions[cell['kind']].get(pin) not in ('input', 'inout'):
                continue
            for i, net in enumerate(nets):
                if net in loads:
                    loads[net].append((name, pin, i))
    def sinks(net):
        return [(n, p, i) for n, cell in after.items() for p, ns in cell['ports'].items()
            if directions[cell['kind']].get(p) in ('input', 'inout')
            for i, candidate in enumerate(ns) if candidate == net]
    def role(cell, pin):
        if cell['kind'].startswith('DFF'):
            if pin in ('RESET', 'CLEAR', 'PRESET', 'SET'):
                return 'SR'
            if pin == 'CE':
                return 'CE'
            if pin == 'CLK':
                return 'CLOCK'
        return 'LOGIC'
    def prove_equal(label, a, b, condition=None):
        difference = z3.simplify(z3.Xor(a, b))
        if condition is not None:
            difference = z3.simplify(z3.And(condition, difference))
        if z3.is_false(difference):
            return 'canonical_logic_identity'
        solver = z3.Solver(); solver.set(timeout=10000); solver.add(difference)
        status = solver.check()
        assert status == z3.unsat, (label, str(status),
            str(solver.model())[:1200] if status == z3.sat else solver.reason_unknown())
        return 'SAT_unsatisfiable_miter'
    mappings, checks, errors = [], [], []
    for request in requests:
        oldnet, routing = request[1], request[2]
        try:
            drivers = logic_old.drivers.get(oldnet, [])
            assert len(drivers) == 1, ('old request driver is not unique', oldnet, drivers)
            name, pin, index = drivers[0]
            a = before[name]
            retained = [(n, p, i) for n, p, i in loads[oldnet]
                if n in after and p in after[n]['ports'] and i < len(after[n]['ports'][p])]
            assert retained, ('request has no surviving physical sink pins', oldnet)
            if name in after and pin in after[name]['ports'] and index < len(after[name]['ports'][pin]):
                newnet = after[name]['ports'][pin][index]
                selection = 'same_actual_primitive_instance_and_output_pin'
            else:
                candidates = {after[n]['ports'][p][i] for n, p, i in retained}
                assert len(candidates) == 1, ('no unique connectivity-derived replacement', oldnet, sorted(candidates))
                newnet, = candidates
                selection = 'all_surviving_actual_sink_pins_agree_on_one_net'
            assert newnet not in ('VCC', 'GND'), ('request reduced to constant', oldnet, newnet)
            actual = logic_new.drivers.get(newnet, [])
            assert len(actual) == 1, ('new request driver is not unique', oldnet, newnet, actual)
            newname, newpin, newindex = actual[0]
            b = after[newname]
            condition = None
            management_exception = None
            if removed_sfp_management_page_contract and oldnet == 'u_nav/a_q_15_17':
                # The source-bound top transformation removes the SFP 0x13xx
                # management-page mux. Bind the condition to the genuine old
                # arbiter address outputs and require all other pages equal.
                bits = []
                for bit in range(8,16):
                    address = 's_addr['+str(bit)+']'
                    assert len(logic_old.drivers.get(address,[])) == 1, (
                        'management-page condition has no actual address driver',address)
                    bits.append(logic_old.expr(address) == z3.BoolVal(bool(0x13 >> (bit-8) & 1)))
                condition = z3.Not(z3.And(*bits))
                management_exception = dict(removed_page='0x1300..0x13ff',
                    condition_bound_to_actual_old_arbiter_address_bits=list(range(8,16)),
                    non_removed_pages_must_be_strictly_equal=True,
                    removed_page_stub_behavior_accepted=False,
                    full_state_or_UART_acceptance=False,
                    purpose='Map the retained NAV address-load routing role after explicitly removing SFP management.')
            function_method = prove_equal(oldnet+' output function',
                logic_old.expr(oldnet), logic_new.expr(newnet), condition)
            sequential = None
            if a['kind'].startswith('DFF') or b['kind'].startswith('DFF'):
                assert a['kind'] == b['kind'] and oldparams(a) == newparams(b), (
                    'requested register type/INIT changed', oldnet)
                assert a['ports'].get('CLK') == b['ports'].get('CLK'), ('requested register clock changed', oldnet)
                sequential = prove_equal(oldnet+' complete next-state',
                    logic_old.next_state(a), logic_new.next_state(b))
            exact = [(n, p, i) for n, p, i in retained if after[n]['ports'][p][i] == newnet]
            assert exact, ('replacement does not drive a retained request load', oldnet, newnet)
            for required in ('SR', 'CE'):
                if required in routing.split('=')[-1].replace(' ', '').split('|'):
                    expected = [(n, p, i) for n, p, i in retained if role(before[n], p) == required]
                    if expected:
                        assert any(role(after[n], p) == required for n, p, i in exact), (
                            'requested control load role disappeared', oldnet, required)
            combinational = lambda kind: bool(re.fullmatch(r'LUT[1-6]',kind) or
                kind.startswith('MUX2_LUT') or kind in ('ALU','INV','IBUF','BUFG','GND','VCC'))
            semantic = [(n,p,i) for n,p,i in retained
                if before[n]['kind'] == after[n]['kind'] and
                not combinational(before[n]['kind']) and
                (not before[n]['kind'].startswith('DFF') or
                 (oldparams(before[n]) == newparams(after[n]) and
                  before[n]['ports'].get('CLK') == after[n]['ports'].get('CLK')))]
            # Generated combinational instance/pin identity is not a semantic
            # boundary: synthesis may repack a different function into it.
            # Check genuine state/opaque boundaries instead. Register input
            # rewrites are accepted only if the full transition is equal.
            moved = [(n,p,i,after[n]['ports'][p][i]) for n,p,i in semantic
                     if after[n]['ports'][p][i] != newnet]
            boundary_proofs = []
            for n,p,i,other in moved:
                if before[n]['kind'].startswith('DFF'):
                    method = prove_equal('retained request register transition '+n,
                        logic_old.next_state(before[n]), logic_new.next_state(after[n]),condition)
                else:
                    assert oldparams(before[n]) == newparams(after[n]), ('opaque boundary parameters changed',n)
                    method = prove_equal('retained request opaque input '+n+':'+p+':'+str(i),
                        logic_old.expr(oldnet), logic_new.expr(other),condition)
                boundary_proofs.append(dict(instance=n,pin=p,index=i,method=method))
            row = dict(old_target=oldnet, new_target=newnet, original_request=request[0],
                new_request='CLOCK_LOC "'+newnet+'" '+routing+';', selection=selection,
                old_driver=dict(instance=name, pin=pin, index=index, primitive=a),
                new_driver=dict(instance=newname, pin=newpin, index=newindex, primitive=b),
                old_parameters=oldparams(a), new_parameters=newparams(b),
                output_function_proof=function_method, requested_register_next_state_proof=sequential,
                removed_SFP_management_page_contract=management_exception,
                old_loads=len(loads[oldnet]), retained_old_loads=len(retained),
                exact_retained_loads=len(exact), equivalent_remapped_loads=len(moved),
                stable_state_or_opaque_loads=len(semantic),
                combinational_intermediate_pins_not_assumed_semantically_identical=len(retained)-len(semantic),
                actual_state_or_opaque_boundary_proofs=boundary_proofs,
                old_load_roles=dict(Counter(role(before[n], p) for n, p, i in loads[oldnet])),
                new_load_roles=dict(Counter(role(after[n], p) for n, p, i in sinks(newnet))),
                CST_target_changed=oldnet != newnet)
            mappings.append(row); checks.append(dict(old_target=oldnet, passed=True))
        except (AssertionError, KeyError, TypeError) as exc:
            errors.append(dict(old_target=oldnet, error=str(exc)))
            checks.append(dict(old_target=oldnet, passed=False, error=str(exc)))
    mapping = {row['old_target']: row['new_request'] for row in mappings}
    new_cst = REQUEST.sub(lambda m: mapping.get(m[1], m[0]), original_cst)
    original_io = [x for x in original_cst.splitlines() if x.startswith(('IO_LOC ', 'IO_PORT '))]
    new_io = [x for x in new_cst.splitlines() if x.startswith(('IO_LOC ', 'IO_PORT '))]
    assert original_io == new_io
    return dict(passed=not errors and len(mappings) == 14, new_cst=new_cst,
        mappings=mappings, checks=checks, errors=errors, state_boundary_audit=boundaries,
        all_real_IO_constraints_preserved=True, guessed_automatic_aliases=0,
        actual_global_allocation_verified=False, physical_timing_accepted=False,
        exploratory_implementation_preflight_only=True)
