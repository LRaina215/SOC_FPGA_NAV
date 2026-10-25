"""Actual mapped retained-shell partition proof for explicit delivery trims.

Register correspondences use the same primitive instance, never a numeric-name
guess. A correspondence is accepted only after INIT, clock, async controls and
the complete next-state function have been checked. Changed child scopes are
separate source-bound contracts; this proof checks every retained state and
opaque primitive input outside them, including globally moved reset/decode.
"""
from collections import Counter
import re
import time

from prove_soc_console_remap import Logic, literal, z3


def within(name, scopes):
    return any(name==p.rstrip('/') or name.startswith(p.rstrip('/')+'/') for p in scopes)


def prove(before, after, old_params, new_params, directions, scopes,
          tx_mapping=None, navigation_result_contract=False,
          removed_tx_shadow_contract=False, separately_proved_soc=False,
          external_endpoints=(), global_signal_endpoints=(),
          result_contract_records=(), timeout_ms=10000):
    started=time.monotonic()
    def selected(name):
        return not (separately_proved_soc and name.startswith('u_soc/')) and not within(name,scopes)
    old_ff={n:c for n,c in before.items() if c['kind'].startswith('DFF')}
    new_ff={n:c for n,c in after.items() if c['kind'].startswith('DFF')}
    shell_old={n:c for n,c in old_ff.items() if selected(n)}
    shell_new={n:c for n,c in new_ff.items() if selected(n)}
    # Prefer an unchanged actual Q signal. The optimizer can rename a FF
    # instance while keeping that Q, or rename Q while retaining its instance.
    # Neither case is accepted until its actual transition is proved below.
    by_q={c['ports']['Q'][0]:n for n,c in new_ff.items()}
    pairs={}
    for name,c in old_ff.items():
        candidate=by_q.get(c['ports']['Q'][0], name if name in new_ff else None)
        if candidate is not None:pairs[name]=candidate
    assert len(set(pairs.values()))==len(pairs), 'nonbijective retained FF correspondence'
    selected_pairs={n:m for n,m in pairs.items() if selected(n)}
    assert set(selected_pairs)==set(shell_old) and set(selected_pairs.values())==set(shell_new), ('unmatched retained state primitive',
        sorted(shell_old.keys()-selected_pairs.keys()),sorted(shell_new.keys()-set(selected_pairs.values())))
    qa,qb={},{}
    aliases=[]
    for name,x in old_ff.items():
        y=new_ff.get(pairs.get(name))
        shared=z3.Bool('state_instance:'+name)
        qa[x['ports']['Q'][0]]=shared
        if y:qb[y['ports']['Q'][0]]=shared
        if x and y and x['ports']['Q']!=y['ports']['Q'] and selected(name):
            aliases.append(dict(instance=name,old=x['ports']['Q'][0],new=y['ports']['Q'][0]))
    for name,c in new_ff.items():
        qb.setdefault(c['ports']['Q'][0],z3.Bool('new_state_instance:'+name))
    if removed_tx_shadow_contract and tx_mapping is None:
        from prove_trim_tx_memory_mapping import prove as prove_tx
        tx_mapping=prove_tx(before,after,old_params,new_params,directions,qa,qb,
                            timeout_ms=timeout_ms)
        assert tx_mapping['passed']
    comb=lambda k:bool(re.fullmatch('LUT[1-6]',k) or k.startswith('MUX2_LUT') or
        k in ('ALU','INV','VCC','GND','BUFG','IBUF'))
    opaque=lambda cells:{n:c for n,c in cells.items() if not c['kind'].startswith('DFF') and not comb(c['kind'])}
    oa,ob=opaque(before),opaque(after)
    sa,sb={},{}
    for name in oa.keys()|ob.keys():
        x,y=oa.get(name),ob.get(name)
        for c,symbols in ((x,sa),(y,sb)):
            if c is None:continue
            for pin,nets in c['ports'].items():
                if directions[c['kind']].get(pin)=='output':
                    for i in range(len(nets)):
                        symbols[(name,pin,i)]=z3.Bool('opaque_instance:'+name+':'+pin+':'+str(i))
    if tx_mapping:
        assert tx_mapping['passed']
        for row in tx_mapping['memory_leaf_mapping']:
            a,b=row['old'],row['new']
            for pin,nets in oa[a]['ports'].items():
                if directions[oa[a]['kind']].get(pin)=='output':
                    assert len(nets)==len(ob[b]['ports'][pin])
                    for i in range(len(nets)):
                        shared=z3.Bool('proved_TX_memory:'+a+':'+pin+':'+str(i))
                        sa[(a,pin,i)]=sb[(b,pin,i)]=shared
    la,lb=Logic(before,old_params,directions,qa,sa),Logic(after,new_params,directions,qb,sb)
    failures=[];proofs=[];sat_calls=0
    def check(label,x,y,condition=None):
        nonlocal sat_calls
        difference=z3.simplify(z3.Xor(x,y))
        if condition is not None:difference=z3.simplify(z3.And(condition,difference))
        if z3.is_false(difference):
            proofs.append(dict(endpoint=label,method='canonical_logic_identity'));return True
        solver=z3.Solver();solver.set(timeout=timeout_ms);solver.add(difference);sat_calls+=1
        status=solver.check()
        if status!=z3.unsat:
            failures.append(dict(endpoint=label,status=str(status),
                counterexample=str(solver.model())[:4000] if status==z3.sat else solver.reason_unknown()))
            if len(failures)<=80:print('Unproved retained endpoint:',label,str(status),flush=True)
            return False
        proofs.append(dict(endpoint=label,method='SAT_unsatisfiable_miter'));return True
    exceptions=[]
    shadow_update=None;result_updates=[]
    if removed_tx_shadow_contract or navigation_result_contract:
        def bit(net):
            assert net in qa and net in qb,('latched protocol state not present',net)
            assert z3.eq(qa[net],qb[net]);return qa[net]
        # nav_subsystem: B_RF=1 samples nav_regfile, whose sole removed
        # shadow read register is exactly 0x1240. The address is latched a_q;
        # all transport ACK/state transitions remain independently checked.
        def bus_state(value):
            return z3.And(*[bit('u_nav/bs['+str(i)+']')==z3.BoolVal(bool(value>>i&1)) for i in range(4)])
        address=z3.And(*[bit('u_nav/a_q['+str(i)+']')==z3.BoolVal(bool(0x1240>>i&1)) for i in range(2,16)])
        # The core reset value itself is proved at every changed retained
        # RESET pin below. Use a real bus state register's RESET source.
        state_cell=next(c for c in shell_old.values() if c['ports']['Q']==['u_nav/bs[0]'])
        assert state_cell['kind']=='DFFRE'
        not_reset=z3.Not(la.one(state_cell,'RESET'))
        if removed_tx_shadow_contract:shadow_update=z3.And(not_reset,bus_state(1),address)
        if navigation_result_contract:
            assert {r['bus_state'] for r in result_contract_records}=={6,8}, 'actual result interface records missing'
            for record in result_contract_records:
                assert len(record['old_stored_nets'])==len(record['new_stored_nets'])==32
                assert record['new_valid_net'] not in ('GND','VCC')
                result_updates.append((record,z3.And(not_reset,bus_state(record['bus_state']))))
    for name,x in sorted(shell_old.items()):
        y=shell_new[selected_pairs[name]]
        assert x['kind']==y['kind'],('retained FF kind changed',name)
        assert x['ports']['CLK']==y['ports']['CLK'],('retained FF clock changed',name)
        px,py=old_params(x),new_params(y)
        default="1'b1" if x['kind'] in ('DFFSE','DFFPE') else "1'b0"
        assert literal(px.get('INIT',default))==literal(py.get('INIT',default)),('retained FF INIT changed',name)
        assert {k:v for k,v in px.items() if k!='INIT'}=={k:v for k,v in py.items() if k!='INIT'},('retained FF non-INIT parameters changed',name)
        try:
            if x['kind'] in ('DFFCE','DFFPE'):
                pin='CLEAR' if x['kind']=='DFFCE' else 'PRESET'
                check(name+':async_'+pin,la.one(x,pin),lb.one(y,pin))
            if 'RESET' in x['ports'] and x['ports']['RESET']!=y['ports']['RESET']:
                check(name+':synchronous_RESET_value',la.one(x,'RESET'),lb.one(y,'RESET'))
            nx,ny=la.next_state(x),lb.next_state(y)
            bus_word=re.fullmatch(r'u_nav/bus_rdata_(\d+)_s\d+',name)
            if (removed_tx_shadow_contract or navigation_result_contract) and bus_word:
                exceptions_for_value=[condition for _,condition in result_updates]
                if shadow_update is not None:exceptions_for_value.append(shadow_update)
                check(name+':other_next_state',nx,ny,z3.Not(z3.Or(*exceptions_for_value)))
                if shadow_update is not None:check(name+':removed_shadow_read_returns_zero',ny,z3.BoolVal(False),shadow_update)
                index=int(bus_word[1])
                for record,condition in result_updates:
                    old_data=la.expr(record['old_stored_nets'][index])
                    new_data=z3.And(lb.expr(record['new_stored_nets'][index]),lb.expr(record['new_valid_net']))
                    check(name+':'+record['module']+':original_selected_result',nx,old_data,condition)
                    check(name+':'+record['module']+':new_selected_validated_result',ny,new_data,condition)
            else:check(name+':next_state',nx,ny)
        except AssertionError as error:
            exceptions.append(dict(endpoint=name,error=str(error)))
        if len(proofs)%4000<2:
            print('Retained state proof:',len(proofs),'endpoints,',len(failures),'counterexamples,',len(exceptions),'unsupported cones',flush=True)
    primitive_bits=0;retained_opaque=0
    for name,x in sorted(oa.items()):
        if not selected(name) or x['kind']=='IOBUF' and name in ('sck_iobuf','sda_iobuf'):continue
        if name not in ob:
            exceptions.append(dict(endpoint=name,error='retained opaque primitive removed'));continue
        y=ob[name];assert x['kind']==y['kind'] and old_params(x)==new_params(y),('retained opaque geometry/INIT changed',name)
        assert x['ports'].keys()==y['ports'].keys(),('retained opaque ports changed',name)
        retained_opaque+=1
        for pin,nets in x['ports'].items():
            assert len(nets)==len(y['ports'][pin])
            if directions[x['kind']].get(pin) not in ('input','inout'):continue
            for i,(a,b) in enumerate(zip(nets,y['ports'][pin])):
                try:check(name+':'+pin+':'+str(i),la.expr(a),lb.expr(b));primitive_bits+=1
                except AssertionError as error:exceptions.append(dict(endpoint=name+':'+pin+':'+str(i),error=str(error)))
    added_opaque=sorted(n for n in ob.keys()-oa.keys() if selected(n) and n not in ('sck_s0','sda_s0'))
    mapped_tx_new=set(r['new'] for r in tx_mapping['memory_leaf_mapping']) if tx_mapping else set()
    added_opaque=[n for n in added_opaque if n not in mapped_tx_new]
    if added_opaque:exceptions.append(dict(endpoint='added opaque primitives',error=added_opaque))
    for row in external_endpoints:
        try:check('external '+row['external_port']+':'+row.get('pin','direct')+':'+str(row.get('index',0)),
                  la.expr(row['before']),lb.expr(row['after']))
        except AssertionError as error:exceptions.append(dict(endpoint=row,error=str(error)))
    for net in global_signal_endpoints:
        try:
            assert len(la.drivers.get(net,[]))==len(lb.drivers.get(net,[]))==1,('global target not unique',net)
            assert la.drivers[net][0]==lb.drivers[net][0],('global target actual driver identity changed',net)
            check('named global/clock target '+net,la.expr(net),lb.expr(net))
        except AssertionError as error:exceptions.append(dict(endpoint=net,error=str(error)))
    return dict(passed=not failures and not exceptions,retained_state_instances=len(shell_old),
        same_instance_Q_correspondences=aliases,register_clocks_and_INIT_identical=True,
        same_Q_primitive_instance_renames=[dict(old=n,new=m,Q=old_ff[n]['ports']['Q'][0]) for n,m in selected_pairs.items() if n!=m],
        retained_opaque_primitives=retained_opaque,opaque_input_bits_checked=primitive_bits,
        proof_endpoints=len(proofs),SAT_calls=sat_calls,failures=failures,exceptions=exceptions,
        proofs=proofs,solver='Z3 '+z3.get_version_string(),elapsed_s=time.monotonic()-started,
        source_bound_navigation_result_parent_selection_records=list(result_contract_records),
        navigation_result_proof_uses_real_selected_parent_next_state_and_no_net_memo_cuts=True,
        actual_live_TX_storage_migration_proof=tx_mapping,
        external_output_or_buffer_input_endpoints_checked=len(external_endpoints),
        named_global_or_clock_target_values_checked=list(global_signal_endpoints),
        allowed_parent_read_value_change='Only !rst && bs==B_RF(1) && latched word address a_q==0x1240; returns zero' if removed_tx_shadow_contract else None,
        abstraction='Same actual FF instance state, same actual opaque primitive outputs; modified child scopes are independently source-bound contracts. All retained FF next-state/async inputs and opaque input functions are checked.',
        physical_timing_or_deployment_acceptance=False)
