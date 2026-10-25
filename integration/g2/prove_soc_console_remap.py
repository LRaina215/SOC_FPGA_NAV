"""Prove the actual mapped SoC's retained state across console removal.

The proof cuts at stable register Qs and matching memory/DSP output pins. It
checks every retained register's next-state function and every retained opaque
primitive input using the public Gowin LUT/ALU/FF semantics. Only the documented
debug read-data update that formerly returned console storage may differ.
This is a combinational partition proof, not CPU program or timing acceptance.
"""
from collections import defaultdict
from pathlib import Path
import re
import sys
import time

DEPENDENCY = Path('F:/nav_integration_g2_20261023/routing_proof_dependencies')
if str(DEPENDENCY) not in sys.path:
    sys.path.insert(0, str(DEPENDENCY))
import z3


def literal(text):
    text = str(text).strip()
    m = re.fullmatch(r"(?:\d+)'([bhd])([0-9a-fA-F]+)", text)
    if m:
        return int(m[2], {'b':2,'h':16,'d':10}[m[1]])
    return int(text)


def console_path(name):
    return bool(re.match(r'^u_soc/g_con\[[0-3]\]\.u_con/', name))


class Logic:
    def __init__(self, cells, params, directions, q_symbols, opaque_symbols):
        self.cells, self.params, self.directions = cells, params, directions
        self.q_symbols, self.opaque_symbols = q_symbols, opaque_symbols
        self.drivers = defaultdict(list)
        self.memo = {'GND':z3.BoolVal(False), 'VCC':z3.BoolVal(True)}
        self.visiting = set()
        for name,c in cells.items():
            for pin,nets in c['ports'].items():
                if directions[c['kind']].get(pin) == 'output':
                    for index,net in enumerate(nets):
                        if net not in ('GND','VCC'):
                            self.drivers[net].append((name,pin,index))

    def one(self, c, pin, default=None):
        values = c['ports'].get(pin, [default] if default else None)
        assert values and len(values)==1, ('non-scalar logic pin', c['name'],pin,values)
        return self.expr(values[0])

    def expr(self, net):
        if net in self.memo:
            return self.memo[net]
        assert net not in self.visiting, ('combinational cycle', net)
        self.visiting.add(net)
        drivers = self.drivers.get(net, [])
        if not drivers:
            result = z3.Bool('external:'+net)
        else:
            assert len(drivers)==1, ('nonunique used signal driver',net,drivers)
            name,pin,index = drivers[0]
            c=self.cells[name];kind=c['kind'];p=self.params(c)
            if kind.startswith('DFF'):
                assert pin=='Q' and index==0
                result=self.q_symbols.get(net,z3.Bool('external_state:'+net))
            elif (name,pin,index) in self.opaque_symbols:
                result=self.opaque_symbols[(name,pin,index)]
            elif kind.startswith('LUT') and re.fullmatch('LUT[1-6]',kind):
                width=int(kind[3:]);bits=[self.one(c,'I'+str(i)) for i in range(width)]
                truth=literal(p['INIT'])
                def mux(level, table):
                    if level==0:return z3.BoolVal(bool(table&1))
                    half=1<<(level-1)
                    return z3.If(bits[level-1],mux(level-1,table>>half),mux(level-1,table&((1<<half)-1)))
                result=mux(width,truth)
            elif kind.startswith('MUX2_LUT'):
                result=z3.If(self.one(c,'S0'),self.one(c,'I1'),self.one(c,'I0'))
            elif kind=='INV':
                result=z3.Not(self.one(c,'I'))
            elif kind in ('IBUF','BUFG'):
                assert pin=='O'
                result=self.one(c,'I')
            elif kind=='ALU':
                i0,i1,i3,carry=[self.one(c,pin) for pin in ('I0','I1','I3','CIN')]
                mode=literal(p.get('ALU_MODE','0'))
                if mode==0:s,co=z3.Xor(i0,i1),i0
                elif mode in (1,4):s,co=z3.Xor(i0,z3.Not(i1)),i0
                elif mode==2:s,co=z3.If(i3,z3.Xor(i0,i1),z3.Xor(i0,z3.Not(i1))),i0
                elif mode==3:s,co=z3.Xor(i0,z3.Not(i1)),z3.BoolVal(True)
                elif mode==5:s,co=z3.Xor(z3.Not(i0),i1),i1
                elif mode==6:s,co=i0,z3.BoolVal(False)
                elif mode==7:s,co=z3.Not(i0),z3.BoolVal(True)
                elif mode==8:s,co=z3.If(i3,i0,z3.Not(i0)),i0
                else:raise AssertionError(('unsupported ALU mode',name,mode))
                assert pin in ('SUM','COUT')
                result=z3.Xor(s,carry) if pin=='SUM' else z3.If(s,carry,co)
            elif kind in ('VCC','GND'):
                result=z3.BoolVal(kind=='VCC')
            else:
                raise AssertionError(('unsupported mapped logic in proof cone',name,kind,pin))
        self.visiting.remove(net)
        result=z3.simplify(result)
        self.memo[net]=result
        return result

    def next_state(self,c, forced_D=None):
        q=self.one(c,'Q');d=self.one(c,'D') if forced_D is None else forced_D
        ce=self.one(c,'CE','VCC');kind=c['kind']
        if kind in ('DFFRE','DFFCE'):
            reset=self.one(c,'RESET' if kind=='DFFRE' else 'CLEAR')
            return z3.If(reset,z3.BoolVal(False),z3.If(ce,d,q))
        if kind in ('DFFSE','DFFPE'):
            reset=self.one(c,'SET' if kind=='DFFSE' else 'PRESET')
            return z3.If(reset,z3.BoolVal(True),z3.If(ce,d,q))
        if kind in ('DFF','DFFE'):
            return z3.If(ce,d,q)
        raise AssertionError(('unsupported register semantics',c['name'],kind))


def prove(before,after,old_params,new_params,directions,timeout_ms=10000):
    started=time.monotonic()
    a={n:c for n,c in before.items() if n.startswith('u_soc/')}
    b={n:c for n,c in after.items() if n.startswith('u_soc/')}
    assert a and b
    def state(cells):
        result={}
        for c in cells.values():
            if c['kind'].startswith('DFF'):
                net,=c['ports']['Q'];assert net not in result;result[net]=c
        return result
    qa,qb=state(a),state(b)
    # The CPU's actual output FF can keep its complete primitive definition
    # while the optimizer removes a shared _Z output alias. Pair only that
    # same actual instance; its complete transition is still proved below.
    by_name={c['name']:q for q,c in qb.items()}
    pairs={};renamed=[]
    for q,c in qa.items():
        if q in qb:pairs[q]=q
        elif c['name'] in by_name:
            new_q=by_name[c['name']];y=qb[new_q]
            assert c['name'].startswith('u_soc/u_cpu/') and c['kind']==y['kind'],('unexpected SoC state correspondence',q,new_q)
            assert {p:v for p,v in c['ports'].items() if p!='Q'}=={p:v for p,v in y['ports'].items() if p!='Q'},('renamed CPU Q changed another primitive pin',c['name'])
            assert old_params(c)==new_params(y),('renamed CPU Q changed INIT',c['name'])
            pairs[q]=new_q;renamed.append(dict(instance=c['name'],old=q,new=new_q,all_non_Q_pins_and_INIT_identical=True))
    assert len(pairs.values())==len(set(pairs.values())), 'nonbijective SoC state correspondence'
    removed=sorted(qa.keys()-pairs.keys());added=sorted(qb.keys()-set(pairs.values()))
    assert not added and removed,('unmatched SoC state',removed,added)
    allowed_removed=re.compile(r'^u_soc/con_(?:q(?:_\d+)?\[\d+\]|ra\[\d+\]|ra_\d+_\d+)$')
    assert all(allowed_removed.fullmatch(n) for n in removed), ('non-console state disappeared',removed)
    qsymbols={n:z3.Bool('state:'+n) for n in qa}
    for old,new in pairs.items():
        if new in qsymbols:assert z3.eq(qsymbols[new],qsymbols[old]),('SoC state alias collision',old,new)
        qsymbols[new]=qsymbols[old]
    opaque=lambda cells:{n:c for n,c in cells.items() if c['kind'] in ('SP','RAM16SDP4','MULT27X36') and not console_path(n)}
    oa,ob=opaque(a),opaque(b)
    assert oa.keys()==ob.keys(), ('retained CPU RAM/DSP leaf set changed',sorted(oa.keys()-ob.keys()),sorted(ob.keys()-oa.keys()))
    symbols={}
    for name,x in oa.items():
        y=ob[name];assert x['kind']==y['kind'] and x['ports'].keys()==y['ports'].keys()
        px,py=old_params(x),new_params(y)
        if x['kind']=='SP' and '/g_lane[' in name:
            assert {k:v for k,v in px.items() if not k.startswith('INIT_RAM')}=={k:v for k,v in py.items() if not k.startswith('INIT_RAM')}
        else:assert px==py,('non-program CPU RAM/DSP parameters changed',name)
        for pin,nets in x['ports'].items():
            assert len(nets)==len(y['ports'][pin]), ('primitive width changed',name,pin)
            if directions[x['kind']][pin]=='output':
                for index in range(len(nets)):
                    symbols[(name,pin,index)]=z3.Bool('primitive:'+name+':'+pin+':'+str(index))
    # Console-only RAM outputs are old, unconstrained state. They must have no
    # effect on retained transitions outside the explicit debug-return clause.
    for name,c in a.items():
        if console_path(name) and c['kind']=='RAM16SDP4':
            for pin,nets in c['ports'].items():
                if directions[c['kind']][pin]=='output':
                    for index in range(len(nets)):symbols[(name,pin,index)]=z3.Bool('removed_console:'+name+':'+pin+':'+str(index))
    la,lb=Logic(before,old_params,directions,qsymbols,symbols),Logic(after,new_params,directions,qsymbols,symbols)
    input_cuts=[]
    for q in sorted(pairs):
        if not re.fullmatch(r'u_soc/nav_q\[\d+\]',q):continue
        # nav_q samples the external navigation return bus. The reduced
        # navigation candidate changes some register-read values by design;
        # do not pull that separate implementation through this SoC proof.
        x,y=qa[q],qb[pairs[q]]
        assert x['ports']['D']==y['ports']['D'] and len(x['ports']['D'])==1
        net=x['ports']['D'][0]
        assert all(not name.startswith('u_soc/') for name,_,_ in la.drivers.get(net,[]))
        assert all(not name.startswith('u_soc/') for name,_,_ in lb.drivers.get(net,[]))
        shared=z3.Bool('SoC_navigation_data_input:'+net)
        la.memo[net]=lb.memo[net]=shared
        input_cuts.append(net)
    proved=[];sat_calls=0
    def check(label,x,y,condition=None):
        nonlocal sat_calls
        x,y=z3.simplify(x),z3.simplify(y)
        difference=z3.Xor(x,y)
        if condition is not None:difference=z3.And(condition,difference)
        difference=z3.simplify(difference)
        if z3.is_false(difference):
            proved.append(dict(endpoint=label,method='canonical_logic_identity'));return
        solver=z3.Solver();solver.set(timeout=timeout_ms);solver.add(difference);sat_calls+=1
        status=solver.check()
        assert status==z3.unsat, ('mapped retained behavior differs or remains unproved',label,str(status),
            str(solver.model())[:1000] if status==z3.sat else solver.reason_unknown())
        proved.append(dict(endpoint=label,method='SAT_unsatisfiable_miter'))
    # Derive the original reset expression from a retained resettable debug
    # state, then require the new one to have exactly the same Boolean value.
    hold_a,hold_b=qa['u_soc/hold'],qb['u_soc/hold']
    assert hold_a['kind']==hold_b['kind']=='DFFRE'
    rst_a=la.one(hold_a,'RESET');rst_b=lb.one(hold_b,'RESET');check('SoC protocol reset',rst_a,rst_b)
    state3=z3.And(qsymbols['u_soc/d_st[0]'],qsymbols['u_soc/d_st[1]'])
    # Synthesis shares the F008 decode outside nav_soc and does not retain a
    # net named u_soc/d_ram. Use the actual UART address bus, expanding its
    # genuine source drivers so shared decode outputs are not free aliases.
    assert all('u_addr['+str(i)+']' in la.drivers and 'u_addr['+str(i)+']' in lb.drivers for i in range(16))
    def ram_decode(logic):
        return z3.And(*[logic.expr('u_addr['+str(i)+']') == z3.BoolVal(bool(0xF008>>i&1)) for i in range(16)])
    ram_a,ram_b=ram_decode(la),ram_decode(lb);check('debug F008 address decode',ram_a,ram_b)
    console_update=z3.And(z3.Not(rst_a),state3,z3.Not(ram_a))
    type_changes=[]
    for q in sorted(pairs):
        x,y=qa[q],qb[pairs[q]]
        assert x['ports']['CLK']==y['ports']['CLK']==['clk50_d']
        old_init=literal(old_params(x).get('INIT',"1'b1" if x['kind'] in ('DFFSE','DFFPE') else "1'b0"))
        new_init=literal(new_params(y).get('INIT',"1'b1" if y['kind'] in ('DFFSE','DFFPE') else "1'b0"))
        assert old_init==new_init, ('retained state initial value changed',q)
        if x['kind']!=y['kind']:
            assert q=='u_soc/con_wp[0]' and x['kind']=='DFFCE' and y['kind']=='DFFRE'
            check('counter bit0 inactive async clear',la.one(x,'CLEAR'),z3.BoolVal(False))
            check('counter bit0 inactive sync reset',lb.one(y,'RESET'),z3.BoolVal(False))
            type_changes.append(dict(Q=q,before=x['kind'],after=y['kind'],both_clear_and_reset_constant_zero=True))
        elif x['kind'] in ('DFFCE','DFFPE'):
            pin='CLEAR' if x['kind']=='DFFCE' else 'PRESET';check(q+' async control',la.one(x,pin),lb.one(y,pin))
        nx,ny=la.next_state(x),lb.next_state(y)
        debug_read=bool(re.fullmatch(r'd_rdata_Z\[\d+\]',q))
        if debug_read:
            check(q+' retained non-console next-state',nx,ny,z3.Not(console_update))
            check(q+' removed-console read returns zero',ny,z3.BoolVal(False),console_update)
        else:check(q+' complete next-state',nx,ny)
    primitive_inputs=0
    for name,x in sorted(oa.items()):
        y=ob[name]
        for pin,nets in x['ports'].items():
            if directions[x['kind']][pin] not in ('input','inout'):continue
            for index,(old,new) in enumerate(zip(nets,y['ports'][pin])):
                check(name+':'+pin+':'+str(index),la.expr(old),lb.expr(new));primitive_inputs+=1
    # A deliberately broken retained counter update must be rejected by the
    # same SAT formulation; otherwise the mapping proof is not trustworthy.
    sample=qa['u_soc/con_wp[10]'];sample_new=qb['u_soc/con_wp[10]']
    solver=z3.Solver();solver.set(timeout=timeout_ms)
    solver.add(la.next_state(sample)!=lb.next_state(sample_new,forced_D=z3.BoolVal(True)))
    assert solver.check()==z3.sat, 'negative counter-data corruption was not rejected'
    return dict(passed=True,old_SoC_cells=len(a),new_SoC_cells=len(b),
        retained_register_Qs=len(qb),removed_console_register_Qs=removed,new_register_Qs=added,
        same_actual_CPU_FF_Q_renames=renamed,
        retained_CPU_register_Qs=sum(n.startswith('u_soc/u_cpu/') for n in qb),
        exact_unchanged_external_navigation_return_input_cuts=input_cuts,
        all_retained_state_INIT_and_clock_preserved=True,FF_type_changes=type_changes,
        retained_opaque_CPU_RAM_DSP_leaves=len(oa),primitive_input_bits_checked=primitive_inputs,
        proved_endpoints=len(proved),SAT_calls=sat_calls,proofs=proved,
        counter_corruption_negative_test_rejected=True,
        allowed_value_change='Only d_rdata next-state when !rst && d_st==3 && !d_ram; new value is zero. Normal held-address protocol restricts this to E-page console reads.',
        all_other_retained_next_state_and_CPU_RAM_DSP_inputs_proved=True,
        solver='Z3 '+z3.get_version_string(),dependency=str(DEPENDENCY),
        elapsed_s=time.monotonic()-started,full_CPU_program_or_physical_timing_acceptance=False)
