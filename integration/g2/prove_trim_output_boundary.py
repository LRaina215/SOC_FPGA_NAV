"""Exact OLED release and bus-output boundary proofs on actual mapped cells.

Renamed arbiter outputs are paired by their entire LUT function and actual
input pins, never by generated name similarity. Optional shared Logic objects
can also prove each pair. Top-level endpoints are returned for the caller's
global partition proof; this helper does not waive any other changed cell.
"""
from collections import defaultdict
import json
import re

from synthesis_connectivity import DECL


def outputs(cell, directions):
    return {p:n for p,n in cell['ports'].items() if directions[cell['kind']][p] == 'output'}


def prove(before, after, oldparams, newparams, directions, logic_old=None, logic_new=None,
          *, old_definitions=None, new_definitions=None):
    a = {n:c for n,c in before.items() if n.startswith('u_arb/')}
    b = {n:c for n,c in after.items() if n.startswith('u_arb/')}
    assert a and b and len(a) == len(b), 'arbiter primitive count changed'
    common = a.keys() & b.keys()
    assert all(a[n] == b[n] and oldparams(a[n]) == newparams(b[n]) for n in common), 'retained arbiter primitive changed'
    removed, added = a.keys()-b.keys(), b.keys()-a.keys()
    assert len(removed) == len(added)
    def signature(cell, params):
        assert cell['kind'] == 'LUT3' and cell['ports'].keys() == {'F','I0','I1','I2'}
        init = params(cell)
        assert init.keys() == {'INIT'}
        literal = re.fullmatch(r"8'[hH]([0-9a-fA-F]+)", init['INIT'])
        assert literal and int(literal[1],16) == 0xCA, ('arbiter output is not the exact source mux', cell['name'])
        return json.dumps([cell['kind'], init, {p:n for p,n in cell['ports'].items() if p != 'F'}], sort_keys=True)
    index = defaultdict(list)
    for name in added:
        index[signature(b[name],newparams)].append(name)
    mappings, covered = [], []
    for name in sorted(removed):
        old = a[name]
        matches = index[signature(old,oldparams)]
        assert len(matches) == 1, ('renamed mux has no unique exact input/function match',name,matches)
        new_name, = matches
        new = b[new_name]
        old_net, = old['ports']['F']; new_net, = new['ports']['F']
        payload = re.fullmatch(r's_(addr|wdata)\[(\d+)\]',old_net)
        assert payload and old['ports']['I0'] == [f'c_{payload[1]}[{payload[2]}]']
        assert old['ports']['I1'] == [f'u_{payload[1]}[{payload[2]}]'] and old['ports']['I2'] == ['sel']
        method = 'identical_LUT3_INIT_and_all_actual_input_pins'
        if logic_old is not None or logic_new is not None:
            assert logic_old is not None and logic_new is not None
            from prove_soc_console_remap import z3
            difference = z3.simplify(z3.Xor(logic_old.expr(old_net),logic_new.expr(new_net)))
            if not z3.is_false(difference):
                solver = z3.Solver(); solver.set(timeout=10000); solver.add(difference)
                assert solver.check() == z3.unsat, ('actual arbiter output function remains different/unproved',old_net,new_net)
            method += '_and_shared_logic_equivalence'
        mappings.append(dict(before=old_net,after=new_net,old_driver=name,new_driver=new_name,
            actual_inputs={p:v for p,v in old['ports'].items() if p != 'F'}, parameters=oldparams(old),method=method))
        covered.extend([dict(category='removed',name=name),dict(category='added',name=new_name)])
    assert len({m['new_driver'] for m in mappings}) == len(added), 'output mapping is not bijective'
    oled = []
    for port in ('sck','sda'):
        old = [c for c in before.values() if c['kind']=='IOBUF' and c['ports'].get('IO') == [port]]
        new = [c for c in after.values() if c['kind']=='TBUF' and c['ports'].get('O') == [port]]
        assert len(old) == len(new) == 1
        old,new = old[0],new[0]
        assert old['ports']['I'] == ['GND'] and new['ports'] == {'O':[port],'I':['GND'],'OEN':['VCC']}
        receive, = old['ports']['O']
        readers = [(n,p) for n,c in before.items() for p,ns in c['ports'].items()
                   if directions[c['kind']][p]=='input' and receive in ns]
        assert readers and all(n.startswith('u_control/display/') for n,p in readers), ('OLED input has non-display consumers',port,readers)
        oled.append(dict(port=port,old_buffer=old,new_buffer=new,
            old_receiver_consumers=readers,actual_open_drain_release=True))
        covered.extend([dict(category='removed',name=old['name']),dict(category='added',name=new['name'])])
    endpoints = []
    if old_definitions is not None or new_definitions is not None:
        assert old_definitions is not None and new_definitions is not None
        endpoints = top_endpoints(before,after,old_definitions,new_definitions,directions)
    return dict(passed=True,arbiter_common_cells_identical=len(common),
        arbiter_renamed_output_mappings=mappings,actual_net_mapping={m['before']:m['after'] for m in mappings},
        OLED_pin_release=oled,covered_cell_changes=covered,
        top_endpoint_pairs=endpoints,
        global_state_opaque_input_and_external_endpoint_proof_still_required=True,
        timing_or_deployment_accepted=False)


def top_endpoints(before,after,old_definitions,new_definitions,directions):
    def ports(definitions):
        return {n.lstrip('\\'):(direction,int(hi) if hi else None,int(lo) if lo else None)
            for direction,hi,lo,n in DECL.findall(definitions['g2_top']) if direction != 'wire'}
    old_ports,new_ports = ports(old_definitions),ports(new_definitions)
    assert old_ports == new_ports, 'external port directions/widths/names changed'
    def drivers(cells):
        result=defaultdict(list)
        for name,c in cells.items():
            for pin,nets in c['ports'].items():
                if directions[c['kind']][pin] in ('output','inout'):
                    for index,net in enumerate(nets):result[net].append((name,pin,index))
        return result
    da,db=drivers(before),drivers(after)
    result=[]
    for port,(direction,hi,lo) in old_ports.items():
        if direction == 'input' or port in {'sck','sda'}:
            continue
        bits=[port] if hi is None else [f'{port}[{i}]' for i in range(hi,lo-1,-1)]
        for net in bits:
            assert len(da[net]) == len(db[net]) == 1, ('nonunique external driver',net,da[net],db[net])
            an,ap,ai=da[net][0];bn,bp,bi=db[net][0];a,b=before[an],after[bn]
            if a['kind'] in {'OBUF','IOBUF','TBUF'} or b['kind'] in {'OBUF','IOBUF','TBUF'}:
                assert a['kind']==b['kind'] and (ap,ai)==(bp,bi), ('non-OLED output buffer changed',net,a,b)
                assert a['ports'].keys()==b['ports'].keys()
                for pin,nets in a['ports'].items():
                    if directions[a['kind']][pin] != 'input':continue
                    assert len(nets)==len(b['ports'][pin])
                    for index,(x,y) in enumerate(zip(nets,b['ports'][pin])):
                        result.append(dict(external_port=net,kind='buffer_input',pin=pin,index=index,
                            before=x,after=y,old_driver=an,new_driver=bn))
            else:
                result.append(dict(external_port=net,kind='direct_external_net',before=net,after=net,
                    old_driver=[an,ap,ai],new_driver=[bn,bp,bi]))
    return result
