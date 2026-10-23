"""Prove the two renamed SLF address LUTs before reusing physical constraints.

Only rb_ra[0:1] are allowed aliases. Match actual pins, LUT INIT/truth table,
and each unique upstream register's identity, inputs and explicit parameters.
This is a combinational/structural proof, not physical timing signoff.
"""
from audit_netlist_memories import PARAM
from revalidate_lidar_cdc import module_at

PREFIX = 'u_nav/u_match/u_ex/'


def parameters(cell, definitions):
    hierarchy,local = cell['name'].rsplit('/',1)
    body = module_at(definitions,hierarchy)
    pairs = [(k,v.strip()) for n,k,v in PARAM.findall(body) if n.strip().lstrip('\\') == local]
    assert len(pairs) == len(dict(pairs)), 'duplicate primitive parameter'
    return dict(pairs)


def prove(net, old, new, old_cells, new_cells, old_definitions, new_definitions):
    assert net in (PREFIX+'rb_ra[0]',PREFIX+'rb_ra[1]'), 'unreviewed alias target'
    assert len(old) == len(new) == 1, 'address driver not unique'
    old,new = old[0],new[0]
    assert old['kind'] == new['kind'] == 'LUT3', 'address driver type changed'
    assert old['ports'] == new['ports'], 'address driver pins changed'
    bit = net[-2]
    expected = {'F':[net],'I0':[PREFIX+f'u[{bit}]'],'I1':[PREFIX+f'fu[{bit}]'],'I2':[PREFIX+'flushing']}
    assert new['ports'] == expected, 'unexpected address-mux boundary'
    a,b = parameters(old,old_definitions),parameters(new,new_definitions)
    assert a == b == {'INIT':"8'hCA"}, 'address LUT function changed'
    for value in range(8):
        assert (0xca>>value)&1 == ((value>>1)&1 if value&4 else value&1)
    inputs = []
    for pin in ('I0','I1','I2'):
        signal, = new['ports'][pin]
        def upstream(cells):
            return [c for c in cells.values() if c['kind'].startswith('DFF') and c['ports'].get('Q') == [signal]]
        left,right = upstream(old_cells),upstream(new_cells)
        assert len(left) == len(right) == 1, 'address source register not unique'
        assert left[0] == right[0], 'address source register/clock/enable/reset changed'
        assert parameters(left[0],old_definitions) == parameters(right[0],new_definitions), 'address source INIT changed'
        inputs.append(dict(pin=pin,net=signal,register=left[0]['name'],register_inputs_and_parameters_identical=True))
    return dict(net=net,old_driver=old['name'],new_driver=new['name'],kind='LUT3',init="8'hCA",
                ports_identical=True,truth_table_cases=8,input_registers=inputs,
                scope='two proved generated LUT name aliases only; no clock exception or physical acceptance added')
