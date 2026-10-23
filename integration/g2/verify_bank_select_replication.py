"""Prove the measured selector replicas' transition function from SYN cells.

Enumerates all64 states,64 bank addresses and both read-enable values. Every
replica starts at0 and must implement the original update/hold function. This
checks the selector cone only, not RAM contents, PNR or whole-design timing.
"""
import argparse
import json
from pathlib import Path
import re

from audit_netlist_memories import MODULE, INST, PARAM
from synthesis_connectivity import read


def literal(value):
    m = re.fullmatch(r"(\d+)'([bhd])([0-9a-fA-F]+)", value)
    assert m, value
    return int(m[3], {'b':2,'h':16,'d':10}[m[2]])


def verify(netlist):
    netlist = Path(netlist)
    modules = dict(MODULE.findall(netlist.read_text(encoding='utf-8')))
    parameters = {}
    def walk(kind, prefix=''):
        local = {}
        for instance, key, value in PARAM.findall(modules[kind]):
            local.setdefault(instance.lstrip('\\').strip(), {})[key] = value.strip()
        for child, name in INST.findall(modules[kind]):
            name = name.lstrip('\\')
            full = (prefix+'/'+name).strip('/')
            if child in modules:
                walk(child, full)
            else:
                parameters[full] = local.get(name, {})
    walk('g2_top')
    cells = read(netlist)
    drivers, registers = {}, {}
    for name, c in cells.items():
        if c['kind'].startswith('DFF') and 'selected' in name:
            bit = int(re.search(r'selected_(\d+)_s', name)[1])
            assert literal(parameters[name]['INIT']) == 0
            assert c['ports']['CLK'] == ['clk_d'] and c['ports']['RESET'] == ['GND']
            registers[name] = bit
        for pin in ('F','Q','O'):
            if pin in c['ports'] and len(c['ports'][pin]) == 1:
                net, = c['ports'][pin]
                assert net not in drivers
                drivers[net] = name
    assert set(registers.values()) == set(range(6))
    feedback = {cells[n]['ports']['Q'][0]: bit for n, bit in registers.items()}
    checks = 0
    for state in range(64):
        for address in range(64):
            for enable in (0,1):
                values = {'GND':0, 'VCC':1, 're':enable}
                values.update({f'raddr[{10+i}]':address>>i&1 for i in range(6)})
                values.update({net:state>>i&1 for net,i in feedback.items()})
                def evaluate(net):
                    if net in values:
                        return values[net]
                    assert net in drivers, ('unexpected dependency', net)
                    name = drivers[net]
                    c = cells[name]
                    if c['kind'] == 'IBUF':
                        answer = evaluate(c['ports']['I'][0])
                    elif re.fullmatch(r'LUT[1-4]', c['kind']):
                        n = int(c['kind'][-1])
                        index = sum(evaluate(c['ports']['I'+str(i)][0])<<i for i in range(n))
                        answer = literal(parameters[name]['INIT'])>>index&1
                    elif c['kind'].startswith('MUX2_'):
                        answer = evaluate(c['ports']['I1' if evaluate(c['ports']['S0'][0]) else 'I0'][0])
                    else:
                        raise AssertionError(('unexpected selector cone', name, c['kind']))
                    values[net] = answer
                    return answer
                expected = address if enable else state
                for name, bit in registers.items():
                    p = cells[name]['ports']
                    actual = evaluate(p['D'][0]) if evaluate(p['CE'][0]) else state>>bit&1
                    assert actual == expected>>bit&1, (name,state,address,enable,actual,expected)
                    checks += 1
    return dict(netlist=str(netlist.resolve()), selector_registers=len(registers),
                state_address_enable_cases=8192, register_transition_checks=checks,
                all_initial_zero=True, clock_reset_match=True, read_hold_and_update_exact=True,
                passed=True, scope='exhaustive actual SYN selector-cone induction; not RAM data or PNR verification')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('netlist', type=Path)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    result = verify(a.netlist)
    a.out.write_text(json.dumps(result, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps(result))
