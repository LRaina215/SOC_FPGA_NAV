"""Inspect the real joint SYN snapshot data cone, without claiming routability.

Counts local LUT input pins in the policy module. Child ports are retained as
boundary nets; this is not a flattened global fanout or physical routing audit.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
from measure_snapshot_fanout import MODULE, CELL, PORT


def audit(build):
    build = Path(build).resolve()
    netlist = build / 'impl/gwsynthesis/g2_top.vg'
    modules = dict(MODULE.findall(netlist.read_text(encoding='utf-8')))
    module = 'rl_policy_tick_v5'
    assert module in modules, 'policy module missing'
    norm = lambda value: value.replace('\\', '').strip()
    cells = [(kind, norm(name), {pin: norm(net) for pin, net in PORT.findall(body)})
             for kind, name, body in CELL.findall(modules[module])]
    registers = [(kind, name, ports) for kind, name, ports in cells
                 if kind.startswith('DFF') and re.fullmatch(r'snapshot\[\d+\]', ports.get('Q', ''))]
    bits = [int(re.search(r'\[(\d+)\]', ports['Q'])[1]) for _, _, ports in registers]
    assert len(bits) == 4032 and set(bits) == set(range(4032)), 'snapshot register coverage changed'
    luts = {ports['F']: (kind, name, ports) for kind, name, ports in cells
            if kind.startswith('LUT') and 'F' in ports}
    data_nets = {ports['D'] for _, _, ports in registers}
    direct_luts = [luts[net] for net in data_nets if net in luts]
    loads = Counter(net for _, _, ports in direct_luts
                    for pin, net in ports.items() if pin.startswith('I') and net not in ('VCC', 'GND'))
    controls = {pin: dict(Counter(ports[pin] for _, _, ports in registers if pin in ports))
                for pin in ('CLK', 'CE', 'CLEAR', 'RESET', 'PRESET', 'SET')}
    rows = []
    for net, count in loads.most_common(20):
        driver = luts.get(net)
        rows.append(dict(net=net, direct_data_lut_input_pins=count,
                         local_lut_driver=dict(kind=driver[0], name=driver[1], ports=driver[2]) if driver else None))
    return dict(build=str(build), netlist=str(netlist), module=module,
                snapshot_register_count=len(registers), snapshot_bits_complete=True,
                direct_data_luts=len(direct_luts),
                data_nets_without_local_lut=sorted(data_nets - luts.keys()),
                maximum_direct_data_lut_input_pins=max(loads.values(), default=0),
                top_data_lut_inputs=rows, register_control_pins=controls,
                scope='actual joint SYN policy-local direct data cone; child ports are boundaries, not global fanout, unrouted-net identities or physical timing')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('build', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.build)
    args.out.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({key: result[key] for key in ('build', 'snapshot_register_count', 'direct_data_luts', 'maximum_direct_data_lut_input_pins')}))
