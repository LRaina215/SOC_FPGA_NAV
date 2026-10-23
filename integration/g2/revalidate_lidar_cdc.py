"""Recheck the 26 known crossings, allowing only proved reset-output aliases.

Generated reset wire numbers are not identity. A changed reset net is accepted
only when its unique driver is the same register with identical inputs and an
identical containing module definition (including every INIT parameter).
"""
from collections import defaultdict
from pathlib import Path
from synthesis_connectivity import MODULE, CELL


def definitions(path):
    return dict(MODULE.findall(Path(path).read_text(encoding='utf-8')))


def module_at(modules, hierarchy):
    assert hierarchy, 'top-level reset changes require separate review'
    kind = 'g2_top'
    for instance in hierarchy.split('/'):
        matches = [child for child, name, _ in CELL.findall(modules[kind])
                   if name.lstrip('\\') == instance]
        assert len(matches) == 1, ('reset hierarchy not unique', hierarchy)
        kind = matches[0]
    return modules[kind]


def revalidate(before, after, reference, directions, old_definitions, new_definitions):
    assert reference['passed'] and len(reference['paths']) == 26
    def drivers(cells):
        result = defaultdict(list)
        for name, cell in cells.items():
            for pin, nets in cell['ports'].items():
                if directions[cell['kind']][pin] == 'output':
                    for net in nets:result[net].append((name, cell))
        return result
    old_drivers, new_drivers = drivers(before), drivers(after)
    aliases = {}
    changed_pins = 0
    for row in reference['paths']:
        for role in ('source_register', 'first_register', 'second_register'):
            name = row[role]
            old, new = before.get(name), after.get(name)
            assert old and new and old['kind'] == new['kind'], ('CDC register missing/type changed', name)
            differing = {pin for pin in old['ports'].keys() | new['ports'].keys()
                         if old['ports'].get(pin) != new['ports'].get(pin)}
            assert differing <= {'RESET'}, ('CDC non-reset connection changed', name, differing)
            if not differing:continue
            old_net, = old['ports']['RESET']; new_net, = new['ports']['RESET']
            pair = old_net, new_net
            if pair not in aliases:
                assert len(old_drivers[old_net]) == len(new_drivers[new_net]) == 1, 'reset driver not unique'
                old_name, old_driver = old_drivers[old_net][0]
                new_name, new_driver = new_drivers[new_net][0]
                assert old_name == new_name and old_driver['kind'] == new_driver['kind'], 'reset driver changed'
                assert old_driver['kind'].startswith('DFF')
                assert old_driver['ports']['Q'] == [old_net] and new_driver['ports']['Q'] == [new_net]
                assert {p:v for p,v in old_driver['ports'].items() if p != 'Q'} == {
                    p:v for p,v in new_driver['ports'].items() if p != 'Q'}, 'reset driver input changed'
                hierarchy = old_name.rsplit('/', 1)[0]
                assert module_at(old_definitions, hierarchy) == module_at(new_definitions, hierarchy), 'reset module/INIT changed'
                aliases[pair] = dict(old_net=old_net, new_net=new_net, driver=old_name,
                                     kind=old_driver['kind'], all_driver_inputs_identical=True,
                                     module_definition_including_init_identical=True)
            changed_pins += 1
    # Recount actual loads; equality of named endpoint cells alone is insufficient.
    first_qs = {after[row['first_register']]['ports']['Q'][0] for row in reference['paths']}
    loads = defaultdict(list)
    for name, cell in after.items():
        for pin, nets in cell['ports'].items():
            if directions[cell['kind']][pin] in ('input', 'inout'):
                for net in nets:
                    if net in first_qs:loads[net].append((name,pin))
    for row in reference['paths']:
        source, first, second = [after[row[k]] for k in ('source_register','first_register','second_register')]
        assert source['ports']['Q'] == first['ports']['D'] and len(source['ports']['Q']) == 1
        assert new_drivers[first['ports']['D'][0]] == [(source['name'],source)]
        assert first['ports']['Q'] == second['ports']['D'] and len(first['ports']['Q']) == 1
        assert loads[first['ports']['Q'][0]] == [(second['name'],'D')], 'first stage has unexpected loads'
        assert source['ports']['CLK'] == [row['source_clock']]
        for cell in (first, second):
            assert cell['ports']['CLK'] == [row['target_clock']]
            assert cell['ports'].get('CE',['VCC']) == ['VCC']
    return dict(passed=True, paths=26, changed_reset_pin_connections=changed_pins,
                proved_reset_aliases=list(aliases.values()),
                source_first_second_data_clock_enable_and_single_fanout_rechecked=True,
                exception_endpoints_and_budgets_unchanged=True,
                scope='known SYN connectivity and proved reset alias only; no physical/reset-release/MTBF signoff')
