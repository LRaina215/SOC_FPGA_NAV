"""Read-only SYN39 ring storage/resource/memory/clock/CDC review.

This never launches EDA or PNR. The separate region reviewer proves the actual
lane-write and snapshot-word Boolean next-state functions; this entry measures
storage/selection topology and retains unknown physical quantities explicitly.
"""
from pathlib import Path
from datetime import datetime
from collections import Counter, defaultdict
from functools import lru_cache
import argparse, json, re, traceback
from bs4 import BeautifulSoup
from synthesis_connectivity import read
from revalidate_lidar_cdc import definitions, revalidate
from review_small_block_stores import parameter_reader
from audit_netlist_memories import BSRAM
from audit_routing_topology import primitive_directions
from prepare_baseline_fusion_retime import metadata, same_bytes
from review_baseline_single_factor_syn import cpu_image
from review_lease_deadline_syn_mapping import hierarchy_resources
from review_lease_bounded_MAX_joint_mapping import report_paths
from review_model25_joint_syn import module_bindings

BASE = Path('F:/nav_integration_g2_20261023')
OLD = BASE / 'baseline_fusion_model25_bounded_MAX_syn37'
NEW = BASE / 'baseline_fusion_model25_observation_ring_syn39'
PREP = BASE / 'baseline_fusion_model25_observation_ring_prepared2'
LIB = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
RADAR = BASE / 'model25_SYN34_actual_review1/actual_review4/actual_original_radar26.json'
PREFIX = 'u_control/model_thread/model/'
OBS = PREFIX + 'observation/'
POLICY = PREFIX + 'policy/'
DELTA = {'control/rl_observation_v5.v', 'control/rl_policy_tick_v5.v',
         'control/mem_policy.v', 'control/mem_policy_related25.v'}

def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))

def save(path, value):
    assert not path.exists(), ('do not overwrite evidence', str(path))
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf8')

def resources(cells):
    c = Counter(row['kind'] for row in cells.values())
    return dict(FF=sum(v for k, v in c.items() if k.startswith('DFF')),
                LUT=sum(v for k, v in c.items() if re.fullmatch(r'LUT[1-6]', k)),
                INV=c['INV'], ALU=c['ALU'], BSRAM=sum(c[k] for k in BSRAM),
                RAM16=sum(v for k, v in c.items() if k.startswith('RAM16')),
                ROM16=c['ROM16'], primitive_counts=dict(c))

def input_names(root):
    paths = [Path(p) for p in load(root / 'manifest.json')['source_files']]
    result = {p.relative_to(root).as_posix(): p for p in paths}
    assert len(result) == len(paths) == 66
    return result

def source_review():
    old, new, prepared = map(input_names, (OLD, NEW, PREP))
    assert old.keys() == new.keys() == prepared.keys()
    changed = {p for p in old if not same_bytes(old[p], new[p])}
    assert changed == DELTA, ('unexpected source delta', sorted(changed))
    assert all(same_bytes(new[p], prepared[p]) for p in new)
    sets = {}
    for suffixes, label, expected in [({'.hex', '.mem', '.bin', '.mif'}, 'assets', 69),
                                     ({'.h', '.vh'}, 'headers', 31)]:
        paths = {p.relative_to(OLD): p for p in OLD.rglob('*')
                 if p.is_file() and 'impl' not in p.relative_to(OLD).parts
                 and '__pycache__' not in p.parts and p.suffix.lower() in suffixes}
        assert len(paths) == expected
        assert all(same_bytes(p, NEW / name) for name, p in paths.items())
        sets[label] = len(paths)
    firmware = {}
    for folder in ('fw', 'firmware'):
        paths = {p.relative_to(OLD): p for p in (OLD / folder).rglob('*')
                 if p.is_file() and '__pycache__' not in p.parts}
        assert all(same_bytes(p, NEW / name) for name, p in paths.items())
        firmware[folder] = len(paths)
    for name in ('g2.cst', 'g2.sdc', 'fc_v4_top.v', 'fixed4500_weight_rom.v',
                 'control/policy_action_map.v', 'control/imu_fusion.v'):
        assert same_bytes(OLD / name, NEW / name), name
    tcl = (NEW / 'run.tcl').read_text()
    selected = re.findall(r'^add_file -type verilog \{(.+)\}$', tcl, re.M)
    assert len(selected) == 66 and {Path(p).resolve() for p in selected} == {p.resolve() for p in new.values()}
    assert not any('prepared' in p.lower() for p in selected)
    assert (NEW / 'g2.sdc').read_text().count('create_generated_clock -name model25') == 1
    return dict(passed=True, changed_RTL=sorted(changed), other62_RTL_identical=True,
                all66_inputs_frozen_in_actual_SYN39=True, identical_assets_headers=sets,
                complete_original_firmware=firmware, core_math_input_mem_weights_CST_SDC_unchanged=True,
                automatic_PNR=False, source_bindings=[metadata(p) for p in new.values()])

def index(cells, directions):
    drivers, loads = defaultdict(list), defaultdict(list)
    for name, c in cells.items():
        assert c['kind'] in directions, ('unknown primitive', name, c['kind'])
        for pin, nets in c['ports'].items():
            assert pin in directions[c['kind']], (name, pin)
            for bit, net in enumerate(nets):
                if net in ('GND', 'VCC'):
                    continue
                if directions[c['kind']][pin] == 'output':
                    drivers[net].append((name, pin, bit))
                elif directions[c['kind']][pin] in ('input', 'inout'):
                    loads[net].append((name, pin, bit))
    return drivers, loads

def clock_reader(cells, drivers):
    @lru_cache(None)
    def clock(net):
        if net in ('GND', 'VCC'):
            return net, 0
        assert len(drivers[net]) == 1, ('clock driver', net, drivers[net])
        n, p, i = drivers[net][0]
        c = cells[n]
        if c['kind'] == 'IBUF':
            assert p == 'O' and i == 0 and c['ports']['I'] in (['clk50'], ['rgmii_rxc'])
            return c['ports']['I'][0], 0
        if c['kind'] == 'PLL':
            assert n == 'u_model_pll/PLL_inst' and p == 'CLKOUT0' and i == 0
            assert clock(c['ports']['CLKIN'][0]) == ('clk50', 0)
            return 'model25', 0
        assert c['kind'] in ('INV', 'BUF', 'BUFG'), ('unexpected clock primitive', net, c)
        r, phase = clock(c['ports']['I'][0])
        return r, phase ^ int(c['kind'] == 'INV')
    return clock

def clock_inventory(cells, clock):
    pins = {}
    for name, c in cells.items():
        for pin, nets in c['ports'].items():
            if re.fullmatch(r'CLK\d*|CLKA|CLKB|CLOCK|WCLK|RCLK', pin):
                for bit, net in enumerate(nets):
                    if net not in ('GND', 'VCC'):
                        pins[name, pin, bit] = clock(net)
    counts = Counter(pins.values())
    return pins, [dict(root=r, inverted=bool(p), pins=n) for (r, p), n in sorted(counts.items())]

def memories(cells, parameters):
    return {n: dict(name=n, kind=c['kind'], ports=c['ports'], parameters=parameters(c))
            for n, c in cells.items() if c['kind'] in BSRAM or c['kind'].startswith(('RAM16', 'ROM16'))}

def group_ffs(cells):
    stems = dict(history_flat=OBS + 'history_flat_', current_flat=OBS + 'current_flat_',
                 words=OBS + 'words_', snapshot=POLICY + 'snapshot_')
    return {key: {n: c for n, c in cells.items() if c['kind'].startswith('DFF') and n.startswith(stem)
                  and (key == 'words' or re.match(r'\d+_s', n[len(stem):]))}
            for key, stem in stems.items()}

def ring_structure(cells, parameters, directions, drivers, loads, clock):
    ring = {n: c for n, c in cells.items() if n.startswith(OBS + 'g_ring.')}
    rams = {n: c for n, c in ring.items() if c['kind'].startswith('RAM16')}
    address_dsp = {n: c for n, c in cells.items()
                   if n.startswith(OBS) and c['kind'] == 'MULT12X12'}
    assert rams, 'ring did not map to actual distributed RAM'
    assert not any(c['kind'] in BSRAM for c in ring.values()), 'ring unexpectedly consumes BSRAM'
    assert not any(c['kind'].startswith('DFF') and 'frame_mem' in n for n, c in ring.items()), 'ring storage was rebuilt in FFs'
    assert all(c['kind'] in ('RAM16SDP1', 'RAM16SDP4') for c in rams.values())
    widths = {'RAM16SDP1': 1, 'RAM16SDP4': 4}
    banks = defaultdict(list)
    ram_rows = []
    for n, c in rams.items():
        assert set(c['ports']) == {'DO', 'DI', 'WAD', 'RAD', 'WRE', 'CLK'}, (n, c)
        assert len(c['ports']['WAD']) == len(c['ports']['RAD']) == 4
        assert len(c['ports']['DO']) == len(c['ports']['DI']) == widths[c['kind']]
        assert [clock(v) for v in c['ports']['CLK']] == [('model25', 0)]
        write = {p: c['ports'][p] for p in ('CLK', 'WRE', 'WAD', 'DI')}
        banks[json.dumps(dict(kind=c['kind'], parameters=parameters(c), ports=write), sort_keys=True)].append(n)
        ram_rows.append(dict(**c, parameters=parameters(c), words=16, width=widths[c['kind']],
                             primitive_capacity_bits=16 * widths[c['kind']],
                             single_synchronous_write_port=True, separate_asynchronous_read_address=True,
                             no_RAM_reset_pin=True))
    prefetch = {n: c for n, c in cells.items() if c['kind'].startswith('DFF') and n.startswith(POLICY + 'snapshot_word_')}
    assert len(prefetch) == 24, ('actual prefetch width', len(prefetch))
    pointers = {}
    for stem in ('work_slot', 'live0', 'live1', 'live2', 'live3', 'live4',
                 'frozen0', 'frozen1', 'frozen2', 'frozen3', 'frozen4'):
        group = {n: c for n, c in ring.items() if c['kind'].startswith('DFF') and n.startswith(OBS + 'g_ring.' + stem + '_')}
        assert len(group) <= 3, ('pointer wider than source slot', stem, len(group))
        pointers[stem] = [dict(**c, parameters=parameters(c)) for c in group.values()]
    @lru_cache(None)
    def cone(net):
        d = drivers.get(net, [])
        if not d:
            return frozenset(), frozenset(), 0
        assert len(d) == 1, ('prefetch unique driver', net, d)
        n, pin, bit = d[0]
        c = cells[n]
        if c['kind'].startswith('RAM16'):
            assert pin == 'DO'
            return frozenset(), frozenset([n]), 0
        if c['kind'].startswith('DFF'):
            return frozenset(), frozenset(), 0
        if c['kind'] == 'MULT12X12':
            p = parameters(c)
            assert all(p.get(k + '_CLK', '"BYPASS"') == '"BYPASS"'
                       for k in ('AREG', 'BREG', 'PREG', 'OREG')), ('registered DSP in prefetch combinational cone', n, p)
            children = [cone(v) for pin in ('A', 'B') for v in c['ports'][pin]
                        if v not in ('GND', 'VCC')]
            return (frozenset([n]).union(*(x[0] for x in children)),
                    frozenset().union(*(x[1] for x in children)),
                    1 + max((x[2] for x in children), default=0))
        assert c['kind'] in ('INV', 'BUF', 'BUFG', 'ALU', 'GND', 'VCC') or re.fullmatch(r'LUT[1-6]|MUX2_LUT[5-8]', c['kind']), ('prefetch unexpected primitive', n, c['kind'])
        children = [cone(v) for p, ns in c['ports'].items() if directions[c['kind']][p] in ('input', 'inout') for v in ns if v not in ('GND', 'VCC')]
        return (frozenset([n]).union(*(x[0] for x in children)),
                frozenset().union(*(x[1] for x in children)),
                1 + max((x[2] for x in children), default=0))
    cones, used_rams, rows = set(), set(), []
    for n, c in prefetch.items():
        assert len(c['ports']['D']) == 1
        logic, memory, depth = cone(c['ports']['D'][0])
        cones.update(logic); used_rams.update(memory)
        rows.append(dict(register=n, actual_D=c['ports']['D'], asynchronous_RAM_sources=sorted(memory),
                         combinational_primitive_levels=depth))
    assert used_rams, 'prefetch does not consume the actual RAM read outputs'
    read_address_rows = []
    for n, c in rams.items():
        for bit, net in enumerate(c['ports']['RAD']):
            logic, memory, depth = cone(net)
            assert not memory, ('RAM read address depends on RAM output', n, bit)
            read_address_rows.append(dict(RAM=n, pin='RAD', bit=bit, net=net,
                                          combinational_primitive_levels_from_FF_or_input=depth,
                                          combinational_resources=resources({k: cells[k] for k in logic})))
    local_nets = {v for c in ring.values() for p, ns in c['ports'].items() for v in ns if v not in ('GND', 'VCC')}
    related_controls = {v for n, c in {**ring, **prefetch}.items() for p, ns in c['ports'].items()
                        if p in ('CE', 'RESET', 'CLEAR', 'SET', 'PRESET', 'WRE') for v in ns if v not in ('GND', 'VCC')}
    fanout = []
    for net in local_nets | related_controls:
        consumers = [dict(instance=n, pin=p, index=i) for n, p, i in loads.get(net, [])
                     if not re.fullmatch(r'CLK\d*|CLKA|CLKB|CLOCK|WCLK|RCLK', p)]
        if consumers:
            fanout.append(dict(net=net, nonclock_primitive_input_pins=len(consumers),
                               related_memory_write_or_FF_control=net in related_controls, consumers=consumers))
    fanout.sort(key=lambda x: (-x['nonclock_primitive_input_pins'], x['net']))
    return dict(passed=True, source_logical_ring_words=168, source_logical_word_bits=24,
                source_logical_capacity_bits=4032, actual_ring_resources=resources(ring),
                actual_RAMs=ram_rows, actual_allocated_RAM_bits=sum(x['primitive_capacity_bits'] for x in ram_rows),
                actual_observation_address_DSPs=[dict(**c, parameters=parameters(c)) for c in address_dsp.values()],
                DSP_local_registered_boundaries_and_address_semantics_require_region_proof=True,
                identical_write_INIT_groups=[v for v in banks.values()],
                maximum_same_write_port_read_replicas=max(map(len, banks.values())),
                actual_prefetch_FFs=[dict(**c, parameters=parameters(c)) for c in prefetch.values()],
                actual_prefetch_width=24, slot_pointer_FFs=pointers,
                slot_pointer_total_FFs=sum(map(len, pointers.values())),
                prefetch_RAM_read_cones=rows,
                RAM_asynchronous_read_address_cones=read_address_rows,
                address_cones_and_RAM_DO_to_prefetch_are_separate_structural_measurements_not_timing_sum=True,
                prefetch_combinational_union_resources=resources({n: cells[n] for n in cones}),
                maximum_prefetch_combinational_levels=max(x['combinational_primitive_levels'] for x in rows),
                ring_related_nonclock_fanout=fanout,
                highest_actual_ring_related_nonclock_fanout=fanout[0]['nonclock_primitive_input_pins'],
                actual_ring_related_nonclock_nets_over64=[x for x in fanout if x['nonclock_primitive_input_pins'] > 64],
                actual_ring_related_nonclock_nets_over256=[x for x in fanout if x['nonclock_primitive_input_pins'] > 256],
                only24bit_prefetch_output_storage_not4032bit_shift_snapshot=True,
                local_capture_write_read_value_Boolean_proof='Separate region reviewer; not substituted by resource counts.',
                hardware_RAM_initial_value_not_used_as_validity_proof=True,
                physical_read_delay_new_global_assignment_and_CLS_unknown=True)

def timing_and_fanout_summary(ring, listed_paths):
    report = NEW / 'impl/gwsynthesis/g2_top_syn.rpt.html'
    soup = BeautifulSoup(report.read_text(), 'html.parser')
    tables = {}
    for anchor in ('clock', 'performance'):
        tag = soup.find('a', attrs={'name': anchor})
        assert tag is not None
        table = tag.find_parent().find_next('table')
        headers = [x.get_text(' ', strip=True) for x in table.find_all('th')]
        rows = [[x.get_text(' ', strip=True) for x in tr.find_all('td')]
                for tr in table.find_all('tr') if tr.find_all('td')]
        assert rows and all(len(row) == len(headers) for row in rows)
        tables[anchor] = [dict(zip(headers, row)) for row in rows]
    rx = [x for x in tables['clock'] if x['Clock Name'] == 'rgmii_rxc']
    assert len(rx) == 1
    return dict(passed=True, actual_synthesis_clock_summary=tables['clock'],
                actual_synthesis_max_frequency_summary=tables['performance'],
                actual_detailed_listed_paths=[p['summary'] for p in listed_paths],
                actual_listed_path_count=len(listed_paths),
                only_listed_estimates_not_global_routed_STA=True,
                synthesis_RX_period_ns=float(rx[0]['Period']),
                actual_frozen_SDC_RX_target_period_ns=10,
                RX_synthesis_report_constraint_does_not_prove100MHz=True,
                listed_ring_prefetch_endpoint_paths=[p['summary'] for p in listed_paths
                    if 'snapshot_word' in p['summary']['To']],
                ring_specific_delay_ns_not_listed_in_available_report=True,
                prefetch_D_cone_primitive_depth=ring['maximum_prefetch_combinational_levels'],
                RAM_RAD_cone_primitive_depth=max(x['combinational_primitive_levels_from_FF_or_input']
                                               for x in ring['RAM_asynchronous_read_address_cones']),
                primitive_depth_is_not_ns_or_timing_margin=True,
                generated_ring_net_max_nonclock_fanout=ring['maximum_explicit_generated_ring_net_nonclock_fanout'],
                generated_ring_nets_over256=[x for x in ring['ring_related_nonclock_fanout']
                    if x['explicit_generated_ring_net_name'] and x['nonclock_primitive_input_pins'] > 256],
                existing_shared_high_fanout_nets=[{k: v for k, v in x.items() if k != 'consumers'}
                    for x in ring['ring_related_nonclock_fanout']
                    if not x['explicit_generated_ring_net_name'] and x['nonclock_primitive_input_pins'] > 256],
                no_claim_that_all_existing_high_fanout_nets_disappeared=True,
                actual_report_binding=metadata(report))

def run(out, prepare_only):
    out.mkdir(exist_ok=False)
    source = source_review()
    save(out / 'source_binding.json', source)
    if prepare_only:
        result = dict(source_checks_passed=True, actual_mapping_reviewed=False,
                      actual_complete_review_pending=True, actual_job=load(NEW / 'job.json'),
                      source=source, automatic_PNR=False, new_EDA_started=False)
        save(out / 'prepared_review_entry.json', result)
        return result
    job = load(NEW / 'job.json')
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0, ('actual SYN39 incomplete', job['phase'])
    watched = [OLD / 'impl/gwsynthesis/g2_top.vg', NEW / 'impl/gwsynthesis/g2_top.vg', NEW / 'impl/gwsynthesis/g2_top_syn_rsc.xml', LIB, RADAR]
    binding = [metadata(p) for p in watched]
    print('parse complete actual SYN37 and SYN39', flush=True)
    om, nm = definitions(watched[0]), definitions(watched[1])
    before, after = read(watched[0]), read(watched[1])
    op, np = parameter_reader(om), parameter_reader(nm)
    directions = primitive_directions(LIB)
    od, ol = index(before, directions); nd, nl = index(after, directions)
    oc, nc = clock_reader(before, od), clock_reader(after, nd)
    oi, ocounts = clock_inventory(before, oc); ni, ncounts = clock_inventory(after, nc)
    assert {x['root'] for x in ncounts} == {'clk50', 'model25', 'rgmii_rxc'}
    assert all(oi[k] == ni[k] for k in oi.keys() & ni.keys())
    plls = {n for n, c in before.items() if c['kind'] == 'PLL'}
    assert plls == {n for n, c in after.items() if c['kind'] == 'PLL'} == {'u_model_pll/PLL_inst'}
    for n in plls:
        assert before[n] == after[n] and op(before[n]) == np(after[n])
    save(out / 'actual_three_clocks_PLL.json', dict(passed=True, before_counts=ocounts, after_counts=ncounts,
         all_common_clock_pin_roots_and_phase_equal=True, actual_after_clock_pins=len(ni),
         PLL_all_ports_and_explicit_parameters_identical=True,
         PLL_actual=after['u_model_pll/PLL_inst'], PLL_parameters=np(after['u_model_pll/PLL_inst']),
         removed_clock_pins=[dict(instance=k[0], pin=k[1], bit=k[2], root=oi[k]) for k in sorted(oi.keys()-ni.keys())],
         added_clock_pins=[dict(instance=k[0], pin=k[1], bit=k[2], root=ni[k]) for k in sorted(ni.keys()-oi.keys())],
         physical_global_assignment_delay_and_timing_verified=False))
    # Carry the existing INIT1 cold-start/stop guard proof only through exact
    # actual module and instance-port correspondence, not source identity alone.
    guards = ('g2_model25_guard', 'safety_guard')
    assert all(om[k] == nm[k] for k in guards), 'actual health/safety guard module changed'
    targets = ['u_control/model_clock_guard', 'u_model_pll']
    gb, ga = module_bindings(om, targets), module_bindings(nm, targets)
    assert all(gb[n]['ports'] == ga[n]['ports'] for n in targets)
    cold = BASE / 'model25_SYN34_actual_review1/actual_review4/actual_guard_cold_start_and_INIT1_proof.json'
    assert load(cold)['passed']
    guard_review = dict(passed=True, actual_guard_and_NAV_safety_definitions_identical=True,
                        exact_actual_guard_and_PLL_instance_ports_equal=True,
                        guarded_modules=list(guards), guarded_actual_instances=targets,
                        inherited_cold_INIT1_proof=str(cold),
                        inherited_scope='Unchanged actual cold-clear/health-stop guard definitions and exact guard/PLL binding; new ring RAM validity requires its separate arbitrary-initial-state and capture proof.',
                        common_hardware_reset_stop_sources_unchanged=True,
                        physical_reset_release_pulse_and_whole_vehicle_stop_not_signed_off=True)
    save(out / 'actual_guard_stop_inheritance.json', guard_review)
    print('inspect all memory parameters INIT and original CPU image', flush=True)
    oldmem, newmem = memories(before, op), memories(after, np)
    ringmem = {n for n in newmem if n.startswith(OBS + 'g_ring.')}
    assert set(oldmem) == set(newmem) - ringmem, ('nonring memory names changed', sorted(set(oldmem)-set(newmem)), sorted(set(newmem)-set(oldmem)-ringmem))
    port_changes = []
    for n, a in oldmem.items():
        b = newmem[n]
        assert a['kind'] == b['kind'] and a['parameters'] == b['parameters'], ('nonring memory kind/all INIT parameters changed', n)
        assert a['ports'].keys() == b['ports'].keys()
        for pin in a['ports']:
            assert len(a['ports'][pin]) == len(b['ports'][pin])
            if 'CLK' in pin:
                assert [oc(v) for v in a['ports'][pin]] == [nc(v) for v in b['ports'][pin]], ('nonring memory clock changed', n, pin)
        if a['ports'] != b['ports']:
            port_changes.append(dict(name=n, ports={p: dict(before=a['ports'][p], after=b['ports'][p]) for p in a['ports'] if a['ports'][p] != b['ports'][p]}))
    libtext = LIB.read_text()
    spbody = re.search(r'\bmodule\s+SP\b(.*?)\bendmodule', libtext, re.S)[1]
    defaults = dict(re.findall(r'\bparameter\s+(INIT_RAM_\w+)\s*=\s*([^;]+);', spbody))
    assert len(defaults) == 64
    oldimage, newimage = cpu_image(oldmem, defaults), cpu_image(newmem, defaults)
    binary = (NEW / 'firmware/nav_fw.bin').read_bytes()
    wordimage = b''.join(int(x, 16).to_bytes(4, 'little') for x in (NEW / 'ram_w.hex').read_text().split())
    assert len(binary) == 51712 and len(newimage) == 131072
    assert newimage == oldimage == wordimage == binary + bytes(131072-len(binary))
    for lane in range(4):
        assert bytes(int(x, 16) for x in (NEW / f'ram_b{lane}.hex').read_text().split()) == newimage[lane::4]
    assert sum(c['kind'] in BSRAM for c in after.values()) == 314
    save(out / 'actual_nonring_memory_INIT_CPU.json', dict(passed=True, unchanged_nonring_memory_leaves=len(oldmem),
         all_nonring_primitive_kinds_all_parameters_INIT_equal=True, new_ring_RAM_leaves=len(ringmem),
         BSRAM=314, wide32_36_leaves=sum(c['kind'] in BSRAM and any(np(c).get(k) in ('32', '36') for k in ('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1')) for c in after.values()),
         CPU_SP_bitplanes=64, CPU_reconstructed_bytes=131072, original_firmware_bytes=51712,
         CPU_BIN_zero_padding_wordHEX_four_lanes_equal=True, memory_port_changes=port_changes,
         all_changed_memory_data_port_logic_equivalence_proved=not bool(port_changes),
         original_nonring_memory_metadata_only=[dict(name=n, kind=c['kind'], parameters=c['parameters']) for n, c in oldmem.items()]))
    print('strict original26 and actual ring topology', flush=True)
    radar = revalidate(before, after, load(RADAR), directions, om, nm)
    save(out / 'actual_original26.json', radar)
    oldgroups, newgroups = group_ffs(before), group_ffs(after)
    assert {k: len(v) for k, v in oldgroups.items()} == dict(history_flat=3332, current_flat=560, words=672, snapshot=4032)
    assert all(not v for v in newgroups.values()), ('legacy wide FF storage survived', {k: len(v) for k, v in newgroups.items()})
    ring = ring_structure(after, np, directions, nd, nl, nc)
    for row in ring['ring_related_nonclock_fanout']:
        row['same_named_net_before_nonclock_primitive_input_pins'] = sum(
            not re.fullmatch(r'CLK\d*|CLKA|CLKB|CLOCK|WCLK|RCLK', p)
            for n, p, i in ol.get(row['net'], []))
        row['explicit_generated_ring_net_name'] = row['net'].startswith(OBS + 'g_ring.')
    ring['maximum_explicit_generated_ring_net_nonclock_fanout'] = max(
        (x['nonclock_primitive_input_pins'] for x in ring['ring_related_nonclock_fanout']
         if x['explicit_generated_ring_net_name']), default=0)
    ring['existing_shared_reset_controls_not_called_new_high_fanout'] = True
    save(out / 'new_ring_actual_structure.json', ring)
    oldr, newr = hierarchy_resources(OLD/'impl/gwsynthesis/g2_top_syn_rsc.xml'), hierarchy_resources(NEW/'impl/gwsynthesis/g2_top_syn_rsc.xml')
    old_paths, new_paths = report_paths(OLD), report_paths(NEW)
    timing = timing_and_fanout_summary(ring, new_paths)
    save(out / 'actual_ring_fanout_and_timing_summary.json', timing)
    result = dict(passed=True, recorded_at=datetime.now().astimezone().isoformat(), source=source,
         actual_complete_SYN39_exit=0, actual_source_FF_resources_before=resources(before), actual_source_FF_resources_after=resources(after),
         global_resource_reports_before=oldr['g2_top'], global_resource_reports_after=newr['g2_top'],
         actual_old_named_FFs={k: len(v) for k, v in oldgroups.items()}, actual_old8596FFs_removed=True,
         actual_new_ring_resources=ring['actual_ring_resources'], actual_ring_RAM_bits=ring['actual_allocated_RAM_bits'],
         actual_prefetch_FFs=24, actual_slot_pointer_FFs=ring['slot_pointer_total_FFs'],
         actual_maximum_ring_related_nonclock_fanout=ring['highest_actual_ring_related_nonclock_fanout'],
         actual_nonclock_ring_nets_over256=len(ring['actual_ring_related_nonclock_nets_over256']),
         three_real_clock_roots_and_PLL_preserved=True, original26_strict_rechecked=True,
         guard_stop_inheritance=guard_review,
         original_nonring_RAM_all_parameters_INIT_and_CPU_preserved=True,
         all_new_capture_lane_write_and_prefetch_Boolean_semantics='Requires separate region proof, not replaced by this topology/resource audit.',
         old_reported_paths=old_paths, new_reported_paths=new_paths,
         new_actually_reported_negative_estimates=[r for r in new_paths if float(r['summary']['Slack']) < 0],
         complete_negative_endpoint_coverage_verified=False, actual_physical_CLS_and_routing_or_timing_unknown=True,
         net_FF_LUT_gain_not_equal8596_old_storage_bits=True, no_predicted72percent_CLS_claim=True,
         automatic_PNR=False, new_EDA_started=False, metadata_bindings=binding,
         inputs_unchanged_at_end=all(metadata(Path(row['path'])) == row for row in binding))
    assert result['inputs_unchanged_at_end']
    save(out / 'result.json', result)
    return result

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--prepare-only', action='store_true')
    args = ap.parse_args()
    try:
        result = run(args.out, args.prepare_only)
        print(json.dumps({k: v for k, v in result.items() if k in ('passed', 'source_checks_passed', 'actual_mapping_reviewed', 'actual_old8596FFs_removed', 'actual_new_ring_resources', 'actual_maximum_ring_related_nonclock_fanout')}, ensure_ascii=False), flush=True)
    except Exception as error:
        if args.out.exists():
            save(args.out / 'review_failed.json', dict(passed=False, error=str(error), traceback=traceback.format_exc(), recorded_at=datetime.now().astimezone().isoformat()))
        raise
