"""Review completed SYN25: routed fusion-only plus exact action widths.

The source and finite behavioural proof are frozen separately. This reviewer
binds that source to the actual complete netlist, every memory geometry/INIT,
the two real clock roots, and all 26 original first-stage CDC structures.
No timing exceptions are added and this cannot authorize deployment.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
from functools import lru_cache
import gc
import json
from pathlib import Path
import re
import shutil
import sys
import time
import traceback
import xml.etree.ElementTree as ET

import psutil

from audit_netlist_memories import BSRAM
from audit_routing_topology import primitive_directions
from prepare_no_sfp_clock_plan import construct_shared_logic
from prove_soc_console_remap import z3
from revalidate_lidar_cdc import definitions
from retry_frozen_synthesis import memory_status
from review_small_block_stores import parameter_reader
from synthesis_connectivity import read

ORIGINAL = Path('F:/nav_integration_g2_20261023/baseline_fusion_only_syn23')
BASELINE = Path('F:/nav_integration_g2_20261023/baseline_fusion_only_pnr32')
REFERENCE = Path('F:/nav_integration_g2_20261023/pnr27_lidar_cdc_hold_candidate1/review_result.json')
LIBRARY = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
GIB = 1024**3
EXPECTED_MODIFIED_RTL = ['control/policy_action_map.v']


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def save(path, value):
    path = Path(path)
    temporary = path.with_name(path.name+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    temporary.replace(path)


def metadata(path):
    path = Path(path).resolve(); s = path.stat()
    return dict(path=str(path), bytes=s.st_size, mtime_ns=s.st_mtime_ns)


def same(a, b):
    if a.stat().st_size != b.stat().st_size:
        return False
    with a.open('rb') as first, b.open('rb') as second:
        while True:
            x, y = first.read(65536), second.read(65536)
            if x != y:
                return False
            if not x:
                return True


def hardware(root):
    names = [Path(p).relative_to(root).as_posix() for p in load(root/'manifest.json')['source_files']]
    assert len(names) == len(set(names)) == 63
    return set(names)


def snapshot(vg, directions):
    cells = read(vg)
    params = parameter_reader(definitions(vg))
    memory = {n: dict(name=n, kind=c['kind'], ports=c['ports'], parameters=params(c))
              for n, c in cells.items() if c['kind'] in BSRAM or c['kind'].startswith(('RAM16', 'ROM16'))}
    drivers = defaultdict(list)
    for n, c in cells.items():
        assert c['kind'] in directions, ('unclassified primitive', c['kind'])
        for pin, nets in c['ports'].items():
            assert pin in directions[c['kind']], ('unclassified pin', n, pin)
            if directions[c['kind']][pin] == 'output':
                for net in nets:
                    if net not in ('GND', 'VCC'):
                        drivers[net].append((n, pin))

    @lru_cache(None)
    def clock(net):
        if net in ('GND', 'VCC'):
            return net, 0
        assert len(drivers[net]) == 1, ('clock driver not unique', net, drivers[net])
        n, pin = drivers[net][0]; c = cells[n]
        if c['kind'] == 'IBUF':
            assert c['ports']['I'] in (['clk50'], ['rgmii_rxc'])
            return c['ports']['I'][0], 0
        assert c['kind'] in ('INV', 'BUF', 'BUFG'), ('unexpected derived clock', net, c)
        parent, inversions = clock(c['ports']['I'][0])
        return parent, inversions ^ int(c['kind'] == 'INV')

    clocks = Counter(); nets = {}
    for c in cells.values():
        for pin, values in c['ports'].items():
            if pin == 'CLK' or re.fullmatch(r'CLK\d+|WCLK|RCLK', pin):
                for net in values:
                    if net not in ('GND', 'VCC'):
                        root = clock(net); clocks[root] += 1
                        nets[net] = dict(net=net, root=root[0], inverted=bool(root[1]), driver=cells[drivers[net][0][0]])
    assert {root for root, parity in clocks} == {'clk50', 'rgmii_rxc'}
    q = {net: c for c in cells.values() if c['kind'].startswith('DFF') for net in c['ports'].get('Q', [])}
    edges = []
    for c in cells.values():
        if not c['kind'].startswith('DFF'):
            continue
        for net in c['ports'].get('D', []):
            source = q.get(net)
            if source and source['ports']['CLK'] != c['ports']['CLK']:
                edges.append((source['name'], c['name'], tuple(source['ports']['CLK']), tuple(c['ports']['CLK'])))
    counts = Counter(c['kind'] for c in cells.values())
    bsram = {n:m for n,m in memory.items() if m['kind'] in BSRAM}
    wide = [n for n,m in bsram.items() if any(m['parameters'].get(k) in ('32','36') for k in ('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1'))]
    inventory = dict(actual_primitive_counts=dict(counts), source_BSRAM=len(bsram), wide_leaves=len(wide),
                     predicted_physical_BSRAM_with_split1=len(bsram)+len(wide),
                     BSRAM_by_top=dict(Counter(n.split('/')[0] for n in bsram)),
                     clock_pin_counts=[dict(root=k[0],inverted=bool(k[1]),pins=v) for k,v in clocks.items()],
                     clock_net_drivers=list(nets.values()), direct_CDC_edges=sorted(edges))
    return cells, params, memory, drivers, clock, inventory


def cpu_image(memory, zero_defaults):
    result = bytearray(131072); seen = set()
    for n, row in memory.items():
        if not n.startswith('u_soc/g_lane['):
            continue
        match = re.fullmatch(r'u_soc/g_lane\[([0-3])\]\.u_ram/mem_mem_([01])_([0-7])_s', n)
        assert match and row['kind'] == 'SP' and row['parameters']['BIT_WIDTH'] == '1', ('CPU bitplane differs', n)
        lane, bank, bit = map(int, match.groups())
        assert (lane,bank,bit) not in seen; seen.add((lane,bank,bit))
        vector = b''
        for i in range(64):
            name = f'INIT_RAM_{i:02X}'
            raw = row['parameters'].get(name, zero_defaults[name]).strip()
            literal = re.fullmatch(r"256'h([0-9a-fA-F]+)", raw)
            assert literal, ('CPU INIT encoding', n, name, raw)
            vector += int(literal[1],16).to_bytes(32,'little')
        for j, byte in enumerate(vector):
            for k in range(8):
                result[(bank*16384+j*8+k)*4+lane] |= ((byte>>k)&1)<<bit
    assert len(seen) == 64
    return bytes(result)


def bind_action_contract(source, prepared, proof):
    """Bind the only RTL delta to the preserved exact-value/cycle proof."""
    import re
    proofdir = Path('F:/nav_integration_g2_20261022/action_map_exact_widths_equivalence')
    assert Path(proof['source']).resolve() == ORIGINAL.resolve()
    assert Path(proof['routed_reference']).resolve() == BASELINE.resolve()
    assert proof['only_private_scale_function_changed']
    assert proof['no_POSE_weight9_or_other_donor_change_adopted']
    assert proof['all_original_firmware_and_build_timestamp_bytes_identical']
    assert proof['external_registers_validity_stop_and_reset_cycles_preserved']
    for record in proof['bindings']:
        assert metadata(record['path']) == record, ('prepared proof input changed', record['path'])
    action = 'control/policy_action_map.v'
    assert same(proofdir/'action_candidate.v', prepared/action)
    assert same(proofdir/'action_candidate.v', source/action)
    original = (ORIGINAL/action).read_text(encoding='utf-8')
    reference = (proofdir/'action_original.v').read_text(encoding='utf-8').replace(
        'module action_map_original #(', 'module policy_action_map #(', 1)
    assert original == reference
    candidate = (source/action).read_text(encoding='utf-8')
    function = re.compile(r'    function signed \[31:0\] scale;.*?    endfunction',re.S)
    assert len(function.findall(original)) == len(function.findall(candidate)) == 1
    assert function.sub('PRIVATE_SCALE',original) == function.sub('PRIVATE_SCALE',candidate)
    assert re.findall(r'scale\(actions\[[^\]]+\],(-?\d+)\)', candidate) == ['-600','600','19','-19','19','19']
    actual = load(proofdir/'result.json')
    assert actual == proof['reused_proof']
    assert actual['pass_'] and actual['frames'] == 8770 and actual['clip_domain_values_per_gain'] == 8193
    assert actual['outputs_last_actions_and_validity_cycle_exact']
    assert actual['invalid_cycles'] == actual['run_stop_cycles'] == 64
    assert actual['reset_scenarios'] == 1 and actual['parameter_sets'] == 2
    log = (proofdir/'xsim.log').read_text(encoding='utf-8',errors='replace')
    assert 'TEST PASSED' in log and 'Fatal:' not in log
    for root in (prepared,source):
        manifest = load(root/'manifest.json')
        assert manifest['fusion_norm_pipeline'] and manifest['action_map_exact_widths']
        assert not manifest.get('pose_predecode') and not manifest['control_weights_changed']
    assert same(ORIGINAL/'impl/gwsynthesis/g2_top.vg', BASELINE/'g2_top.vg')


def review(source, prepared, out):
    source, prepared, out = [Path(p).resolve() for p in (source, prepared, out)]
    assert source != prepared and source != ORIGINAL
    result = dict(schema_version=1, passed=False, source=str(source), prepared=str(prepared),
                  baseline=str(BASELINE), actual_vg=str(source/'impl/gwsynthesis/g2_top.vg'),
                  scope='Exploratory complete-SYN source/RAM/INIT/2clock/26CDC preflight; not physical timing, whole-system equivalence or deployment acceptance',
                  diagnostic_only=True, functional_deployment_acceptance=False, deployment_accepted=False, programmed=False,
                  automatic_PNR=False, recorded_at=datetime.now().astimezone().isoformat())
    progress = out.with_name(out.stem+'_progress.json')
    def checkpoint(phase, **fields):
        result.update(phase=phase, updated_at=datetime.now().astimezone().isoformat(), **fields)
        save(progress, result)

    checkpoint('waiting_for_actual_netlist_review_memory')
    while True:
        memory = memory_status()
        checkpoint('waiting_for_actual_netlist_review_memory', available_physical_bytes=memory.avail_phys,
                   available_commit_bytes=memory.avail_page, minimum_physical_bytes=2*GIB, minimum_commit_bytes=4*GIB)
        if memory.avail_phys >= 2*GIB and memory.avail_page >= 4*GIB:
            break
        time.sleep(10)
    checkpoint('binding_completed_SYN_to_exact_prepared_source')
    prepared_proof = load(prepared/'prepared_source_review.json')
    assert prepared_proof['passed']
    expected = sorted(prepared_proof['modified_rtl_inputs'])
    assert expected == sorted(EXPECTED_MODIFIED_RTL)
    bind_action_contract(source, prepared, prepared_proof)
    assert hardware(ORIGINAL) == hardware(prepared) == hardware(source)
    changed = sorted(n for n in hardware(source) if not same(ORIGINAL/n, source/n))
    assert changed == expected
    assert all(same(prepared/n, source/n) for n in hardware(source))
    frozen = [metadata(source/n) for n in sorted(hardware(source))]
    assets = {}; headers = {}
    for p in ORIGINAL.rglob('*'):
        if not p.is_file() or 'impl' in p.relative_to(ORIGINAL).parts:
            continue
        relative = p.relative_to(ORIGINAL)
        if p.suffix.lower() in {'.hex','.mem','.bin','.mif'}:
            assets[relative.as_posix()] = p
        if p.suffix.lower() in {'.vh','.h'}:
            headers[relative.as_posix()] = p
    assert len(assets) == 69 and len(headers) == 31
    assert all(same(p, prepared/n) and same(p, source/n) for n,p in {**assets,**headers}.items())
    firmware_counts = {}
    excluded_host_bytecode = []
    for directory in ('fw','firmware'):
        # retry_frozen_synthesis deliberately excludes __pycache__ everywhere.
        # These interpreter caches are neither firmware source nor HW assets.
        excluded_host_bytecode.extend(str(p.relative_to(ORIGINAL)) for p in (ORIGINAL/directory).rglob('*')
                                      if p.is_file() and '__pycache__' in p.relative_to(ORIGINAL).parts)
        paths = [p for p in (ORIGINAL/directory).rglob('*') if p.is_file()
                 and '__pycache__' not in p.relative_to(ORIGINAL).parts]
        for p in paths:
            relative = p.relative_to(ORIGINAL)
            assert same(p, prepared/relative) and same(p, source/relative)
        firmware_counts[directory] = len(paths)
    for name in ('g2.cst','g2.sdc'):
        assert same(ORIGINAL/name, BASELINE/name) and same(ORIGINAL/name, prepared/name) and same(ORIGINAL/name, source/name)
    sdc = (source/'g2.sdc').read_text(encoding='utf-8')
    commands = [x.strip() for x in sdc.splitlines() if x.strip() and not x.lstrip().startswith('#')]
    assert commands == ['create_clock -name clk50 -period 20 [get_ports {clk50}]',
                        'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]']
    assert 'CLOCK_LOC ' not in (source/'g2.cst').read_text(encoding='utf-8')
    job = load(source/'job.json')
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0
    log = (source/'gw_sh.log').read_text(encoding='utf-8', errors='replace')
    assert 'GowinSynthesis finish' in log and '[100%] Generate report file' in log
    assert not re.search(r'^.*ERROR\s*\(', log, re.M)
    options = {e.get('type'):e.get('value') for e in ET.parse(source/'impl/gwsynthesis/g2_top.prj').getroot().findall('./OptionList/Option')}
    assert float(options['global_freq']) == 50
    manifest = load(source/'manifest.json')
    assert manifest['wide_split'] == 1 and manifest['clock_core_mhz'] == 50 and not manifest['control_weights_changed']
    assert manifest['effective_options']['global_freq_mhz'] == 50
    reference = load(REFERENCE)
    assert reference['passed'] and len(reference['paths']) == 26
    assert same(ORIGINAL/'impl/gwsynthesis/g2_top.vg', BASELINE/'g2_top.vg')
    checkpoint('parsing_actual_old_and_new_VG_once')
    directions = primitive_directions(LIBRARY)
    before, oldparams, oldmem, olddrivers, oldclock, oldinventory = snapshot(BASELINE/'g2_top.vg', directions)
    after, newparams, newmem, drivers, newclock, inventory = snapshot(source/'impl/gwsynthesis/g2_top.vg', directions)
    assert set(oldmem) == set(newmem), 'actual memory leaf identities changed'
    pin_differences = []
    for n in oldmem:
        x,y = oldmem[n],newmem[n]
        assert x['kind'] == y['kind'] and x['parameters'] == y['parameters'], ('RAM geometry or INIT changed',n)
        assert set(x['ports']) == set(y['ports']), ('memory live pin set changed',n)
        for pin in x['ports']:
            assert len(x['ports'][pin]) == len(y['ports'][pin])
            if 'CLK' in pin:
                assert [oldclock(v) for v in x['ports'][pin]] == [newclock(v) for v in y['ports'][pin]], ('memory clock root/phase changed',n,pin)
        if x['ports'] != y['ports']:
            pin_differences.append(dict(name=n, changed_ports={p:[x['ports'][p],y['ports'][p]] for p in x['ports'] if x['ports'][p]!=y['ports'][p]}))
    assert inventory['source_BSRAM'] == oldinventory['source_BSRAM'] == 314
    assert inventory['wide_leaves'] == oldinventory['wide_leaves'] == 18
    library = LIBRARY.read_text(encoding='utf-8')
    body = re.search(r'\bmodule\s+SP\b(.*?)\bendmodule',library,re.S)[1]
    defaults = dict(re.findall(r'\bparameter\s+(INIT_RAM_\w+)\s*=\s*([^;]+);',body))
    assert len(defaults) == 64 and all(int(re.fullmatch(r"256'h([0-9a-fA-F]+)",v.strip())[1],16)==0 for v in defaults.values())
    image = cpu_image(newmem,defaults); oldimage = cpu_image(oldmem,defaults)
    binary = (source/'firmware/nav_fw.bin').read_bytes()
    words = b''.join(int(x,16).to_bytes(4,'little') for x in (source/'ram_w.hex').read_text().split())
    assert image == oldimage == words == binary+bytes(131072-len(binary))
    for lane in range(4):
        bank = bytes(int(x,16) for x in (source/f'ram_b{lane}.hex').read_text().split())
        assert bank == image[lane::4]
    checkpoint('proving_changed_actual_CDC_inputs_with_real_state_boundaries')
    la,lb,boundaries = construct_shared_logic(before,after,oldparams,newparams,directions)
    checks = []
    def logic(label,a,b):
        delta = z3.simplify(z3.Xor(la.expr(a),lb.expr(b)))
        if z3.is_false(delta):
            method = 'canonical_Boolean_identity'
        else:
            solver = z3.Solver();solver.set(timeout=10000);solver.add(delta)
            status = solver.check()
            assert status == z3.unsat, ('changed actual CDC cone not proved',label,str(status),str(solver.model())[:1800] if status==z3.sat else solver.reason_unknown())
            method = 'SAT_unsatisfiable_miter'
        checks.append(dict(endpoint=label,before=a,after=b,passed=True,method=method))
    first_qs = {after[r['first_register']]['ports']['Q'][0] for r in reference['paths']}
    loads = defaultdict(list)
    for n,c in after.items():
        for pin,values in c['ports'].items():
            if directions[c['kind']][pin] in ('input','inout'):
                for net in values:
                    if net in first_qs:loads[net].append((n,pin))
    paths = []
    for r in reference['paths']:
        record = {k:v for k,v in r.items() if k in ('group','source_register','first_register','second_register','source_clock','target_clock','route_max_delay_ns')}
        for role in ('source_register','first_register','second_register'):
            n=r[role];x,y=before[n],after[n]
            assert x['kind'] == y['kind'] and oldparams(x) == newparams(y), ('CDC kind/INIT changed',n)
            assert x['ports']['CLK'] == y['ports']['CLK']
            assert set(x['ports']) == set(y['ports'])
            for pin in x['ports']:
                assert len(x['ports'][pin]) == len(y['ports'][pin])
                if directions[x['kind']][pin] == 'input' and pin != 'CLK':
                    for i,(a,b) in enumerate(zip(x['ports'][pin],y['ports'][pin])):
                        if a!=b:logic(n+'/'+pin+f'[{i}]',a,b)
            record[role+'_actual'] = dict(primitive=y,parameters=newparams(y))
        src,first,second = [after[r[role]] for role in ('source_register','first_register','second_register')]
        assert src['ports']['Q'] == first['ports']['D'] and len(first['ports']['D'])==1
        assert drivers[first['ports']['D'][0]] == [(src['name'],'Q')]
        assert first['ports']['Q'] == second['ports']['D'] and len(first['ports']['Q'])==1
        assert loads[first['ports']['Q'][0]] == [(second['name'],'D')]
        assert src['ports']['CLK'] == [r['source_clock']]
        assert all(c['ports']['CLK']==[r['target_clock']] and c['ports'].get('CE',['VCC'])==['VCC'] for c in (first,second))
        assert {p:v for p,v in first['ports'].items() if p in ('RESET','CLEAR','SET','PRESET')} == {p:v for p,v in second['ports'].items() if p in ('RESET','CLEAR','SET','PRESET')}
        record['passed'] = True
        paths.append(record)
    assert oldinventory['direct_CDC_edges'] == inventory['direct_CDC_edges']
    assert len(inventory['direct_CDC_edges']) == 26
    assert {(r[0],r[1]) for r in inventory['direct_CDC_edges']} == {(r['source_register'],r['first_register']) for r in reference['paths']}
    resources = ET.parse(source/'impl/gwsynthesis/g2_top_syn_rsc.xml').getroot().attrib
    count = lambda key:int(resources[key].split('(')[0])
    assert count('T_Bsram') == 314
    assert count('T_Register') == sum(v for k,v in inventory['actual_primitive_counts'].items() if k.startswith('DFF'))
    assert count('T_Alu') == inventory['actual_primitive_counts']['ALU']
    evidence = out.parent/(out.stem+'_evidence')
    evidence.mkdir(exist_ok=False)
    save(evidence/'actual_memory_binding.json',dict(passed=True,old_memory=oldmem,new_memory=newmem,
        actual_all_geometry_and_INIT_identical=True,port_name_differences=pin_differences,
        changed_memory_port_logic_proved_by_this_review=False))
    save(evidence/'actual_CDC_binding.json',dict(passed=True,paths=paths,changed_input_logic_checks=checks,
        actual_state_boundary_contract=boundaries,full_sequential_equivalence_claimed=False))
    save(evidence/'actual_resource_and_clock_inventory.json',dict(passed=True,inventory=inventory,vendor_resources=resources))
    save(evidence/'actual_CPU_INIT_binding.json',dict(passed=True,CPU_SP_count=64,INIT_pages_examined=4096,
        reconstructed_bytes=131072,byte_identical_to_routed_fusion_PNR32=True,
        actual_BIN_wordHEX_and_four_laneHEX_bound=True,changed_byte_offsets=[]))
    copies = [(source/'impl/gwsynthesis/g2_top.vg',evidence/'reviewed_g2_top.vg'),
              (source/'g2.cst',evidence/'reviewed_g2.cst'),(source/'g2.sdc',evidence/'reviewed_g2.sdc')]
    for a,b in copies:
        shutil.copyfile(a,b);assert same(a,b)
    assert all(metadata(Path(item['path']))==item for item in frozen)
    result.update(passed=True,phase='actual_netlist_preflight_passed',
        actual_RAM_INIT_clock_26CDC_preflight_passed=True,changed_HW_inputs=changed,
        unchanged_hardware_inputs=63-len(expected),identical_assets=69,identical_headers=31,firmware_files=firmware_counts,
        excluded_host_interpreter_caches=excluded_host_bytecode,
        source_BSRAM=314,wide_leaves=18,predicted_physical_BSRAM=332,physical_BSRAM_not_yet_measured=True,
        CPU_SP_count=64,true_clocks=2,known_CDC_paths=26,actual_clock_roots=['clk50','rgmii_rxc'],
        clock_periods_ns={'clk50':20,'rgmii_rxc':10},added_CDC_timing_exceptions=0,
        actual_all_memory_geometry_and_INIT_identical=True,memory_pin_name_differences=len(pin_differences),
        memory_port_full_logic_equivalence_proved=False,actual_CDC_changed_input_cones_proved=checks,
        actual_direct_CDC_count=26,unproved_CDC_aliases=0,paths=paths,
        actual_complete_SYN=True,actual_synthesis_global_freq_mhz=50,
        vendor_resources=resources,primitive_counts=inventory['actual_primitive_counts'],
        source_prepared_review=str(prepared/'prepared_source_review.json'),
        evidence_directory=str(evidence),reviewed_netlist_copy=str(copies[0][1]),
        reviewed_CST_copy=str(copies[1][1]),reviewed_SDC_copy=str(copies[2][1]),
        reviewed_input_metadata=[metadata(p) for pair in copies for p in pair]+[metadata(BASELINE/'g2_top.vg'),metadata(BASELINE/'g2.cst'),metadata(BASELINE/'g2.sdc')],
        functional_pipeline_contract_bound_to_existing_prepared_proof=True,
        actual_physical_timing_or_CDC_delay_signed_off=False,
        execution_policy=dict(priority=int(psutil.Process().nice()),affinity=psutil.Process().cpu_affinity()))
    save(out,result);save(progress,result)
    del before,after,oldmem,newmem,olddrivers,drivers,la,lb;gc.collect()
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--prepared',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();args.out.parent.mkdir(parents=True,exist_ok=True)
    me=psutil.Process();me.nice(psutil.IDLE_PRIORITY_CLASS);me.cpu_affinity([18])
    try:
        result=review(args.source,args.prepared,args.out)
    except Exception as error:
        result=dict(passed=False,source=str(args.source.resolve()),prepared=str(args.prepared.resolve()),
                    baseline=str(BASELINE),error=str(error),traceback=traceback.format_exc(),
                    programmed=False,deployment_accepted=False,automatic_PNR=False)
        save(args.out,result);print(result['traceback'],flush=True);return 1
    print(json.dumps({k:v for k,v in result.items() if k in ('passed','source','prepared','source_BSRAM','wide_leaves','CPU_SP_count','true_clocks','known_CDC_paths','reviewed_netlist_copy')},ensure_ascii=False),flush=True)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
