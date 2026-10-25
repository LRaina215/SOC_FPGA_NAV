"""Actual completed-SYN preflight for the full-UART, physical-SFP-off experiment.

This makes inputs for exploratory implementation only. It does not run a tool,
claim physical timing, or release UART business/stop behaviour for deployment.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import gc
import json
from pathlib import Path
import re
import shutil
import traceback
import xml.etree.ElementTree as ET

from audit_netlist_memories import BSRAM
from audit_routing_topology import primitive_directions
from audit_cdc_first_stages import sdc_fragment
from prepare_sfp_off_uart import SFP_SOURCES, transform_top
from revalidate_lidar_cdc import definitions
from revalidate_observation_routing import LIBRARY
from review_small_block_stores import parameter_reader
from synthesis_connectivity import read


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def save(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8', newline='\n')


def inputs(root, manifest):
    names = [Path(p).relative_to(root).as_posix() for p in manifest['source_files']]
    assert len(names) == len(set(names))
    return names


def data_files(root, suffixes):
    return {p.relative_to(root).as_posix(): p for p in root.rglob('*') if p.is_file()
            and p.suffix.lower() in suffixes and 'impl' not in p.relative_to(root).parts}


def memory_name(row):
    return row['path'].removeprefix('g2_top/').replace('\\', '')


def direct_edges(cells):
    q = {net: c for c in cells.values() if c['kind'].startswith('DFF')
         for net in c['ports'].get('Q', [])}
    result = []
    for c in cells.values():
        if not c['kind'].startswith('DFF'): continue
        for net in c['ports'].get('D', []):
            src = q.get(net)
            if src and src['ports']['CLK'] != c['ports']['CLK']:
                result.append(dict(source_register=src['name'], target_register=c['name'],
                    source_clock=src['ports']['CLK'], target_clock=c['ports']['CLK']))
    return sorted(result, key=lambda x:(x['source_register'], x['target_register']))


def physical_inputs(build, out, result, cst, reference):
    assert not out.exists(), ('PNR inputs directory already exists; no replacement', str(out))
    # Publish only after every actual check has completed. There is no run.tcl,
    # CSR or SFP CDC reference here, so this preparer cannot invoke a build.
    stage = out.with_name(out.name+'.preflight_tmp')
    assert not stage.exists()
    (stage/'impl/gwsynthesis').mkdir(parents=True)
    for name in ('g2_top.vg', 'g2_top.prj'):
        shutil.copyfile(build/'impl/gwsynthesis'/name, stage/'impl/gwsynthesis'/name)
    for name in ('effective_options.tcl', 'g2.sdc'):
        shutil.copyfile(build/name, stage/name)
    (stage/'g2.cst').write_text(cst, encoding='utf-8', newline='\n')
    save(stage/'cdc_reference_audit.json', reference)
    manifest = load(build/'manifest.json')
    manifest.update(source=str(build), frozen_netlist=True, implementation_preflight_passed=True,
        diagnostic_only=True, real_sfp=False, full_control_telemetry=True,
        source_preparation_only=False, post_SYN_CDC_and_clock_driver_review_pending=False,
        expected_true_clock_domains=['clk50_d', 'rgmii_rxc_d'], known_CDC_constraint_pairs=26,
        full_UART_business_and_disconnect_stop_verified=False, deployment_accepted=False,
        automatic_programming=False, programmed=False, physical_CDC_and_timing_signed_off=False)
    save(stage/'manifest.json', manifest)
    save(stage/'implementation_preflight.json', result)
    stage.rename(out)
    metadata = []
    for path in sorted(out.rglob('*')):
        if path.is_file():
            st = path.stat()
            metadata.append(dict(path=path.relative_to(out).as_posix(), bytes=st.st_size,
                                 modified_at=datetime.fromtimestamp(st.st_mtime).astimezone().isoformat()))
    assert not any(p.name in ('serdes.csr', 'sfp_cdc_reference.json') for p in out.rglob('*'))
    return metadata


def review(build, baseline, prepared, pnr_inputs, progress=None):
    build, baseline, prepared, pnr_inputs = [Path(p).resolve() for p in (build, baseline, prepared, pnr_inputs)]
    r = dict(passed=False, implementation_preflight_passed=False, source=str(build),
             baseline=str(baseline), prepared=str(prepared), created_at=datetime.now().astimezone().isoformat(),
             diagnostic_only=True, deployment_accepted=False, programmed=False)
    def checkpoint(phase):
        r['phase'] = phase
        if progress: save(progress, r)
        print(phase, flush=True)
    manifest, job, prep = load(build/'manifest.json'), load(build/'job.json'), load(prepared/'prepared_source_review.json')
    origin = Path(prep['source']).resolve(); r['origin'] = str(origin)
    original = load(origin/'manifest.json')
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0
    assert prep['passed'] and origin.name == 'sfp_lane2_private_snapshot_syn11_retry1'
    assert Path(job['source']).resolve() == prepared == Path(manifest['frozen_source_build']).resolve()
    assert job['modified_rtl_inputs'] == [] and baseline.name == 'sfp_lane2_observation_lw_pnr15'
    assert (baseline/'g2_top.vg').read_bytes() == (origin/'impl/gwsynthesis/g2_top.vg').read_bytes()
    report = (baseline/'impl/pnr/g2_top.rpt.txt').read_text(encoding='utf-8')
    assert re.search(r'PR0004\).*total 184 unrouted nets', report) and re.search(r'BSRAM\s+\|\s+326/340', report)
    old_names, names = inputs(origin, original), inputs(build, manifest)
    assert len(old_names) == 68 and len(names) == 64 and set(names) == set(old_names)-SFP_SOURCES
    assert inputs(prepared, load(prepared/'manifest.json')) == names
    changes = [n for n in names if (origin/n).read_bytes() != (build/n).read_bytes()]
    assert changes == ['g2_top.v']
    assert (build/'g2_top.v').read_text(encoding='utf-8') == transform_top((origin/'g2_top.v').read_text(encoding='utf-8'))
    assert all((build/n).read_bytes() == (prepared/n).read_bytes() for n in names)
    assert not any((build/n).exists() for n in SFP_SOURCES | {'ip/serdes.csr', 'sfp_cdc_reference.json'})
    for flag in ('coverage_enabled', 'command_lease', 'observation_pipeline', 'packed_weight9',
                 'slf_point_planes', 'snapshot_private_reset', 'full_control_telemetry'):
        assert manifest[flag] == original[flag] and manifest[flag]
    assert not manifest['real_sfp'] and not manifest['control_weights_changed']
    assert manifest['host_extension_enable'] == 1 and manifest['clock_core_mhz'] == 50
    assets = data_files(origin, {'.hex','.mem','.bin','.mif'})
    new_assets = data_files(build, {'.hex','.mem','.bin','.mif'})
    assert assets.keys() == new_assets.keys() and len(assets) == 104
    assert all(path.read_bytes() == new_assets[n].read_bytes() for n,path in assets.items())
    headers = data_files(origin, {'.vh','.h'}); new_headers = data_files(build, {'.vh','.h'})
    assert headers.keys() == new_headers.keys()
    assert all(path.read_bytes() == new_headers[n].read_bytes() for n,path in headers.items())
    assert (build/'g2.sdc').read_bytes() == (prepared/'g2.sdc').read_bytes()
    assert (build/'g2.cst').read_bytes() == (prepared/'g2.cst').read_bytes()
    io_lines = lambda p:[l for l in p.read_text(encoding='utf-8').splitlines() if l.startswith(('IO_LOC ', 'IO_PORT '))]
    assert io_lines(build/'g2.cst') == io_lines(baseline/'g2.cst')
    project = ET.parse(build/'impl/gwsynthesis/g2_top.prj').getroot()
    actual_files = {Path(x.get('path')).resolve() for x in project.findall('./FileList/File') if x.get('type') == 'verilog'}
    assert actual_files == {build/n for n in names}
    opts = {x.get('type'):x.get('value') for x in project.findall('./OptionList/Option')}
    assert float(opts['global_freq']) == 50 and project.find('Device').get('id') == 'GW5AT-138C'
    summary = load(build/'synthesis_summary.json'); assert not summary['errors']
    r.update(modified_rtl_inputs=changes, removed_hardware_inputs=sorted(SFP_SOURCES),
        retained_hardware_inputs=64, unchanged_retained_inputs=63, unchanged_assets=104,
        unchanged_headers=len(headers), source_top_exact_transform=True, original_IO_constraints_preserved=True,
        original_4500_weights_125Hz_CPU128KiB_NAV_capacity_and_stop_sources_preserved=True,
        HOST_EXTENSION_ENABLE=1, live_UART_RX_TX_INJ_TXshadow_console_sources_retained=True)
    checkpoint('source_assets_and_actual_project_passed')
    old_inv, inv = load(origin/'memory_inventory.json'), load(build/'memory_inventory.json')
    old_ram, new_ram = {memory_name(x):x for x in old_inv['leaves']}, {memory_name(x):x for x in inv['leaves']}
    removed = sorted(old_ram.keys()-new_ram.keys())
    assert removed == [f'u_nav/u_neth/u_rxf/mem_mem_0_{i}_s' for i in range(3)]
    assert not new_ram.keys()-old_ram.keys()
    stub_tx = 'u_nav/u_neth/u_txf/mem_mem_0_0_s'
    differing = sorted(n for n in new_ram if (old_ram[n]['primitive'], old_ram[n]['parameters']) != (new_ram[n]['primitive'],new_ram[n]['parameters']))
    assert differing == [stub_tx]
    assert (old_ram[stub_tx]['primitive'], new_ram[stub_tx]['primitive']) == ('SDPX9B','SDPB')
    assert old_ram[stub_tx]['parameters']['BIT_WIDTH_0'] == '9' and new_ram[stub_tx]['parameters']['BIT_WIDTH_0'] == '1'
    assert inv['expanded_bsram_count'] == 313 and inv['wide32_36_count'] == 10 and inv['predicted_split_bsram'] == 323
    assert inv['by_top_instance'] == {'u_soc':64,'u_control':43,'u_nav':206}
    old_vg, new_vg = origin/'impl/gwsynthesis/g2_top.vg', build/'impl/gwsynthesis/g2_top.vg'
    before, after = read(old_vg), read(new_vg)
    old_defs, new_defs = definitions(old_vg), definitions(new_vg)
    oldparams, newparams = parameter_reader(old_defs), parameter_reader(new_defs)
    assert {n for n,c in after.items() if c['kind'] in BSRAM} == set(new_ram)
    for n in new_ram.keys()-{stub_tx}:
        assert oldparams(before[n]) == newparams(after[n]), ('preserved memory INIT/parameters changed',n)
        for pin in before[n]['ports']:
            if 'CLK' in pin:
                assert before[n]['ports'][pin]==after[n]['ports'][pin], ('preserved memory clock changed',n,pin)
    # The disconnected physical-host FIFO's data are constant zero; retain an
    # explicit zero INIT check for its remaining one-bit bookkeeping storage.
    init = {k:v for k,v in newparams(after[stub_tx]).items() if k.startswith('INIT_RAM')}
    body=re.search(r'\bmodule\s+SDPB\b(.*?)\bendmodule\b',LIBRARY.read_text(encoding='utf-8'),re.S)[1]
    defaults=dict(re.findall(r"parameter\s+(INIT_RAM_[0-9A-F]{2})\s*=\s*256'h([0-9a-fA-F]+)\s*;",body))
    assert set(defaults)=={f'INIT_RAM_{i:02X}' for i in range(64)} and all(int(v,16)==0 for v in defaults.values())
    assert set(init)<=set(defaults)
    for k,v in init.items():
        assert re.fullmatch(r"256'h0+",v), ('stub FIFO nonzero INIT',k,v)
    mailbox_counts = {}
    for prefix in ('u_nav/u_mbx/u_injb/', 'u_nav/u_mbx/u_rxb/'):
        a = {n:c for n,c in before.items() if n.startswith(prefix) and c['kind'].startswith(('RAM','SDP','SP','DP'))}
        b = {n:c for n,c in after.items() if n.startswith(prefix) and c['kind'].startswith(('RAM','SDP','SP','DP'))}
        assert a.keys() == b.keys() and a, ('full mailbox memory missing',prefix)
        assert all(a[n]['kind'] == b[n]['kind'] and oldparams(a[n]) == newparams(b[n]) for n in a)
        mailbox_counts[prefix] = dict(leaves=len(a), kinds=dict(Counter(c['kind'] for c in a.values())), geometry_and_INIT_preserved=True)
    shadow_RAD=[f'u_nav/r_MBX_TXB_ADDR_Z[{i}]' for i in (3,2,1,0)]
    old_shadow = {n:c for n,c in before.items() if n.startswith('u_nav/u_mbx/u_txs/')
                  and c['kind']=='RAM16SDP4' and c['ports']['RAD']==shadow_RAD}
    new_shadow = {n:c for n,c in after.items() if n.startswith('u_nav/u_mbx/u_txs/') and c['kind']=='RAM16SDP4'}
    assert len(old_shadow)==len(new_shadow)==256
    # Some generated copies use an offset bank suffix; match actual write
    # slice and DI rather than infer identity from this candidate suffix.
    old_shadow_rows=list(old_shadow.values()); new_shadow_rows=list(new_shadow.values())
    for c in new_shadow_rows:
        assert c['ports']['RAD']==shadow_RAD and c['ports']['CLK']==['clk50_d']
        assert set(c['ports'])=={'CLK','WRE','WAD','DI','RAD','DO'}
    assert Counter(tuple(c['ports']['DI']) for c in old_shadow_rows)==Counter(tuple(c['ports']['DI']) for c in new_shadow_rows)
    assert Counter(tuple(c['ports']['WAD']) for c in old_shadow_rows)==Counter(tuple(c['ports']['WAD']) for c in new_shadow_rows)
    assert Counter(tuple(sorted(oldparams(c).items())) for c in old_shadow_rows)==Counter(tuple(sorted(newparams(c).items())) for c in new_shadow_rows)
    mailbox_counts['u_nav/u_mbx/u_txs/']=dict(old_shadow_leaves=256,new_shadow_leaves=256,
        actual_shadow_read_address=shadow_RAD,write_coordinates_and_INIT_multiset_preserved=True,
        discarded_disconnected_physical_TX_read_copy=256,write_slice_function_proof_pending=True)
    # Check INIT on every surviving distributed memory/ROM too, not only BSRAM.
    distributed=[]
    for n,c in after.items():
        if not (c['kind'].startswith('RAM16') or c['kind']=='ROM16'):continue
        assert n in before and before[n]['kind']==c['kind'], ('new distributed memory identity',n)
        assert oldparams(before[n])==newparams(c), ('distributed memory INIT changed',n)
        distributed.append(n)
    assert not any(c['kind'] == 'GTR12_QUAD' for c in after.values())
    assert not any(n.startswith(('u_sfp/','u_neth/')) for n in after)
    txdis=[c for c in after.values() if c['kind']=='OBUF' and c['ports'].get('O')==['sfp1_tx_dis']]
    assert len(txdis)==1 and txdis[0]['ports']['I']==['VCC'], 'physical SFP TX_DISABLE must be actually high'
    r.update(source_BSRAM=313, wide_leaves=10, predicted_physical_BSRAM=323,
        physical_is_prediction=True, preserved_BSRAM_geometry_and_INIT=312,
        removed_physical_host_RX_RAM=removed, disconnected_physical_host_TX_RAM=dict(path=stub_tx,
            old_kind='SDPX9B',new_kind='SDPB',old_width=9,new_width=1,explicit_zero_INIT_parameters=len(init),vendor_default_zero_INIT_parameters=64-len(init)),
        BSRAM_by_top=inv['by_top_instance'], full_mailbox_memories=mailbox_counts,
        surviving_distributed_memory_ROM_INIT_checked=len(distributed),
        physical_SFP_MAC_PCS_SerDes_removed=True)
    checkpoint('actual_RAM_capacity_and_initialization_passed')
    directions = primitive_directions(LIBRARY)
    drivers = defaultdict(list)
    for n,c in after.items():
        assert c['kind'] in directions, ('unknown primitive direction',c['kind'])
        for pin,nets in c['ports'].items():
            if directions[c['kind']][pin] == 'output':
                for net in nets:
                    if net not in ('GND','VCC'): drivers[net].append((n,pin))
    assert not {n:d for n,d in drivers.items() if len(d)>1}, 'multiple actual primitive output drivers'
    clocks = Counter(tuple(c['ports']['CLK']) for c in after.values() if c['kind'].startswith('DFF'))
    derived={'u_lmac/n13_6':('u_lmac/n13_s2','rgmii_rxc_d',5),
             'u_lmac/n630_6':('u_lmac/n630_s2','clk50_d',1)}
    assert set(clocks) == {('clk50_d',), ('rgmii_rxc_d',)} | {(n,) for n in derived}, ('unexpected actual DFF clock domains',dict(clocks))
    for net,(name,parent,expected_loads) in derived.items():
        assert before[name]==after[name] and after[name]['kind']=='INV'
        assert after[name]['ports']=={'O':[net],'I':[parent]}
        assert clocks[(net,)]==expected_loads
        rows={n:c for n,c in after.items() if c['kind'].startswith('DFF') and c['ports']['CLK']==[net]}
        assert all(before[n]==c and oldparams(before[n])==newparams(c) for n,c in rows.items())
    for net,port in (('clk50_d','clk50'),('rgmii_rxc_d','rgmii_rxc')):
        assert len(drivers[net]) == 1
        cell = after[drivers[net][0][0]]
        assert cell['kind'] == 'IBUF' and cell['ports']['I'] == [port]
    from prepare_no_sfp_clock_plan import plan, construct_shared_logic
    from prove_soc_console_remap import z3
    la,lb,logic_boundaries=construct_shared_logic(before,after,oldparams,newparams,directions)
    logic_checks=[]
    def same_logic(label,old_net,new_net):
        delta=z3.simplify(z3.Xor(la.expr(old_net),lb.expr(new_net)))
        if not z3.is_false(delta):
            solver=z3.Solver();solver.set(timeout=10000);solver.add(delta)
            status=solver.check()
            assert status==z3.unsat, ('actual boundary changed or unproved',label,str(status),str(solver.model())[:1800] if status==z3.sat else solver.reason_unknown())
        logic_checks.append(dict(endpoint=label,old_net=old_net,new_net=new_net,Boolean_equivalent=True))
    reference = load(baseline/'cdc_reference_audit.json')
    assert reference['passed'] and len(reference['paths'])==26
    firsts={after[p['first_register']]['ports']['Q'][0] for p in reference['paths']}
    first_loads=defaultdict(list)
    for n,c in after.items():
        for pin,nets in c['ports'].items():
            if directions[c['kind']][pin] in ('input','inout'):
                for net in nets:
                    if net in firsts:first_loads[net].append((n,pin))
    for row in reference['paths']:
        for key in ('source_register','first_register','second_register'):
            n=row[key]; x,y=before[n],after[n]
            assert x['kind']==y['kind'] and oldparams(x) == newparams(y), ('known CDC kind/INIT changed',n)
            assert x['ports']['CLK']==y['ports']['CLK']
            assert set(x['ports'])==set(y['ports'])
            for pin in x['ports']:
                if directions[x['kind']][pin]!='input' or pin=='CLK':continue
                assert len(x['ports'][pin])==len(y['ports'][pin])
                for i,(a,b) in enumerate(zip(x['ports'][pin],y['ports'][pin])):
                    if a!=b:same_logic(n+'/'+pin+'['+str(i)+']',a,b)
        source,first,second=[after[row[k]] for k in ('source_register','first_register','second_register')]
        assert source['ports']['Q']==first['ports']['D'] and len(source['ports']['Q'])==1
        assert drivers[first['ports']['D'][0]]==[(source['name'],'Q')]
        assert first['ports']['Q']==second['ports']['D'] and len(first['ports']['Q'])==1
        assert first_loads[first['ports']['Q'][0]]==[(second['name'],'D')]
        assert source['ports']['CLK']==[row['source_clock']]
        assert all(c['ports']['CLK']==[row['target_clock']] and c['ports'].get('CE',['VCC'])==['VCC'] for c in (first,second))
    lidar=dict(passed=True,paths=26,source_first_second_direct_data_clocks_INIT_and_single_fanout_checked=True,
        all_changed_endpoint_inputs_Boolean_proved=logic_checks.copy(),
        no_unproved_aliases=True,physical_CDC_delay_reset_release_MTBF_signoff=False)
    edges = direct_edges(after)
    known = {(p['source_register'],p['first_register']) for p in reference['paths']}
    extra = [e for e in edges if (e['source_register'],e['target_register']) not in known]
    sdc = (build/'g2.sdc').read_text(encoding='utf-8')
    assert len(re.findall(r'^set_false_path\b.*-hold',sdc,re.M)) == 26
    assert len(re.findall(r'^set_max_delay\b',sdc,re.M)) == 26
    assert sdc_fragment(reference) in sdc
    assert sorted(float(p) for p in re.findall(r'^create_clock\b.*?-period\s+(\S+)',sdc,re.M)) == [10,20]
    active_sdc='\n'.join(line for line in sdc.splitlines() if not line.lstrip().startswith('#'))
    assert not any(x in active_sdc for x in ('u_sfp/', 'u_neth/', 'h_rx_clk', 'h_tx_clk', 'set_clock_groups'))
    assert not re.search(r'^set_false_path(?![^\n]*-hold)',sdc,re.M)
    r.update(true_clocks=2, register_clock_domains={k[0]:v for k,v in clocks.items()},
        actual_true_clock_input_drivers_checked=True, multiple_primitive_output_drivers=0,
        original_inverted_clock_branches={n:dict(driver=d,parent_clock=p,unchanged_register_loads=c) for n,(d,p,c) in derived.items()},
        lidar26=lidar, hold_only=26, max_delay=26, known_text_constraints=52,
        explicit_direct_CDC=len(edges), additional_direct_CDC=extra,
        remaining_CDC_signed_off=False, physical_CDC_timing_reset_PCS_IO_signed_off=False,
        full_UART_business_and_disconnect_stop_verified=False,
        prior_SFP_host51_required=False, removed_SFP_host51_constraint_pairs=51)
    checkpoint('two_clocks_known_lidar26_and_remaining_CDC_inventory_passed')
    # Match the remaining shadow storage by exact data/read/write coordinate,
    # then prove its bank write-enable functions at actual stable Q boundaries.
    shadow_pairs=[]; unmatched=list(old_shadow_rows)
    for y in new_shadow_rows:
        candidates=[x for x in unmatched if all(x['ports'][p]==y['ports'][p] for p in ('CLK','DI','WAD','RAD'))
                    and oldparams(x)==newparams(y)]
        matches=[]
        for x in candidates:
            delta=z3.simplify(z3.Xor(la.expr(x['ports']['WRE'][0]),lb.expr(y['ports']['WRE'][0])))
            if z3.is_false(delta):matches.append(x);continue
            solver=z3.Solver();solver.set(timeout=10000);solver.add(delta)
            if solver.check()==z3.unsat:matches.append(x)
        assert len(matches)==1, ('shadow write slice mapping nonunique/unproved',y['name'],[x['name'] for x in matches])
        x=matches[0];unmatched.remove(x)
        shadow_pairs.append(dict(old=x['name'],new=y['name']))
    assert not unmatched and len(shadow_pairs)==256
    mailbox_counts['u_nav/u_mbx/u_txs/'].update(write_slice_function_proof_pending=False,
        write_slice_function_proof_passed=True,memory_leaf_mapping=shadow_pairs)
    r['functional_clock_logic_boundaries']=logic_boundaries
    clock = plan(before,after,oldparams,newparams,directions,original_cst=(baseline/'g2.cst').read_text(encoding='utf-8'),logic_old=la,logic_new=lb,
                 removed_sfp_management_page_contract=True)
    cst = clock.get('new_cst',clock.get('cst'))
    clock.pop('new_cst',None); clock.pop('cst',None)
    r['clock_resource_review'] = clock
    assert clock['passed'], ('actual clock/control driver review failed',clock.get('errors'))
    assert isinstance(cst,str) and len(re.findall(r'^CLOCK_LOC ',cst,re.M)) == 14
    assert [l for l in cst.splitlines() if l.startswith(('IO_LOC ','IO_PORT '))] == io_lines(baseline/'g2.cst')
    r.update(passed=True, implementation_preflight_passed=True, phase='implementation_preflight_passed',
        pnr_source=str(pnr_inputs), actual_global_allocation_verified=False,
        exploratory_PNR_only=True, full_UART_behaviour_and_disconnect_stop_are_deployment_gates=True)
    # Avoid duplicating the full CST in the summary; the complete text is the
    # separately published input, and all functional mappings stay in review.
    r['pnr_input_files'] = physical_inputs(build,pnr_inputs,r,cst,reference)
    del before,after,old_defs,new_defs,oldparams,newparams
    gc.collect()
    return r


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('build','baseline','prepared','out','pnr-inputs'):
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    try:
        import psutil
        me=psutil.Process(); me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
        me.cpu_affinity(list(range(min(16,len(me.cpu_affinity())))))
    except ImportError:
        pass
    try:
        r=review(a.build,a.baseline,a.prepared,a.pnr_inputs,progress=a.out)
    except Exception as e:
        r=load(a.out) if a.out.exists() else dict(source=str(a.build))
        r.update(passed=False,implementation_preflight_passed=False,error_type=type(e).__name__,error=str(e),traceback=traceback.format_exc(),
                 diagnostic_only=True,deployment_accepted=False)
        save(a.out,r)
        print(json.dumps(dict(passed=False,phase=r.get('phase'),error=str(e)),ensure_ascii=False),flush=True)
        return 1
    save(a.out,r)
    print(json.dumps({k:r[k] for k in ('passed','implementation_preflight_passed','source_BSRAM','wide_leaves','predicted_physical_BSRAM','true_clocks','explicit_direct_CDC','pnr_source')},ensure_ascii=False),flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
