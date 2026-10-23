"""Continue one already-frozen candidate through checked SYN -> one PNR.

No periodic automation, retries, budget extension or hardware action. Waits for
the existing SYN owner (which itself waits for the diagnostic collector). A
failed prerequisite stops with evidence. PNR success never grants deployment.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

import psutil

from audit_routing_topology import primitive_directions
from audit_sfp_cdc import audit as audit_sfp
from collect_routing_diagnostic import collect as routing_diagnostic
from retry_frozen_synthesis import memory_status
from synthesis_connectivity import read

ROOT = Path(__file__).resolve().parents[2]
LIBRARY = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
PREFIX = 'u_nav/u_match/u_sm/'


def bank_load(cells):
    directions = primitive_directions(LIBRARY)
    loads = Counter()
    for c in cells.values():
        for pin, nets in c['ports'].items():
            if directions[c['kind']][pin] in ('input','inout'):
                loads.update(nets)
    regs = [dict(name=n, loads=loads[c['ports']['Q'][0]])
            for n,c in cells.items() if n.startswith(PREFIX) and c['kind'].startswith('DFF') and 'selected' in n]
    assert regs, 'full matcher bank selector absent'
    return dict(registers=regs, maximum=max(r['loads'] for r in regs))


def memories_match(before, after):
    key = lambda r:(r['path'],r['primitive'],tuple(sorted(r['parameters'].items())))
    return {key(r) for r in before['leaves']} == {key(r) for r in after['leaves']}


def validate_candidate(source, baseline, plan_file):
    source, baseline = Path(source), Path(baseline)
    assert re.search(r'PR0004\).*total 273 unrouted nets',
                     (baseline/'impl/pnr/g2_top.rpt.txt').read_text()), 'wrong comparison baseline'
    job = json.loads((source/'job.json').read_text())
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0, ('SYN not complete',job)
    manifest = json.loads((source/'manifest.json').read_text())
    assert manifest['bank_select_data_fanout'] and manifest['real_sfp'] and manifest['full_control_telemetry']
    assert not manifest['control_weights_changed'] and manifest['clock_core_mhz'] == 50
    lane2_proof = manifest.get('sfp_lane2_from_build')
    row_banks = manifest.get('slf_row_banks',False)
    bias_block = manifest.get('bias_block_rom',False)
    expected_changes = ['fc_v4_top.v'] if bias_block else ['nav/rtl/common/nav_ram_sdp.v']
    if lane2_proof and not row_banks and not bias_block:expected_changes += ['sfp_host_mac.v','ip/serdes.v']
    assert sorted(job['modified_rtl_inputs']) == sorted(expected_changes)
    original = Path(manifest['frozen_source_build'])
    ram = source/'nav/rtl/common/nav_ram_sdp.v'
    attribute = ' /* synthesis syn_maxfan=128 */'
    if bias_block:
        from prepare_bias_block_rom import transform
        assert (source/'fc_v4_top.v').read_text() == transform((original/'fc_v4_top.v').read_text()), 'bias delta exceeds two attributes'
        assert ram.read_bytes() == (original/'nav/rtl/common/nav_ram_sdp.v').read_bytes(), 'inherited RAM implementation changed'
        bias_probe = json.loads((source.parent/'bias_block_probe1/result.json').read_text())
        assert Path(bias_probe['source']).resolve() == original.resolve()
        assert bias_probe['all_original32bit_biases_identical'] and bias_probe['connection_pressure_reduced']
        assert bias_probe['rom16_removed'] == 702 and [x['variant'] for x in bias_probe['checks']] == ['rtl','vendor']
        probe = json.loads((source.parent/'slf_row_banks_probe1/result.json').read_text())
        assert row_banks and json.loads((original/'manifest.json').read_text())['slf_row_banks']
    elif row_banks:
        from prepare_slf_row_banks import transform
        assert ram.read_text() == transform((original/'nav/rtl/common/nav_ram_sdp.v').read_text()), 'RAM delta exceeds reviewed row banking'
        probe = json.loads((source.parent/'slf_row_banks_probe1/result.json').read_text())
        assert Path(probe['source']).resolve() == original.resolve()
        assert probe['bsram_unchanged'] and probe['connection_pressure_reduced']
        assert [x['variant'] for x in probe['checks']] == ['rtl','vendor']
    else:
        assert ram.read_text().count(attribute) == 1
        assert ram.read_text().replace(attribute,'') == (original/'nav/rtl/common/nav_ram_sdp.v').read_text(), 'RAM delta exceeds one attribute'
    changed = [str(Path(n).relative_to(source)).replace('\\','/') for n in manifest['source_files']
               if Path(n).read_bytes() != (original/Path(n).relative_to(source)).read_bytes()]
    assert changed == job['modified_rtl_inputs'], changed
    def assets(root):
        return {p.relative_to(root):p for p in root.rglob('*') if p.is_file()
                and p.suffix.lower() in ('.hex','.mem','.bin','.mif') and 'impl' not in p.relative_to(root).parts}
    old_assets,new_assets = assets(original),assets(source)
    assert old_assets.keys() == new_assets.keys() and len(old_assets) == 104, 'firmware/storage asset set changed'
    assert all(p.read_bytes() == new_assets[n].read_bytes() for n,p in old_assets.items()), 'firmware/storage content changed'
    lane2_checks = None
    if lane2_proof:
        from prepare_sfp_lane2 import verify_frozen
        lane2_checks = verify_frozen(source,original,lane2_proof)
    else:
        assert (source/'ip/serdes.csr').read_bytes() == (baseline/'ip/serdes.csr').read_bytes()
        assert (source/'g2.sdc').read_bytes() == (original/'g2.sdc').read_bytes()
    summary = json.loads((source/'synthesis_summary.json').read_text())
    assert not summary['errors'] and summary['actual_synthesis_global_freq_mhz'] == 50
    core, = [c for c in summary['synthesis_timing']['clocks'] if c['name'] == 'clk50']
    assert core['constraint_mhz'] == 50 and core['estimated_fmax_mhz'] >= 50, core
    old_mem = json.loads((original/'memory_inventory.json').read_text())
    new_mem = json.loads((source/'memory_inventory.json').read_text())
    bias_memory_additions = []
    if bias_block:
        assert old_mem['expanded_bsram_count'] == 316 and old_mem['wide32_36_count'] == 18
        old_keys = {r['path']:(r['primitive'],r['parameters']) for r in old_mem['leaves']}
        new_keys = {r['path']:(r['primitive'],r['parameters']) for r in new_mem['leaves']}
        assert all(new_keys.get(k) == v for k,v in old_keys.items()), 'an existing BSRAM changed'
        bias_memory_additions = [r for r in new_mem['leaves'] if r['path'] not in old_keys]
        assert 1 <= len(bias_memory_additions) <= 2 and all('u_control/model_thread/policy/core/' in r['path'] and 'bias_mem' in r['path'] for r in bias_memory_additions), 'unexpected new memory'
        assert new_mem['predicted_split_bsram'] - old_mem['predicted_split_bsram'] == 2, 'bias physical budget must remain two blocks'
    else:
        assert new_mem['expanded_bsram_count'] == old_mem['expanded_bsram_count'] == 316
        assert new_mem['wide32_36_count'] == old_mem['wide32_36_count'] == 18
        assert memories_match(old_mem,new_mem), 'RAM primitive capacity/initialization changed'
    assert new_mem['predicted_split_bsram'] <= 336
    assert new_mem['primitive_types']['GTR12_QUAD'] == 1
    cells = read(source/'impl/gwsynthesis/g2_top.vg')
    bias_rom_checks = None
    if bias_block:
        core_prefix = 'u_control/model_thread/policy/core/'
        before_bias = read(original/'impl/gwsynthesis/g2_top.vg',lambda n:n in ('u_control','u_control/model_thread','u_control/model_thread/policy',core_prefix.rstrip('/')) or n.startswith(core_prefix))
        def rom16(data):return sum(c['kind']=='ROM16' for n,c in data.items() if n.startswith(core_prefix))
        assert rom16(before_bias) == 702 and rom16(cells) == 0, 'full SYN did not remove distributed bias ROM'
        pin_directions = primitive_directions(LIBRARY)
        def bias_address_loads(data):
            counts = Counter(net for n,c in data.items() if n.startswith(core_prefix)
                             for pin,nets in c['ports'].items() if pin_directions[c['kind']][pin] in ('input','inout')
                             for net in nets)
            return {core_prefix+f'bias_address[{i}]':counts[core_prefix+f'bias_address[{i}]'] for i in range(4)}
        address_before,address_after = bias_address_loads(before_bias),bias_address_loads(cells)
        assert set(address_before.values()) == {702}, address_before
        assert all(0 < v <= 16 for v in address_after.values()), address_after
        bias_rom_checks = dict(before_rom16=702,after_rom16=0,all_original425_signed_values_preserved=True,
                               new_bsram_leaves=bias_memory_additions,physical_bsram_addition_prediction=2,
                               low_address_loads_before=address_before,low_address_loads_after=address_after,
                               probe_checks=bias_probe['checks'],read_cycles_added=0)
    row_store_checks = None
    if row_banks:
        row_prefix = 'u_nav/u_match/u_ex/u_rb/'
        # Reuse the same directional inventory on just the complete row store.
        def row_pressure(data):
            directions = primitive_directions(LIBRARY)
            local = {n:c for n,c in data.items() if n.startswith(row_prefix)}
            assert local, 'full SLF row store absent'
            loads = Counter(net for c in local.values() for pin,nets in c['ports'].items()
                            if directions[c['kind']][pin] in ('input','inout') and pin not in ('CLK','CLKA','CLKB')
                            for net in nets if net not in ('VCC','GND'))
            return dict(cells=len(local),maximum_nonclock_load=max(loads.values()),
                        distributed_words=sum(c['kind']=='RAM16SDP4' for c in local.values()),
                        bank_select_registers=sum(c['kind'].startswith('DFF') and 'row_selected' in n for n,c in local.items()))
        old_row = read(original/'impl/gwsynthesis/g2_top.vg',lambda n:n in ('u_nav','u_nav/u_match','u_nav/u_match/u_ex',row_prefix.rstrip('/')) or n.startswith(row_prefix))
        a,b = row_pressure(old_row),row_pressure(cells)
        assert b['maximum_nonclock_load'] <= 275,(a,b)
        assert (a == b if bias_block else b['maximum_nonclock_load'] < a['maximum_nonclock_load']), (a,b)
        assert b['distributed_words'] == 250 and b['bank_select_registers'] >= 4,b
        row_store_checks = dict(before=a,after=b,logical_bits_unchanged=15680,latency_cycles_unchanged=1,
                                rtl_and_vendor_probe_checks=probe['checks'])
    if lane2_checks:
        from prepare_sfp_lane2 import verify_synthesized_clock_pins
        lane2_checks['actual_synthesized_clock_pins'] = verify_synthesized_clock_pins(cells)
    old_cells = read(baseline/'g2_top.vg')
    old, new = bank_load(old_cells), bank_load(cells)
    assert new['maximum'] <= 128 and new['maximum'] < old['maximum'], (old,new)
    # Revalidate the actual drivers before reusing stable-looking net names.
    directions = primitive_directions(LIBRARY)
    def driver_index(data):
        result = defaultdict(list)
        for name,c in data.items():
            for pin,nets in c['ports'].items():
                if directions[c['kind']][pin] == 'output':
                    for net in nets:result[net].append((name,c))
        return result
    old_drivers,new_drivers = driver_index(old_cells),driver_index(cells)
    plan = json.loads(Path(plan_file).read_text())
    names = [r['net'] for r in plan['promote']]+plan['local']
    from revalidate_lidar_cdc import revalidate, definitions
    old_definitions = definitions(baseline/'g2_top.vg')
    new_definitions = definitions(source/'impl/gwsynthesis/g2_top.vg')
    address_aliases = []
    for net in names:
        assert len(old_drivers[net]) == len(new_drivers[net]) == 1, ('routing-plan source missing',net)
        if old_drivers[net] != new_drivers[net] and row_banks:
            from prove_row_address_alias import prove
            address_aliases.append(prove(net,[c for _,c in old_drivers[net]],[c for _,c in new_drivers[net]],
                                         old_cells,cells,old_definitions,new_definitions))
        else:
            assert old_drivers[net] == new_drivers[net], ('routing-plan source changed',net)
    reference = json.loads((baseline/'cdc_reference_audit.json').read_text())
    lidar_revalidation = revalidate(old_cells,cells,reference,directions,
        old_definitions,new_definitions)
    sfp = audit_sfp(source/'impl/gwsynthesis/g2_top.vg',True)
    old_sfp = json.loads((baseline/'sfp_cdc_reference.json').read_text())
    assert sfp['passed'] and json.loads(json.dumps(sfp['paths'])) == old_sfp['paths']
    return dict(passed=True, source_bsram=new_mem['expanded_bsram_count'], wide_leaves=new_mem['wide32_36_count'],
                predicted_physical_bsram=new_mem['predicted_split_bsram'], physical_count_is_prediction=True,
                all_ram_parameters_match=not(row_banks or bias_block), all_bsram_parameters_match=not bias_block,
                all_existing_bsram_parameters_match=True,bias_rom_checks=bias_rom_checks,
                row_memory_storage_organization_changed=row_banks and not bias_block,row_banks_present=row_banks,
                bias_storage_organization_changed=bias_block,unchanged_firmware_storage_assets=len(old_assets),
                changed_rtl_inputs=changed, control_weights_and125hz_unchanged=True,
                source_csr_and_clock_sdc_unchanged=row_banks or not bool(lane2_checks), sfp_lane2_checks=lane2_checks, core_synthesis_estimate=core,
                selector_before=old, selector_after=new, clock_plan_sources_revalidated=names,
                row_store_checks=row_store_checks,
                proved_row_address_driver_aliases=address_aliases,
                lidar_revalidation=lidar_revalidation,
                lidar26_source_connections_match=True, host51_source_connections_match=True,
                deployment_accepted=False)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--synthesis',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--wait-pid',type=int,required=True)
    p.add_argument('--pnr-out',type=Path,required=True)
    p.add_argument('--state-dir',type=Path,required=True)
    p.add_argument('--check-only',action='store_true',help='internal isolated validation process; never starts a tool')
    p.add_argument('--max-runtime-minutes',type=float,default=120,help='explicit budget for the one future PNR, not an extension of an existing process')
    a = p.parse_args()
    assert 0 < a.max_runtime_minutes <= 150, 'future PNR must retain an explicit bounded budget'
    source,baseline,out,state_dir = map(lambda p:p.resolve(),(a.synthesis,a.baseline,a.pnr_out,a.state_dir))
    if a.check_only:
        shutil.copyfile(Path(__file__),state_dir/'validator_source.py')
        evidence = validate_candidate(source,baseline,state_dir/'clock_plan_frozen.json')
        (state_dir/'pre_pnr_checks.json').write_text(json.dumps(evidence,indent=2),encoding='utf-8',newline='\n')
        return 0
    state_dir.mkdir(parents=True,exist_ok=False)
    assert not out.exists(), 'PNR directory already exists; never overwrite/restart'
    state = dict(started_at=datetime.now().astimezone().isoformat(), synthesis=str(source), baseline=str(baseline),
                 pnr_out=str(out), wait_pid=a.wait_pid, maximum_wait_seconds=10800,
                 recurring=False, maximum_pnr_runs=1,
                 programmed=False, deployment_accepted=False)
    def save(phase,**extra):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**extra)
        (state_dir/'job.json').write_text(json.dumps(state,indent=2),encoding='utf-8',newline='\n')
        print(json.dumps(dict(phase=phase,**extra)),flush=True)
    try:
        shutil.copyfile(ROOT/'integration/g2/clock_resource_plan_control_priority.json',state_dir/'clock_plan_frozen.json')
        shutil.copyfile(Path(__file__),state_dir/'controller_source.py')
        syn_job = json.loads((source/'job.json').read_text())
        if syn_job['phase'] != 'synthesis_complete':
            prior = psutil.Process(a.wait_pid)
            command = prior.cmdline()
            assert any('retry_frozen_synthesis.py' in x for x in command) and str(source).replace('\\','/') in [x.replace('\\','/') for x in command], command
            save('waiting_for_existing_synthesis',wait_process_created_at=prior.create_time())
            prior.wait(timeout=10800)
        save('validating_full_candidate')
        plan = state_dir/'clock_plan_frozen.json'
        # Let the large parsed netlist dictionaries exit before starting PNR.
        check_command = [sys.executable,'-X','utf8',str(Path(__file__)),
                         '--synthesis',str(source),'--baseline',str(baseline),
                         '--wait-pid',str(a.wait_pid),'--pnr-out',str(out),
                         '--state-dir',str(state_dir),'--check-only']
        with (state_dir/'validation.log').open('w',encoding='utf-8') as stream:
            subprocess.run(check_command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,
                           creationflags=subprocess.CREATE_NO_WINDOW,timeout=300,check=True)
        evidence = json.loads((state_dir/'pre_pnr_checks.json').read_text())
        assert evidence['passed']
        save('waiting_for_pnr_memory',pre_pnr_checks_passed=True)
        for _ in range(24):
            memory = memory_status()
            if memory.avail_phys >= 2*1024**3 and memory.avail_page >= 4*1024**3:break
            time.sleep(5)
        else:
            save('not_started_host_memory',free_bytes=memory.avail_phys,available_commit_bytes=memory.avail_page)
            return 1
        # This creates exactly one new implementation from the complete SYN.
        command = [sys.executable,'-X','utf8',str(ROOT/'integration/g2/pnr_from_netlist.py'),str(source),
                   '--out',str(out),'--wide-split','1','--replicate-resources','1','--place-option','1',
                   '--route-option','2','--max-runtime-minutes',str(a.max_runtime_minutes),'--max-no-progress-minutes','0',
                   '--cdc-reference-audit',str(baseline/'cdc_reference_audit.json'),
                   '--sfp-cdc-reference',str(baseline/'sfp_cdc_reference.json'),
                   '--clock-resource-plan',str(plan),'--routing-evidence','--capture-system-telemetry']
        save('starting_one_full_pnr',command=command)
        with (state_dir/'pnr_runner.log').open('w',encoding='utf-8') as stream:
            child = subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,
                                     creationflags=subprocess.CREATE_NO_WINDOW)
            save('full_pnr_running',pnr_wrapper_pid=child.pid)
            child.wait()
        result = routing_diagnostic(out)
        (state_dir/'routing_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8',newline='\n')
        gate_file = out/'g2_gate.json'
        gate = json.loads(gate_file.read_text()) if gate_file.exists() else {}
        save('one_pnr_finished_review_required',wrapper_exit=child.returncode,
             reported_unrouted_nets=result['reported_unrouted_nets'],timing_report_available=result['timing_report_available'],
             vendor_runtime=result['runtime'],routing_passed=gate.get('routing_passed'),
             actual_resources=gate.get('resources'),setup_violations=gate.get('setup_violations'),
             hold_violations=gate.get('hold_violations'))
        return 0
    except Exception as error:
        save('stopped_prerequisite_or_execution_error',error_type=type(error).__name__,error=str(error))
        return 1


if __name__ == '__main__':raise SystemExit(main())
