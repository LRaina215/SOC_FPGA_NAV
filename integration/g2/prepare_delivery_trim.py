"""Freeze explicitly traded peripheral features as independently runnable builds.

The first profile has a strictly scoped frozen netlist for immediate PNR, plus
matching complete RTL inputs. The second is RTL-only because synthesis exported
shared implementation signals across the TX-shadow module boundary. No tools,
synthesis, routing, programming or queue mutation are performed by this file.
"""
import argparse
import copy
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import re
import shutil

from audit_netlist_memories import MODULE, INST, BSRAM
from prepare_routing_ablation_ladder import release_oled, hierarchy
from review_small_block_stores import parameter_reader
from synthesis_connectivity import read
from trim_peripheral_rtl import trim_oled, trim_tx_shadow, trim_console_storage


ASSETS = {'.hex', '.mem', '.bin', '.mif'}
PROFILES = {
    'oled': dict(profile_id='light_oled_off', kind='frozen_netlist',
        cuts=['OLED display and its independent open-drain I2C traffic'],
        changes={'control4500_core.v': trim_oled},
        prefixes=['u_control/display/']),
    'offline': dict(profile_id='light_oled_and_tx_readback_off', kind='rtl_source',
        cuts=['OLED display and its independent open-drain I2C traffic',
              'Offline host mailbox TX mirror readback; reads return zero, live TX and all RX retained'],
        changes={'control4500_core.v': trim_oled, 'nav/rtl/net/host_mbx.v': trim_tx_shadow},
        prefixes=['u_control/display/', 'u_nav/u_mbx/u_txs/']),
    'console': dict(profile_id='light_oled_tx_readback_and_log_storage_off', kind='rtl_source',
        cuts=['OLED display and its independent open-drain I2C traffic',
              'Offline host mailbox TX mirror readback; reads return zero, live TX and all RX retained',
              'Diagnostic console character storage/readback; CPU writes ACK and counter retained, debug download/HOLD retained'],
        changes={'control4500_core.v': trim_oled, 'nav/rtl/net/host_mbx.v': trim_tx_shadow,
                 'nav/rtl/soc/nav_soc.v': trim_console_storage},
        prefixes=['u_control/display/', 'u_nav/u_mbx/u_txs/',
                  *[f'u_soc/g_con[{i}].u_con/' for i in range(4)]])}


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')


def relocate(value, origin, target):
    if isinstance(value, str):return value.replace(origin.as_posix(), target.as_posix()).replace(str(origin), str(target))
    if isinstance(value, list):return [relocate(x, origin, target) for x in value]
    if isinstance(value, dict):return {k:relocate(v, origin, target) for k,v in value.items()}
    return value


def prepare(source, reference, out, selection, interface_proof=None, console_proof=None):
    source, reference, out = (Path(x).resolve() for x in (source, reference, out))
    assert not out.exists() and source not in out.parents and out not in source.parents
    config = PROFILES[selection]
    manifest = json.loads((source/'manifest.json').read_text(encoding='utf-8'))
    old_job = json.loads((source/'job.json').read_text(encoding='utf-8'))
    assert old_job['phase'] == 'synthesis_complete' and old_job['tool_exit'] == 0
    assert manifest['real_sfp'] and manifest['full_control_telemetry'] and not manifest['control_weights_changed']
    assert manifest['clock_core_mhz'] == 50 and manifest['sfp_lane2_input_checks']['lane'] == 'Q0 Lane2'
    source_vg = source/'impl/gwsynthesis/g2_top.vg'
    assert source_vg.read_bytes() == (reference/'g2_top.vg').read_bytes(), 'constraint reference uses another source'
    hardware = [Path(p).relative_to(source) for p in manifest['source_files']]
    assets = {p.relative_to(source) for p in source.rglob('*') if p.is_file()
              and p.suffix.lower() in ASSETS and 'impl' not in p.relative_to(source).parts}
    headers = {p.relative_to(source) for p in source.rglob('*') if p.is_file()
               and p.suffix.lower() in {'.vh','.h'} and 'impl' not in p.relative_to(source).parts}
    assert len(hardware) == len(set(hardware)) == 68 and len(assets) == 104
    extras = {Path('g2.cst'), Path('g2.sdc'), Path('ip/serdes.csr')}
    extras.update(p.relative_to(source) for p in (source/'ip').iterdir()
                  if p.is_file() and p.suffix.lower() in {'.toml','.ipc','.mod','.json'})
    out.mkdir(parents=True)
    for relative in set(hardware)|assets|headers|extras:
        target = out/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source/relative, target)
    for name, transform in config['changes'].items():
        (out/name).write_text(transform((source/name).read_text(encoding='utf-8')), encoding='utf-8', newline='\n')
    for name in ('g2.cst', 'g2.sdc', 'ip/serdes.csr', 'cdc_reference_audit.json', 'sfp_cdc_reference.json'):
        (out/name).parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(reference/name, out/name)
    changed = sorted(p.as_posix() for p in hardware if (source/p).read_bytes() != (out/p).read_bytes())
    assert changed == sorted(config['changes'])
    assert all((source/p).read_bytes() == (out/p).read_bytes() for p in assets|headers)
    for name in changed:
        assert (out/name).read_text(encoding='utf-8') == config['changes'][name]((source/name).read_text(encoding='utf-8'))
    new_manifest = relocate(copy.deepcopy(manifest), source, out)
    new_manifest.pop('effective_options', None)
    new_manifest.update(frozen_source_build=str(source), programmed=False, diagnostic_only=False,
        deployment_accepted=False, complete_sfp_cdc_acceptance=False,
        max_runtime_minutes=0, max_no_progress_minutes=0,
        delivery_trim_profile=config['profile_id'], frozen_netlist=config['kind']=='frozen_netlist')
    original_tcl = (source/'run.tcl').read_text(encoding='utf-8')
    assert [x.strip() for x in original_tcl.splitlines() if x.strip().startswith('run ')] == ['run syn']
    (out/'run.tcl').write_text(relocate(original_tcl, source, out), encoding='utf-8', newline='\n')
    proofs = []
    state = dict(passed=True, source=str(source), source_base=str(source), prepared=str(out),
        profile_id=config['profile_id'], kind=config['kind'], created_at=datetime.now().astimezone().isoformat(),
        modified_rtl_inputs=changed, unchanged_hardware_inputs=len(hardware)-len(changed),
        unchanged_assets=104, copied_headers=len(headers), exact_source_transform_rechecked=True,
        source_file_set_unchanged=True, original_4500_weights_125Hz_CPU128KiB_and_full_navigation_retained=True,
        control_telemetry_and_SFP_transports_retained=True, original_stop_guard_sources_unchanged=True,
        all_original_clock_constraints_retained=True, constraint_reference=str(reference),
        cuts_stop_protection=False, removed_functions=config['cuts'], synthesis_started=False,
        PNR_started=False, programmed=False, deployment_accepted=False,
        permanent_diagnostic_lock=False, requires_complete_deployment_signoff=True)
    if config['kind'] == 'frozen_netlist':
        original = source_vg.read_text(encoding='utf-8')
        matches = list(MODULE.finditer(original)); modules={m[1]:m[2] for m in matches}
        paths, mult = hierarchy(modules)
        kind = paths['u_control/display']; assert kind == 'oled' and mult[kind] == 1
        target, = [m for m in matches if m[1] == kind]
        changed_vg = original[:target.start()]+release_oled(kind,target[2])+original[target.end():]
        (out/'impl/gwsynthesis').mkdir(parents=True)
        (out/'g2_top.vg').write_text(changed_vg, encoding='utf-8', newline='\n')
        shutil.copyfile(out/'g2_top.vg', out/'impl/gwsynthesis/g2_top.vg')
        shutil.copyfile(source/'impl/gwsynthesis/g2_top.prj', out/'impl/gwsynthesis/g2_top.prj')
        shutil.copyfile(source/'effective_options.tcl', out/'effective_options.tcl')
        before, after = read(source_vg), read(out/'g2_top.vg')
        prefix = 'u_control/display/'
        outside = lambda cells:{n:c for n,c in cells.items() if not n.startswith(prefix)}
        assert outside(before) == outside(after)
        removed={n:c for n,c in before.items() if n.startswith(prefix)}
        inserted={n:c for n,c in after.items() if n.startswith(prefix)}
        assert len(inserted)==2 and all(c['kind']=='VCC' for c in inserted.values())
        for name, pin, net in (('sck_iobuf','sck','IIC_SCL_6'), ('sda_iobuf','sda','IIC_SDA_6')):
            assert after[name]['kind']=='IOBUF' and after[name]['ports']['I']==['GND']
            assert after[name]['ports']['IO']==[pin] and after[name]['ports']['OEN']==[net]
            assert any(c['ports']=={'V':[net]} for c in inserted.values())
        params=parameter_reader(modules)
        oldram=[c for c in before.values()if c['kind']in BSRAM]
        removedram=[c for c in removed.values()if c['kind']in BSRAM]
        iswide=lambda c:any(params(c).get(k)in('32','36')for k in('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1'))
        source_bsram=len(oldram)-len(removedram);wide=sum(map(iswide,oldram))-sum(map(iswide,removedram))
        kinds=Counter(c['kind']for c in removed.values())
        state.update(outside_explicit_cells=len(outside(before)), outside_cell_pins_identical=True,
            OLED_open_drain_release_proved=True, removed_primitive_counts=dict(kinds),
            removed_explicit_primitives=len(removed), expected_source_BSRAM=source_bsram,
            expected_wide_leaves=wide, expected_predicted_physical_BSRAM=source_bsram+wide,
            modified_netlist_modules=['oled'], implementation_netlist_derived_without_resynthesis=True)
    else:
        assert interface_proof, 'offline cuts require a completed source-bound main-interface proof'
        proof_path = Path(interface_proof).resolve()
        proof = json.loads(proof_path.read_text(encoding='utf-8'))
        assert proof['passed'] and proof['live_mailbox_interface_passed']
        assert Path(proof['source']).resolve() == source
        assert (out/'nav/rtl/net/host_mbx.v').read_bytes() == (proof_path.parent/'host_mbx_trim.v').read_bytes()
        state['interface_proof']=str(proof_path)
        proofs.append(dict(path=str(proof_path), required_true_fields=['passed','live_mailbox_interface_passed'],
            source_bindings=[dict(candidate='nav/rtl/net/host_mbx.v', proof_source=str(proof_path.parent/'host_mbx_trim.v'))]))
        if selection == 'console':
            assert console_proof, 'console trade requires the completed bus/HOLD/download proof'
            console_path = Path(console_proof).resolve()
            console = json.loads(console_path.read_text(encoding='utf-8'))
            assert console['passed'] and console['console_storage_contract_passed']
            assert console['CPU128KiB_hold_download_and_nav_protocol_passed']
            assert Path(console['source']).resolve() == source
            assert (out/'nav/rtl/soc/nav_soc.v').read_bytes() == (console_path.parent/'nav_soc_trim.v').read_bytes()
            state['console_interface_proof'] = str(console_path)
            state['complete_CPU_program_not_executed_in_console_proof'] = True
            proofs.append(dict(path=str(console_path), required_true_fields=['passed',
                'console_storage_contract_passed','CPU128KiB_hold_download_and_nav_protocol_passed'],
                source_bindings=[dict(candidate='nav/rtl/soc/nav_soc.v',proof_source=str(console_path.parent/'nav_soc_trim.v'))]))
    state['original_IO_SDC_CSR_unchanged']=all((out/p).read_bytes()==(reference/p).read_bytes()
        for p in ('g2.cst','g2.sdc','ip/serdes.csr'))
    profile=dict(schema_version=1, profile_id=config['profile_id'], profile_name=config['profile_id'],
        kind=config['kind'], origin=str(source), source_base=str(source), constraint_reference=str(reference),
        allowed_changed_prefixes=config['prefixes'], allowed_memory_change_prefixes=config['prefixes'],
        modified_rtl_inputs=changed, added_rtl_inputs=[], removed_rtl_inputs=[],
        allowed_changed_assets=[], tradeoffs=config['cuts'], cuts_stop_protection=False,
        full_navigation_control_SFP_retained=True, deployment_accepted=False, diagnostic_only=False,
        permanent_diagnostic_lock=False, max_runtime_minutes=0, max_no_progress_minutes=0,
        requires_complete_deployment_signoff=True,
        source_difference_proof='prepared_source_review.json',
        proofs=[dict(path='prepared_source_review.json', required_true_fields=['passed','exact_source_transform_rechecked'],
            source_bindings=[dict(candidate=name, proof_source=str(out/name)) for name in changed])]+proofs)
    for key in ('expected_source_BSRAM','expected_wide_leaves','expected_predicted_physical_BSRAM'):
        if key in state:profile[key]=state[key]
    save(out/'manifest.json',new_manifest);save(out/'prepared_source_review.json',state)
    save(out/'trim_profile.json',profile);save(out/'job.json',dict(state,phase='prepared_delivery_trim_not_built'))
    return state


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--reference-pnr',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--profile',choices=PROFILES,required=True)
    ap.add_argument('--interface-proof',type=Path)
    ap.add_argument('--console-proof',type=Path)
    a=ap.parse_args();result=prepare(a.source,a.reference_pnr,a.out,a.profile,a.interface_proof,a.console_proof)
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
