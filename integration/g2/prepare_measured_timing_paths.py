"""Compose source-proved repairs on frozen SYN44; never start an EDA tool.

Default plans only. Materialization requires completed component proofs and
no active simulation or Gowin process owned by this integration workspace.
The optional action overlay is accepted only when its exact source transform
and original publication edges are independently bound by its own receipt.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil

import psutil

from prepare_baseline_fusion_retime import metadata, relocate, same_bytes, save
from prepare_nav_command_predecode import derive as nav_predecode
from plan_PNR53_PID_unity_bypass import ORIGINAL_TARGETS, UNITY_TARGETS
from prepare_ring_stride32_private_reader import kept_files, paths_in_manifest, natural_pair_complete

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE/'SLF128_ring_stride32_syn44'
NAV = 'nav/rtl/top/nav_subsystem.v'
CORE = 'control4500_core.v'
ACTION = ('control/policy_action_map.v', 'control/rl_policy_tick_v5.v', 'control/mem_policy.v')


def load(p):
    return json.loads(p.read_text(encoding='utf-8-sig'))


def no_owned_tools():
    names = {'gw_sh.exe', 'xvlog.exe', 'xelab.exe', 'xsim.exe', 'xsimk.exe',
             'vsim.exe', 'vsimk.exe', 'vlog.exe', 'vopt.exe'}
    found, unrelated = [], []
    for p in psutil.process_iter(['name']):
        try:
            if (p.info['name'] or '').lower() in names:
                cwd = Path(p.cwd()).resolve()
                if cwd.is_relative_to(BASE.resolve()):
                    row = dict(pid=p.pid, created_epoch=p.create_time(), cwd=str(cwd))
                    # This exact Oct23 preflight belongs to an older independent
                    # activity. It neither reads the new source nor grants an
                    # exception for any current build or reused process ID.
                    if (p.pid == 46512 and abs(p.create_time()-1792692510.6410265) < .001
                            and cwd == (BASE/'weight9_synthesis/packed9').resolve()
                            and len(p.cmdline()) == 2 and p.cmdline()[1] == 'preflight.tcl'):
                        unrelated.append(dict(row, command=p.cmdline(), untouched=True))
                    else:
                        found.append(row)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    assert not found, ('Own RTL tool active; do not materialize', found)
    return unrelated


def action_contract(overlay):
    """Bind the migrated preview to the original proven three-line interface."""
    ancestor = BASE/'baseline_fusion_only_syn23'
    proven = BASE/'action_preview_product_candidate1'
    assert same_bytes(SOURCE/ACTION[0], ancestor/ACTION[0])
    assert same_bytes(overlay/ACTION[0], proven/ACTION[0])
    old = (SOURCE/ACTION[1]).read_text(encoding='utf8')
    expected = old.replace('    output reg [143:0] actions,',
        '    output reg [143:0] actions,\n    output wire [143:0] preview_actions,', 1)
    anchor = '    wire signed [23:0] a0,a1,a2,a3,a4,a5;'
    assert old.count(anchor) == 1
    expected = expected.replace(anchor, anchor+'\n    assign preview_actions={a5,a4,a3,a2,a1,a0};', 1)
    actual = (overlay/ACTION[1]).read_text(encoding='utf8')
    assert actual == expected, 'Unexpected scheduler or ring change in action preview'
    seq = '    always @(posedge clk or negedge rst_n) begin'
    assert old[old.index(seq):] == actual[actual.index(seq):]
    old_owner = (SOURCE/ACTION[2]).read_text(encoding='utf8')
    expected_owner = old_owner.replace('    wire [143:0] actions;',
                                     '    wire [143:0] actions;\n    wire [143:0] action_preview;', 1)
    for old_ports, new_ports in (
        ('.capture(capture),.action_valid(action_valid),.actions(actions),',
         '.capture(capture),.action_valid(action_valid),.actions(actions),.preview_actions(action_preview),'),
        ('.run(run),.action_valid(action_valid),.actions(actions),',
         '.run(run),.action_valid(action_valid),.actions(actions),.preview_actions(action_preview),')):
        assert old_owner.count(old_ports) == 1
        expected_owner = expected_owner.replace(old_ports, new_ports, 1)
    assert (overlay/ACTION[2]).read_text(encoding='utf8') == expected_owner
    return dict(passed=True, original64bit_private_mapper_byte_identical_to_proved_source=True,
                exact_original_scheduler_sequential_tail=True,
                original_ring_reader_producer_and_LOAD_contract_unchanged=True,
                source_interface_only_connects_actual_six_core_actions=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--receipt', type=Path, required=True)
    ap.add_argument('--materialize', action='store_true')
    ap.add_argument('--action-overlay', type=Path)
    ap.add_argument('--action-review', type=Path)
    a = ap.parse_args()
    assert not a.receipt.exists()
    assert a.out.resolve().parent == BASE.resolve() and not a.out.exists()
    assert (a.action_overlay is None) == (a.action_review is None)
    manifest, inputs = paths_in_manifest(SOURCE)
    job = load(SOURCE/'job.json')
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0
    nav_review = BASE/'NAV_predecode_source_relation_review2/result.json'
    pid_review = BASE/'PID_unity_actual_ModelSim_review1/result.json'
    assert load(nav_review)['passed'] and load(pid_review)['passed']
    assert load(BASE/'NAV_command_predecode_bus_probe2/result.json')['passed']
    assert same_bytes(inputs[NAV], BASE/'SLF128_storage_safeFW_syn43'/NAV)
    assert same_bytes(inputs[CORE], BASE/'SLF128_storage_safeFW_SYN43_prepared1'/CORE)
    nav, _ = nav_predecode(inputs[NAV].read_text(encoding='utf8'))
    assert nav == (BASE/'SLF128_NAV_predecode_prepared1'/NAV).read_text(encoding='utf8')
    core = inputs[CORE].read_text(encoding='utf8')
    assert core.count(ORIGINAL_TARGETS) == 1
    patched = core.replace(ORIGINAL_TARGETS, UNITY_TARGETS, 1)
    stop = '    motor520_closed_loop #('
    assert core[core.index(stop):] == patched[patched.index(stop):]
    rendered = {NAV:nav, CORE:patched}
    proofs = [nav_review, pid_review]
    if a.action_overlay:
        proof = load(a.action_review)
        assert proof['passed'], 'Action source publication proof did not pass'
        action = action_contract(a.action_overlay)
        rendered.update({r:(a.action_overlay/r).read_text(encoding='utf8') for r in ACTION})
        proofs.append(a.action_review)
    else:
        action = dict(selected=False, current_SYN44_worst_action_path_still_unrepaired=True)
    files = kept_files(SOURCE)
    assets = {r:p for r,p in files.items() if p.suffix.lower() in {'.hex','.mem','.mif','.bin'}}
    assert len(assets) == 104 and (SOURCE/'firmware/nav_fw.bin').stat().st_size == 53572
    result = dict(at=datetime.now().astimezone().isoformat(), passed=True,
        source=str(SOURCE), candidate=str(a.out), stage='plan_only',
        changed_RTL_relative_SYN44=sorted(rendered), source_hardware_inputs=63,
        all104_assets_and_CPU128KiB_and53572FW_preserved=True,
        source_clocks_CST_SDC_and_stop_priorities_preserved=True,
        NAV_source_functional_proof=str(nav_review), PID_real_component_proof=str(pid_review),
        action_source_contract=action, rendered_RTL_bytes={r:len(t.encode('utf8')) for r,t in rendered.items()},
        source_bindings=[metadata(p) for p in inputs.values()],
        proof_bindings=[metadata(p) for p in proofs],
        actual_new_mapping_RAM_INIT_clocks_CDC_and_timing_pending=True,
        automatic_SYN=False, automatic_PNR=False, programmed=False)
    if a.materialize:
        result['untouched_older_independent_preflight'] = no_owned_tools()
        result['natural_original_two_PNR_completion'] = natural_pair_complete()
        a.out.mkdir()
        before = {r:metadata(p) for r,p in files.items()}
        for r,p in files.items():
            q = a.out/r
            q.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(p,q)
        for r,text in rendered.items():
            (a.out/r).write_text(text,encoding='utf8',newline='\n')
        tcl = relocate((SOURCE/'run.tcl').read_text(encoding='utf8'), SOURCE, a.out)
        assert [x.strip() for x in tcl.splitlines() if x.strip().startswith('run ')] == ['run syn']
        assert not re.search(r'run pnr|inc_pnr', tcl)
        (a.out/'run.tcl').write_text(tcl,encoding='utf8',newline='\n')
        m = relocate(manifest,SOURCE,a.out)
        m.pop('effective_options',None)
        m.update(stage='syn', source_files=[str(a.out/r) for r in inputs],
                 frozen_source_build=str(SOURCE), source_preparation_only=True,
                 source_prepared_for_exact_one_SYN45=True, synthesis_complete=False,
                 modified_rtl_inputs=sorted(rendered), NAV_command_predecode=True,
                 PID_unity_reachable_bypass=True, private_action_preview=bool(a.action_overlay),
                 automatic_SYN=False, automatic_pnr=False, programmed=False,
                 inherited_receipts_are_ancestor_only=True, deployment_accepted=False)
        save(a.out/'manifest.json',m)
        assert all(before[r] == metadata(p) for r,p in files.items())
        assert all(same_bytes(p,a.out/r) for r,p in inputs.items() if r not in rendered)
        assert all(same_bytes(p,a.out/r) for r,p in assets.items())
        assert all(same_bytes(SOURCE/n,a.out/n) for n in ('g2.cst','g2.sdc'))
        result.update(stage='materialized_proved_source_new_actual_mapping_required',
            source_hardware_RTL_unchanged=63-len(rendered), all_assets_byte_identical=True,
            selected_source_bindings=[metadata(a.out/r) for r in inputs],
            build_command='retry_frozen_synthesis.py <candidate> --out <unique_SYN45> --start-now --tool-priority normal')
        save(a.out/'measured_timing_source_preparation.json',result)
    save(a.receipt,result)
    print(json.dumps(dict(stage=result['stage'], changes=sorted(rendered),
                          RTL_generated=a.materialize, EDA_started=False)))


if __name__ == '__main__':
    main()
