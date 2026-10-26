"""Collect the existing SYN43 storage proofs and their exact acceptance scope.

This starts no RTL/EDA process.  It does not turn arbitrary-state SAT into a
reachable counterexample or claim universal caller reachability from a source
operating contract.  Physical implementation is deliberately a later gate.
"""
import argparse
import json
from pathlib import Path


def metadata(path):
    s = path.stat()
    return dict(path=str(path), bytes=s.st_size, mtime_ns=s.st_mtime_ns)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--base', type=Path, default=Path('F:/nav_integration_g2_20261023'))
    ap.add_argument('--prepared', type=Path)
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args()
    prepared = a.prepared or a.base/'stage3_SLF128_storage_RTL_prepared8'
    names = {
        'actual43_memories': 'SLF128_storage_safeFW_syn43_memory_review1',
        'actual43_LF_trace': 'SLF128_joint43_LF_actual_trace1',
        'actual43_LF_trig_touch': 'SLF128_joint43_local_memory_mapping_review2',
        'actual43_weights': 'SLF128_joint43_actual_weights_official_probe1',
        'actual42_endpoint_control': 'SLF128_actual_memory_control_exclusion_review2',
        'actual42_PB_owner': 'SYN42_PB_owner_actual_binding4',
    }
    paths = {k:a.base/v/'result.json' for k,v in names.items()}
    proofs = {k:json.loads(p.read_text(encoding='utf8')) for k,p in paths.items()}
    m = proofs['actual43_memories']
    lf = proofs['actual43_LF_trace']
    local = proofs['actual43_LF_trig_touch']
    weight = proofs['actual43_weights']
    for k in ('actual43_memories','actual43_LF_trace','actual43_LF_trig_touch','actual43_weights'):
        assert proofs[k]['passed'], (k, 'existing proof did not pass')
    assert m['complete'] and m['actual_source_BSRAM'] == 269 and m['actual_wide_leaves'] == 11
    assert len(m['unchanged_RAM_vs_actual42_all_geometry_INIT']) == 231
    assert m['CPU_INIT'] == dict(passed=True, bitplanes=64, bytes=131072,
                                BIN_bytes=53572, selected_BIN_and_all_lanes_word_exact=True)
    assert m['actual_32_CEB_BLKSEL_read_gates_passed']
    assert len(m['actual_32_array_read_gate_bindings']) == 32
    assert lf['trace_complete'] and lf['scan_run_cycles'] == 16900 and lf['edges'] == 16937
    assert lf['run_prefetch_required'] and not lf['collision_events']
    assert local['LF_packing']['passed'] and local['touch_output_retiming']['passed']
    assert all(t['all_2x1024x16_INIT_words_match'] and t['retiming']['proved'] for t in local['trig'])
    assert any('all40704=1 reads=40708 idle_hold=81409 checks=122117' in x for x in weight['markers'])
    endpoint = proofs['actual42_endpoint_control']
    endpoint_pass = [g['group'] for g in endpoint['groups'] if g['universal_port_exclusion_proved']]
    assert set(endpoint_pass) == {'touch','slist','TXshadow','console'}
    pb = proofs['actual42_PB_owner']
    assert pb['passed'] and pb['UNSAT_queries'] == 31

    fw_app = prepared/'fw/app/nav_app.c'
    fw_hal = prepared/'fw/drv/hal_fpga.c'
    app = fw_app.read_text(encoding='utf8')
    calls = [dict(line=i, text=line.strip()) for i,line in enumerate(app.splitlines(),1)
             if 'hal_lidar_stage(' in line]
    assert len(calls) == 2
    assert 'hal_lidar_stage(HAL_LIDAR_FUSE_ONLY);' in calls[0]['text']
    assert 'matching ? HAL_LIDAR_MATCH_ONLY : HAL_LIDAR_NORMAL' in calls[1]['text']
    assert 'if (!g_app.pm_pend && !g_app.pm_run && hal_lidar_waiting(&ft))' in app
    assert 'if (!g_app.fuse_held) return;' in app
    vg = a.base/'SLF128_storage_safeFW_syn43/impl/gwsynthesis/g2_top.vg'
    manifest = json.loads((prepared/'manifest.json').read_text(encoding='utf8'))
    composition = json.loads((prepared/'composition_review.json').read_text(encoding='utf8'))
    source_proofs = set()
    def collect_proof_paths(obj):
        if isinstance(obj,dict):
            for value in obj.values():
                collect_proof_paths(value)
        elif isinstance(obj,list):
            for value in obj:
                collect_proof_paths(value)
        elif isinstance(obj,str) and obj.lower().endswith('.json') and (':/' in obj or ':\\' in obj):
            path = Path(obj)
            if path.is_file() and path.resolve() != (prepared/'composition_review.json').resolve():
                source_proofs.add(path)
    collect_proof_paths(composition.get('bindings',{}))
    for key in ('LF_RTL_validation','LF_small_C_mapping_review','weight_repair_proofs'):
        collect_proof_paths(manifest.get(key,{}))
    binding_files = [prepared/x for x in ('manifest.json','composition_review.json',
                     'source_contracts.json','weight_repair_binding_review.json',
                     'firmware_binding_review.json')]
    bindings = [metadata(p) for p in (*paths.values(), vg, *binding_files,
                                    prepared/'firmware/nav_fw.bin', fw_app, fw_hal)]
    result = dict(
        passed=True, complete=True,
        scope='Exploratory PNR storage entry under the selected normal source-operation contract',
        exploratory_PNR_permitted=True, exploratory_PNR_entry_passed=True, automatic_PNR=False,
        deployment_accepted=False,
        deployment_storage_safety_accepted=False,
        whole_G2_or_all_input_reachability_proved=False,
        current43_demonstrated_unresolved_public_numeric_or_reachable_collision_counterexamples=[],
        prior42_real_counterexamples_superseded_by_actual43_proofs=[
            'LF final drain read/write physical chunk overlap: repaired read gating and complete actual43 scan/drain verified',
            'Packed high-bank stale public weight: common high enable and official actual43 primitive all-weight/idle checks verified'],
        actual43_confirmed=dict(
            source_BSRAM=269, wide_leaves=11, split1_physical_prediction=280,
            physical_prediction_not_measured=True,
            unchanged_non_LF_non_weight_leaves_all_geometry_INIT=231,
            CPU=m['CPU_INIT'], array_read_gate_bindings=32,
            resources=m['synthesis_resources'], LF_scan_edges=16937,
            LF_conflicting_physical_chunks=0,
            weight_accepted_reads=40708, weight_idle_checks=81409,
            touch_trig_consumer_latency_and_capture_preserved=True),
        source_operation_contracts=[
            dict(group='point planes', actual43='All eight plane geometry, INIT and actual WE/RE/address inventory retained by memory proof; caller remapping is a separate conditional obligation',
                 inherited='Original ownership/RAW backpressure/WAIT_POINTS last-write drain RTL contract, with actual42 complete pending/owner/RAW binding:31 UNSAT',
                 assumptions='Ordered SOF and mode/config stable for each accepted pass. No arbitrary mid-frame debug CSR mode changes',
                 selected_FW='Only two stage writes; primary tick requires !pm_pend&&!pm_run&&hal_lidar_waiting. finish_fusion guarded by fuse_held and replays retained frame after matcher processing',
                 universal_actual43_reachability_proved=False),
            dict(group='bitmap', inherited='A/B write, C/D read, original sixteen-cycle flush/drain source contract',
                 actual43='Array enable/address inventory and true CEB/BLKSEL gating verified',
                 missing='Complete actual43 caller temporal phase invariant is not claimed. Arbitrary free-state SAT and failed induction overapproximation are retained, without a reachable trace'),
            dict(group='slist/touch', inherited='Original mutually exclusive state phases; actual42 endpoint next-state exclusions are UNSAT',
                 actual43='slist/touch storage geometry, INIT and real read gates retained; touch READ_MODE1 OCE equals original !reset&&S_U1 consumer capture',
                 missing='Full actual43 state-transition identity of all callers is not claimed'),
            dict(group='TX shadow/console', inherited='Original demand-read/post-commit source contracts; actual42 endpoint next-state exclusions UNSAT',
                 actual43='All leaf geometry/INIT and true CEB/BLKSEL read gates retained; nav_soc local module text identical',
                 missing='TX caller aliases/remapping require final actual43 binding; no proven new reachable collision'),
            dict(group='structure map and other inherited banks', inherited='Original accepted map/clear/prior/online operation ordering and existing model regressions',
                 actual43='All unaffected geometry and every INIT exact; no new storage-port implementation in this step',
                 missing='This report is not a universal all-input proof for existing shared-port callers')],
        selected_FW_stage_call_sites=calls,
        free_state_SAT_interpretation='32 arbitrary-FF same-chunk SAT cases are not 32 reachable collisions. Source lifecycle assumptions and actual endpoint mapping remain explicitly scoped; only the two concrete42 failures were treated as actual counterexamples',
        remaining_deployment_checks=[
            'Actual physical RAM split/INIT, all clocks setup/hold, effective exceptions, IO/reset pulse and CDC payload timing after PNR',
            'Bind remaining actual43 caller transitions to the stated normal operating contract, including PB/RAW handshake cuts and bitmap lifecycle; restrict debug/config writes to legal pass boundaries',
            'Same selected firmware/physical image end-to-end RK/P11 feedback, re-arm, independent ESTOP and physical stop verification',
            'No claim that two successful place options establish absolute routing robustness'],
        proof_paths={k:str(p) for k,p in paths.items()},
        inherited_source_proof_metadata=[metadata(p) for p in sorted(source_proofs)],
        all_input_bindings=bindings)
    a.out.mkdir(parents=True, exist_ok=False)
    (a.out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:result[k] for k in ('passed','complete','exploratory_PNR_permitted','deployment_storage_safety_accepted')},ensure_ascii=False))


if __name__ == '__main__':
    main()
