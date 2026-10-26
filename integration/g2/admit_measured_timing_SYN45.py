"""Combine independent actual SYN45 checks for physical exploration only."""
from datetime import datetime
from pathlib import Path
import argparse
import json

from audit_cdc_first_stages import sdc_fragment
from prepare_baseline_fusion_retime import load, metadata, same_bytes, save

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE/'SLF128_measured_timing_syn45'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    assert not args.out.exists()
    args.out.mkdir()
    locations = dict(
        storage=BASE/'SYN45_storage_clock_ring_final1/result.json',
        nav_pid=BASE/'SYN45_NAV_PID_actual_review3/result.json',
        action=BASE/'SYN45_action_preview_actual_review1/result.json')
    try:
        reports = {name:load(p) for name,p in locations.items()}
        st, nav, act = (reports[k] for k in ('storage','nav_pid','action'))
        assert st['passed'] and all(st[k] for k in (
            'storage_CPU_passed','ring_passed','two_clocks_passed','original26_passed',
            'FIFO_reset_passed','IR_passed','lease_passed'))
        assert nav['combined_actual_goal_passed'] and all(nav[k] for k in (
            'mapped_NAV_pulse_contract_passed','NAV_decode_removed_from_selected_consumer_CE',
            'mapped_PID_reachable_contract_passed','PID_target_amplification_removed',
            'IR_immediate_stop_preserved'))
        assert act['passed'] and all(act[k] for k in (
            'actual6_product_registers_verified','actual_source_core_actions_verified',
            'actual_signed_clip_gain_modes_verified','source_edge_contract_preserved'))
        j = load(SOURCE/'job.json')
        assert j['phase'] == 'synthesis_complete' and j['tool_exit'] == 0
        summary = load(SOURCE/'synthesis_summary.json')
        top = summary['synthesis_hierarchy'][0]
        n = lambda field:int(top[field].split('(')[0])
        assert n('T_Bsram') == 269
        assert n('T_Lut') < 45825 and n('T_Register') < 31120
        radar = Path(st['actual_roles26'])
        roles = load(radar)
        fragment = sdc_fragment(roles)
        assert len(roles['paths']) == 26 and fragment.count('set_false_path ') == 26
        assert fragment.count('set_max_delay ') == 26
        assert same_bytes(SOURCE/'g2.cst',BASE/'baseline_fusion32_CDC52_pnr35/g2.cst')
        prep = BASE/'SLF128_measured_timing_SYN45_preparation1.json'
        prepared = load(prep)
        assert prepared['all_assets_byte_identical'] and prepared['source_hardware_RTL_unchanged'] == 58
        proof_files = list(locations.values())+[radar,prep,SOURCE/'job.json',
            SOURCE/'synthesis_summary.json', SOURCE/'g2.cst', SOURCE/'g2.sdc',
            SOURCE/'impl/gwsynthesis/g2_top.vg']
        root = dict(passed=True,at=datetime.now().astimezone().isoformat(),
            source=str(SOURCE),actual_complete_SYN45=True,
            SYN_resource_gate_passed=True,actual_storage_INIT_and_read_gate_legal=True,
            actual_two_clocks_and_stop_review_passed=True,
            original26_revalidated_after_FIFO_reset_change=True,
            actual_NAV6_capture_and_consumer_mapping_review_passed=True,
            actual_PID_unity_and_IR_stop_mapping_review_passed=True,
            actual_action_preview_register_and_source_review_passed=True,
            actual_ring_mapping_or_bound_retention_review_passed=True,
            no_added_setup_or_whole_domain_exceptions=True,
            predicted_physical_BSRAM=280,actual_source_BSRAM=269,
            predicted_width_split_pairs=11,actual_SYN_LUT=n('T_Lut'),
            actual_SYN_FF=n('T_Register'),actual_SYN_ALU=n('T_Alu'),
            radar26_file=str(radar),actual_VG_binding=metadata(SOURCE/'impl/gwsynthesis/g2_top.vg'),
            actual_synthesis_timing_estimates=summary['synthesis_timing'],
            estimates_are_not_physical_timing_acceptance=True,
            independent_actual_reports={k:str(p) for k,p in locations.items()},
            bindings=[metadata(p) for p in proof_files],
            pending_deployment_items=[
                'Actual routing and all50/100MHz setup/hold/effective52/CDC payload/reset/IO',
                'Same final P11 command/arrival age/host-loss/BMI attitude preservation firmware',
                'Physical sensor freshness, independent stop and500ms standstill',
                'SLF128 long-term drift remains unaccepted; camera reserve remains unimplemented'],
            normal_ordered_frames_and_stable_config_scope=True,
            arbitrary_midbusy_debug_mode_changes_not_accepted=True,
            programmed=False,deployment_accepted=False)
        save(args.out/'result.json',root)
        print(json.dumps(dict(passed=True,root_review=str(args.out/'result.json'),
            predicted_BSRAM=280,LUT=n('T_Lut'),FF=n('T_Register'))))
    except BaseException as error:
        save(args.out/'failed.json',dict(passed=False,error=repr(error),
            reports_expected={k:str(p) for k,p in locations.items()},
            PNR_started=False,hardware_modified=False))
        raise


if __name__ == '__main__':
    main()
