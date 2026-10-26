"""Small source-only collision-lifecycle review; no RTL/EDA execution.

The state/latency reasoning below is a static review, not a formal model check.
Run with --out NEW_DIRECTORY. Only that directory is written.
"""
from __future__ import annotations

import argparse
import difflib
import json
from pathlib import Path


BASE = Path('F:/nav_integration_g2_20261023')
OLD = BASE / 'baseline_fusion50_ring_MAX_syn40'
POINT = BASE / 'baseline_fusion50_pointplanes_prepared1'
STORES = BASE / 'stage2B_bitmap_mailbox_slist_trig_console_bias_prepared1'
GUIDE = BASE / 'stage2B_BSRAM_mode_collision_review1/UG300.txt'


def review(out: Path) -> dict:
    if out.exists():
        raise FileExistsError(f'Will not replace existing evidence: {out}')
    sources = {}
    excerpts = []

    def read(root: Path, name: str) -> str:
        p = root / name
        text = p.read_text(encoding='utf-8')
        sources[str(p)] = {'bytes': p.stat().st_size, 'lines': len(text.splitlines())}
        return text

    def excerpt(root: Path, name: str, lo: int, hi: int):
        text = read(root, name).splitlines()
        assert 1 <= lo <= hi <= len(text), (name, lo, hi, len(text))
        excerpts.append({'path': str(root / name), 'first_line': lo,
                         'last_line': hi, 'text': '\n'.join(text[lo-1:hi])})

    matches = 'nav/rtl/match/slf_match.v'
    extract = 'nav/rtl/match/slf_extract.v'
    trav = 'nav/rtl/map/trav_v3.v'
    top = 'nav/rtl/top/nav_subsystem.v'
    rx = 'nav/rtl/lidar/xt_frame_rx.v'
    hal = 'fw/drv/hal_fpga.c'
    app = 'fw/app/nav_app.c'
    binding = {}
    for name in (matches, extract, top, rx, hal, app):
        read(OLD, name)
        read(POINT, name)
        binding[name] = (OLD/name).read_bytes() == (POINT/name).read_bytes()
    assert all(binding.values()), binding
    candidate_diffs = {}
    for name in (matches, trav):
        old, new = read(OLD, name), read(STORES, name)
        assert new.replace(', .STAGE2B_BLOCK(1)', '') == old, name
        candidate_diffs[name] = list(difflib.unified_diff(
            old.splitlines(), new.splitlines(), 'SYN40', 'stage2B', n=2))

    sm = read(POINT, matches)
    ex = read(POINT, extract)
    tv = read(STORES, trav)
    guard_strings = {
        'point_new_sof_restarts_writer': 'act <= en; u <= 0; r <= 0; n_pts <= 0; busy <= en;' in ex,
        'point_read_write_independent': '.we(pb_we), .waddr(pb_wa), .wdata(pb_wd), .re(pb_re)' in sm,
        'eval_no_extraction_guard': 'else if (cmd_eval || cmd_update) begin\n                    nsnap <= n_pts;' in sm,
        'point_basis_changes_on_sof': 'if (s_fire && s_sof && frame_en) begin base_x <= cfg_tx; base_y <= cfg_ty; end' in sm,
        'ready_sticky_done': "if (ex_done) pts_ready <= 1'b1;" in sm,
        'bitmap_flush_16': "r <= 0; flush <= 5'd16;" in tv,
        'bitmap_read_C_D_only': 'assign bm_re    = ((ph == P_C) && inr0) || ((ph == P_D)' in tv,
        'slist_default_both_off': "sl_we <= 1'b0; sl_re <= 1'b0;" in sm,
        'slist_write_U3': "sl_we <= 1'b1; sl_wa <= ncl[9:0];" in sm,
        'slist_read_UC0': "sl_re <= 1'b1; sl_ra <= cli[9:0]; st <= S_UC1;" in sm,
    }
    assert all(guard_strings.values()), guard_strings
    for name, lo, hi in (
        (matches, 86, 102), (matches, 126, 143), (matches, 177, 185),
        (matches, 348, 401), (matches, 408, 461), (matches, 488, 521),
        (extract, 93, 120), (extract, 145, 156),
        (top, 202, 212), (top, 375, 413), (rx, 395, 428),
        (hal, 106, 124), (hal, 185, 239), (hal, 331, 354),
        (app, 170, 241), (app, 74, 88),
        ('nav/rtl/match/slf_lf.v', 225, 244),
    ):
        excerpt(OLD, name, lo, hi)
    for lo, hi in ((56, 94), (357, 424), (444, 447), (464, 499), (509, 545)):
        excerpt(STORES, trav, lo, hi)
    excerpt(POINT, 'nav/rtl/common/nav_ram_sdp.v', 8, 29)
    excerpt(GUIDE.parent, GUIDE.name, 310, 320)
    excerpt(GUIDE.parent, GUIDE.name, 1136, 1158)

    result = {
        'scope': 'Source-only point producer/consumer, bitmap and slist collision review',
        'method': 'Read exact frozen sources and inspect reachable state/control paths; no simulator, synthesis, PNR, SAT or CPU run.',
        'production_or_frozen_source_modified': False,
        'hardware_collision_safety_established': False,
        'vendor_contract': {
            'pdf': 'D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/doc/EN/UG300-1.3.6E_Arora V BSRAM & SSRAM User Guide.pdf',
            'local_text': str(GUIDE),
            'section': '2.1 p4 note and 3.3 p24',
            'rule': 'Cross-port simultaneous read/write to the same physical address is prohibited; equality is on input AD[13:4]. SDP supports Normal write mode. 138K does not support read-before-write.',
            'consequence': 'RTL nonblocking old-data semantics and prior vendor-model tests do not establish electrical collision behavior.',
        },
        'source_bindings': {
            'point_lifecycle_FW_exact_bytes_equal_SYN40': binding,
            'store_candidate_only_opt_in_parameter_differences': candidate_diffs,
            'critical_source_guards_checked': guard_strings,
        },
        'bitmap': {
            'rtl_nonoverlap': 'proved by static state/pipeline inspection after valid reset; no we and re overlap at all',
            'reads': 'Only current phase C or D (trav_v3:359).',
            'writes': 'Only delayed phase A or B tokens (trav_v3:530-545).',
            'drain_proof': [
                'Let e0 be the final B scan edge, which enters FLUSH with counter16.',
                'That final token reaches v1@e0, wval@e1, s2@e2, s3@e3, s4@e4, s5@e5, s6@e6, s7@e7, s8@e8, bm_we@e9; RAM samples its write at e10.',
                'FLUSH consumes old counter16 through0 at e1..e17; phase C begins after e17, and the first C RAM read samples at e18.',
                'Thus even the latest B write precedes first C read by8 clocks. A/B do not request bitmap reads; C/D never generate bitmap writes. Reset clears all valid tokens and write enable.',
            ],
            'firmware_dependency': None,
            'physical_pending': 'Verify actual mapped read CEB/BLKSEL is disabled when RTL re=0. OCE-only output hold with CEB always active is insufficient.',
            'minimum_change': 'No new consumer cycle or lifecycle change required if physical read enable respects re; qualify actual primitive at stage3.',
        },
        'slist': {
            'rtl_nonoverlap': 'proved by mutually exclusive assignments in one state case after valid reset',
            'writes': 'Only S_U3 sets registered sl_we; default0 each edge.',
            'reads': 'Only S_UC0 sets registered sl_re; default0 each edge. These are mutually exclusive case arms.',
            'nearest_transition': [
                'e0 state U3 sets sl_we1 and moves to T0.',
                'e1 RAM writes; T0 can enter UC0 when all sampled points finished.',
                'e2 UC0 sets sl_re1; RAM has no slist operation.',
                'e3 RAM reads while state UC1. Later UC2 clears touch bitmap, not slist.',
            ],
            'firmware_dependency': None,
            'physical_pending': 'Same actual CEB/BLKSEL qualification as bitmap; no inferred-port assumption.',
            'minimum_change': 'Keep existing latency/state order; require effective physical read enable in mapped review.',
        },
        'point_buffer': {
            'rtl_nonoverlap': False,
            'capacity': '4096x36; candidate4 banks x(low18+high18), each1024 deep. No truncation or added read stage in candidate.',
            'ungated_paths': [
                'slf_extract .en(frame_en), independent of matcher st/ex_busy/pts_ready (slf_match:92-101).',
                'Every accepted SOF resets producer n_pts to0 and base_x/base_y even while EVAL/UPDATE runs (extract:104-106; match:365).',
                'IDLE accepts EVAL/UPDATE without testing pts_ready, ex_busy or pending pb_we (match:379-390).',
                'busy is a status OR, not admission control; mt_frame_en contains only enable and valid height band (top:385).',
            ],
            'concrete_RTL_reachable_collision': [
                'Complete one extraction with exactly one retained point. Start EVAL with multiple candidates while in IDLE; nsnap=1.',
                'Each candidate visits S_T0 with j0, causing reads of address0. Start a new accepted extraction SOF during this EVAL; no RTL check blocks it.',
                'Choose the legal new input-point timing/vertical-edge data so the first emitted point produces registered pb_we1,pb_wa0 on an edge immediately before one of those repeated pb_re1,pb_ra0 samples.',
                'The next common RAM edge has both enables and the same logical/bank address0, therefore the same physical row for any faithful point mapping. No CDC, X, invalid address or point truncation is needed.',
            ],
            'witness_limit': 'A legal RTL interface/state witness, not an executed simulator trace or proof this exact timing is generated by the frozen Pico FW.',
            'normal_firmware_serialization': [
                'WAIT_POSE is enabled; MATCH_ONLY retains the original raw frame (hal:113-123; top:206-207; xt_frame_rx:410-427).',
                'nav_app only commits a new first pass when both pm_pend and pm_run are false (app:174-175); pm_pend waits for ready, pm_run waits for EVAL_DONE.',
                'UPDATE polls busy before finish_fusion releases the retained frame; FUSE_ONLY replay is excluded from slf_match s_fire (app:234-235,274-280; hal:222-233; top:388).',
            ],
            'firmware_contract_not_general_proof': [
                'Readiness is sticky across SOF. Normal valid-band frames while receiving a map or in prior LOC_ODOM can still extract even though matching=false, leaving pts_ready1 without UPDATE. A later matching frame can therefore observe readiness from an earlier frame. This proves a missing generation contract; physical collision under that exact frozen CPU timing is not established here.',
                'HAL start checks only enable and candidate count; no busy/current-frame check. UPDATE and wait-idle functions return void at a finite timeout, so timeout success is not checked by callers.',
                'These gaps prevent treating FW conventions as an unconditional hardware mutual-exclusion invariant. They do not by themselves prove a collision in ordinary matched-frame execution.',
            ],
            'update_timeout_audit': {
                'assumptions': 'Accepted online UPDATE, N256/PB4096, valid reset state, uninterrupted50MHz control clock, no asynchronous register-mode/reset changes.',
                'sampled_points_max': 1024,
                'point_states_per_sample': 11,
                'slist_clear_states_per_entry': 3,
                'slist_entries_max': 1024,
                'LF_scan_edges': 258*258,
                'LF_flush_counter': 28,
                'conservative_total_control_cycles_less_than': 82000,
                'conservative_ms_at_50MHz_less_than': 1.64,
                'conclusion': 'The5ms timeout is not an established reachable timeout for the fixed ordinary accepted UPDATE. Do not cite it as a demonstrated collision mechanism.',
            },
            'minimum_robust_avoidance': [
                'Serialize ownership at existing raw-frame WAIT_POSE/start boundary: keep a full accepted frame waiting while the previous point generation is owned by EVAL/UPDATE; defer/ack or hold a pending commit rather than dropping its one-cycle pulse.',
                'Accept or queue EVAL/UPDATE only for the completed current point generation, after extraction including its final registered pb_we has drained. Clear/replace ready on accepted new SOF and release ownership only after all point-dependent work is finished.',
                'Preserve the existing one-cycle enabled point-RAM read and all4096 points. Add control/handshake state, not another point RAM. Exact FF/config-pending cost needs implementation; no new BSRAM is inherently required because raw-frame waiting already exists.',
                'A simple frame_en &= !busy can silently discard a frame; gating only RAM reads/writes can corrupt output or truncate points. Address equality/bypass alone also fails to preserve old-data semantics or frame/base coherence. Do not use these as lifecycle fixes.',
                'If firmware-only admission is chosen, make current-generation readiness and successful idle/release explicit, cover map-upload/mode/pose transitions, and retain raw-frame backpressure on timeout. This is a new verified contract, not already proved by the old tests.',
            ],
        },
        'acceptance_pending': [
            'Actual stage3 SDP primitive geometry, AD collision rows, CEA/CEB/BLKSEL and OCE semantics for all migrated stores.',
            'Point ownership/admission implementation and focused temporal proof/assertions, if pointplanes remain selected.',
            'No claim of physical, full Pico FW,350ms stopping, routing or deployment acceptance.',
        ],
        'source_inventory': sources,
    }
    out.mkdir(parents=True)
    (out/'source_excerpts.json').write_text(json.dumps(excerpts, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    (out/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    r = review(args.out)
    print(json.dumps({'out': str(args.out), 'point_RTL_exclusion': False,
                      'bitmap_slist_RTL_exclusion': True,
                      'hardware_safety': r['hardware_collision_safety_established']}, ensure_ascii=False))
