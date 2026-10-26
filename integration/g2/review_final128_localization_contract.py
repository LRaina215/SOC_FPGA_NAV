"""Read frozen N1c interfaces and completed traces; never run or alter a model.

Writes only a new output directory. No DLL loading, hashes, model imports or jobs.
"""
from pathlib import Path
from datetime import datetime
import argparse
import csv
import json
import math
import struct
import numpy as np

BASE = Path('F:/nav_integration_g2_20261023')
BATCH = BASE / 'nav3x5_N1c_final128_multiseed_20261026'
PATROL = BASE / 'nav3x5_N1c_final128_long_patrol_20261026'
HEIGHT = BASE / 'nav3x5_N1c_final128_outer_height_stage1b_20261026'


def read(p):
    return json.loads(p.read_text(encoding='utf8'))


def pe_exports(p):
    """Parse the saved PE export directory without loading executable code."""
    b = p.read_bytes()
    pe = struct.unpack_from('<I', b, 0x3c)[0]
    assert b[:2] == b'MZ' and b[pe:pe + 4] == b'PE\0\0'
    coff = pe + 4
    ns = struct.unpack_from('<H', b, coff + 2)[0]
    optional = coff + 20
    size = struct.unpack_from('<H', b, coff + 16)[0]
    magic = struct.unpack_from('<H', b, optional)[0]
    assert magic in (0x10b, 0x20b)
    er, _ = struct.unpack_from('<II', b, optional + (112 if magic == 0x20b else 96))
    sections = []
    for i in range(ns):
        vs, va, rs, rp = struct.unpack_from('<IIII', b, optional + size + 40 * i + 8)
        sections.append((va, max(vs, rs), rp))

    def off(rva):
        for va, length, rp in sections:
            if va <= rva < va + length:
                return rp + rva - va
        raise ValueError(rva)

    exp = off(er)
    count = struct.unpack_from('<I', b, exp + 24)[0]
    names = off(struct.unpack_from('<I', b, exp + 32)[0])
    result = []
    for i in range(count):
        start = off(struct.unpack_from('<I', b, names + 4 * i)[0])
        result.append(b[start:b.index(0, start)].decode('ascii'))
    return dict(path=str(p), bytes=len(b), exports=result,
                export_count=len(result), executable_loaded=False)


def spans(mask):
    result = []
    start = None
    for i, v in enumerate(np.r_[mask, False]):
        if v and start is None:
            start = i
        elif not v and start is not None:
            result.append([start, i - 1, i - start])
            start = None
    return result


def completed_cell(job):
    root = Path(job['out'])
    metric = read(root / 'metrics.json')
    with (root / 'timeseries.csv').open(encoding='utf8', newline='') as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r['t']) for r in rows])
    error = np.array([1000 * math.hypot(float(r['x']) - float(r['ex']),
                                      float(r['y']) - float(r['ey'])) for r in rows])
    with np.load(root / 'final_maps.npz', allow_pickle=False) as z:
        match = z['match_log'].copy()
    assert len(match.shape) == 2 and match.shape[1] == 12
    count = match[:, 11].astype(np.int64)
    inc = np.r_[False, np.diff(count) > 0]
    starts = np.flatnonzero(inc)
    after = np.arange(len(match)) >= (int(starts[0]) if len(starts) else len(match))
    gaps = spans((~inc) & after)
    audit_path = root / 'window_audit.json'
    score = None
    if audit_path.exists():
        audit = read(audit_path)
        events = audit.get('events', [])
        scores = [e for e in events if e['op'] == 'score']
        builds = [e for e in events if e['op'] == 'build_lf']
        score = dict(start_callback_records=len(scores),
                     n_valid_lt60_start_callbacks=sum(e['effective_n_valid'] < 60 for e in scores),
                     zero_input_start_callbacks=sum(e['points_input'] == 0 for e in scores),
                     input_points_range=[min(e['points_input'] for e in scores), max(e['points_input'] for e in scores)] if scores else None,
                     effective_n_valid_range=[min(e['effective_n_valid'] for e in scores), max(e['effective_n_valid'] for e in scores)] if scores else None,
                     candidate_counts=sorted({e['candidates'] for e in scores}),
                     LF_build_records=len(builds),
                     has_event_timestamps=any('t' in e or 't_us' in e or 'now_us' in e for e in events),
                     meaning='Synchronous model scoring callback at HAL match_start; not a count of C solver invocations',
                     path=str(audit_path))
    return dict(name=job['name'], profile=job['profile'], case=job['case'], seed=job['seed'],
                prior=job['prior'], actual_end_s=metric['simulation_end_s'],
                trace_samples=len(rows), match_commit_samples=len(match),
                runner_loc_valid_all_one=all(float(r['loc_valid']) == 1 for r in rows),
                n_lost=metric['n_lost'], n_applied_final=int(count[-1]) if len(count) else None,
                counter_decreases=int(np.sum(np.diff(count) < 0)),
                counter_increment_observations=int(inc.sum()),
                post_first_increment_longest_repeat_commit_samples=max((s[2] for s in gaps), default=0),
                latest_status_low_valid_commit_samples=int(np.sum(match[:, 5] < 60)),
                latest_status_both_xy_degenerate_commit_samples=int(np.sum((match[:, 8] != 0) & (match[:, 9] != 0))),
                latest_status_applied_zero_delta_commit_samples=int(np.sum((match[:, 0] != 0) & (match[:, 1] == 0) & (match[:, 2] == 0) & (match[:, 4] == 0))),
                full_truth_XY_P95_mm=float(np.percentile(error, 95)),
                full_truth_XY_RMSE_mm=float(np.sqrt(np.mean(error ** 2))),
                full_truth_error_trend_mm_per_min=float(np.polyfit(t, error, 1)[0] * 60) if len(t) > 1 else None,
                score_audit=score, source=str(root),
                exact_match_timestamps_available=False, loss_free_proved=None)


def review(out):
    assert not out.exists(), 'Use a new independent evidence directory'
    state = read(BATCH / 'job.json')
    manifest = read(Path(state.get('active_manifest', str(BATCH / 'manifest.json'))))
    items, pending = [], []
    for j in manifest['jobs']:
        p = Path(j['out'])
        if j['name'] == state.get('current') or not all((p / f).exists() for f in ('metrics.json', 'timeseries.csv', 'final_maps.npz')):
            pending.append(j['name'])
        else:
            items.append(completed_cell(j))
    dll = pe_exports(BATCH / 'nav_fw.dll')
    assert all(name.startswith('py_') for name in dll['exports'])
    geometry = read(HEIGHT / 'static_scene_contract.json')
    old_geometry = read(BASE / 'nav3x5_N1c_stage1c_outer_height_20261026/minimal_proof.json')
    old_rows = old_geometry['outputs'][0]['exterior_samples']
    old_totals = {k: sum(v[k] for v in old_rows) for k in ('actual_exterior_geometry_valid_hits', 'exterior_structure1500', 'exterior_structure350')}
    source = BATCH / 'firmware_source'
    execution = BATCH / 'execution_tree'
    anchors = [
        [execution / 'sim/closed_loop/run_cl.py', 344, 'Polling and commit sampling; match_log has no timestamps'],
        [execution / 'sim/closed_loop/run_cl.py', 380, 'Public simulated time set every1ms'],
        [execution / 'sim/closed_loop/run_cl.py', 386, 'No-map localized is unconditional'],
        [execution / 'sim/closed_loop/final_forward_case.py', 52, 'Scoring callback audit: input/effective points and candidates; no times'],
        [source / 'test/hal_py.c', 119, 'py_get_state fields exclude pose timestamp'],
        [source / 'test/hal_py.c', 128, 'py_get_loc forces no-map loc_state0'],
        [source / 'test/hal_py.c', 137, 'py_get_match12 fields exclude n_runs, n_in_best, at_edge'],
        [source / 'test/hal_py.c', 334, 'Synchronous score callback and fixed40000us modeled latency'],
        [source / 'nav/pose_match.c', 36, 'Solver starts with appliedfalse; candidate count and min_valid acceptance'],
        [source / 'nav/pose_match.c', 155, 'Successful solve can apply all-zero correction and increment n_applied'],
        [source / 'app/nav_app.c', 130, 'One pending frame gated by pm_pend/pm_run and attitude'],
        [source / 'app/nav_app.c', 205, 'loc_track called only when prior is true'],
        [source / 'nav/loc.c', 164, 'Existing prior-mode quality/blind/edge rules'],
        [HEIGHT / 'execution_tree/model/slf_match.py', 96, 'Vertical-pair structure selection and relative inclusive height band'],
        [HEIGHT / 'execution_tree/model/slf_match.py', 224, 'Predicted-position LF core and prior-near gating before candidate scoring'],
        [HEIGHT / 'execution_tree/sim/closed_loop/stage1c_support.py', 39, 'Both prior point arrays cropped to inner XY ROI'],
        [HEIGHT / 'execution_tree/sim/closed_loop/stage1c_support.py', 56, 'Actual C op5 height interception'],
        [HEIGHT / 'execution_tree/model/sim_world.py', 48, 'First-hit solid-column ray occlusion'],
        [HEIGHT / 'execution_tree/model/lidar_lut.py', 82, 'Real intrinsic/distortion/corner-mask unit rays, not an ideal nominal FOV'],
    ]
    result = dict(
        recorded_at=datetime.now().astimezone().isoformat(),
        scope='Read-only source/interface/finished-output audit; no model, DLL execution, EDA or current-input changes',
        conclusion=dict(no_map_loss_free_proved=False,
                        reason='The existing no-map flag is constant and no no-map freshness/observability loss monitor is configured. Truth P95 and matching activity are distinct evidence.',
                        existing_finite_truth_error_evidence=True, exact_solver_freshness_retrospectively_recoverable=False),
        batch_snapshot=state, long_patrol_snapshot=read(PATROL / 'job.json'),
        completed_trace_rows=len(items), not_read_incomplete_or_current=pending,
        interface=dll,
        exported_match_fields=['applied','dx','dy','reserved0','dyaw','n_valid','cost0','cost_best','degen_x','degen_y','degen_yaw','n_applied'],
        internal_not_exported=['pm.n_runs','pm.n_in_best','pm.at_edge','pose.cur.t_us','pm_pend','pm_run','n_skip','n_lut_skip'],
        reusable=['10Hz timestamped truth/estimated XY/yaw and physical speed in timeseries.csv',
                  'Commit-ordered last solve result, valid point count, degen flags, cumulative n_applied in match_log',
                  'Forward profiles: ordered scoring-start callback input/effective point counts,245 candidates, LF origin/center/n_struct in window_audit.json',
                  '330s native waypoint transitions and actual time can establish finite route cycles, full/rolling errors and collisions'],
        missing=['Exact frame/commit/scoring/completion/accepted-solve timestamps',
                 'Internal n_runs/failed-solve counter and at_edge/n_in_best',
                 'Why each frame skipped: LUT/attitude/map reception/disable/stale',
                 'A defined no-map loss-age/degeneracy rule and sustained-drift numeric threshold'],
        interpretation_limits=['n_applied increment can accompany zero correction and degenerate axes; it is acceptance, not proof of fully constrained XY',
                               'Repeated commit statuses are samples, not counts of failed solver attempts',
                               'Multiplying a commit gap by0.1s is nominal, not an exact solve age; frame handshaking can delay frames',
                               'Do not pair match_log and10Hz tlog by array index to claim exact movement-aligned gaps',
                               'Prior2s low-quality/edge_n6 rules are not no-map rules; n_lost0 in no-map does not test a loss detector',
                               'Current controller has already frozen the original two patrol inputs; leave them unchanged'],
        future_minimum_observer=dict(
            run_now=False, rebuild_DLL=False, mutate_current_inputs=False,
            preparation='New independent entry/copy only after current input freeze; wrap the existing Firmware factory and model callbacks without replacing numeric methods',
            time_source='Observe original py_set_time(uint32) arguments; record unsigned microseconds, poll ordinal and phase, preserving every original call and return',
            dispatch='Wrap original FpgaModel.match_eval, log scoring sequence/time/commit sequence/k/points input and all result n_valid,n_in,n_pts,cost; return original array unchanged',
            completion='Wrap original FpgaModel.match_update (HAL completion callback), record original time and public py_get_match/py_get_state before/after; zero delta update can also be stale/cancel, so label as update completion, not unconditional solver run',
            success='Read-only py_get_match after each original py_poll; log every n_applied delta and accompanying valid/degen/correction. No extra polling or consuming py_get_commit calls',
            frame='Wrap original frame/commit calls and the already-called py_get_commit result; log capture/commit sequence/time, selected points, height band and LF core/gate context',
            accuracy='Retain original10Hz timestamped truth/estimate/physical speed; record exact sampled timestamps alongside public pose. Logging adds wall time only, not accelerated hardware or modified simulated clock',
            acceptance='Report exact accepted-solve ages while actually moving, point starvation and axis degeneracy separately from truth P95. A new numerical no-map loss criterion needs explicit agreement; do not invent it from prior rules',
            unavoidable_limit='The existing DLL cannot expose exact C n_runs or every early rejection reason. Public callback sequences can bound starts/completions; internal counters require a future separately reviewed interface change, not this frozen DLL'),
        height_pilot=dict(
            snapshot=read(HEIGHT / 'job.json'), planned=16,
            selected_distance_bound=(HEIGHT / 'bound_manifest.json').exists(),
            DLL_bytes_equal_N1c=(HEIGHT / 'nav_fw.dll').read_bytes() == (BATCH / 'nav_fw.dll').read_bytes(),
            geometry_pass_is_visibility_pass=False,
            new_human_visibility_measured=geometry['new_human_visibility_measured'],
            inherited_original7_pose_totals=old_totals,
            inherited_original_pose_with_zero_exterior_hits=[r['world_body_pose'] for r in old_rows if r['actual_exterior_geometry_valid_hits'] == 0],
            human_model='Two static350x350mm solid heightfield prisms,1700mm high, not articulated people/moving obstacles',
            height_semantics='Online mask is z_relative_to_estimated_foot in inclusive[60,ZHI] no-map; prior mode changes lower bound to-300. Vertical-pair test and subsampling also apply',
            prior_semantics='Survey upper=zcap and both structure/all-point XY arrays cropped to[0,-1500,5000,1500]; live exterior rays still exist but near-prior gate can remove them. This changes usable points, not visibility itself',
            geometry_reason='500mm inner boards can occlude outer prisms. First-hit rays use actual distorted M60/corner mask and10deg-down155mm mount;1700mm object height does not exclude lower side returns at1500mm',
            startup_minimum_checks=[
                'Bind selected forward distance from the completed72-case selection, freeze fresh manifest; do not launch SELECTED_BEST_MM placeholders',
                'Before16 closed loops, use a separately authorized finite diagnostic of the new scene at original7 poses plus legal headings facing each person. Count actual first-hit identities per object; report zero visibility instead of treating box placement as evidence',
                'For each such capture record valid raw hits, inner/exterior/object labels, vertical-pair selection, selected z-relative min/max, PB4096/subsample1024 and effective post-LF-core/near-gate counts separately',
                'Compare350/1500 on the same captured depths/poses, preserve original board heights and inner geometry. At foot<=84mm,350-relative ends<=434mm<500mm board only under the stated unobstructed geometry/pose assumptions',
                'Assert actual C op5 and cold MAP_CLEAR dispatch for every cell; prior/no-prior lower bounds are distinct, log applied band rather than environment only',
                'Prior caches must be fresh byscenev2/SN/LF/ZLO/ZHI/ROI; assert both cropped arrays contain no exterior XY before packaging and keep prior LF fixed',
                'For pilot only, attach the separate timestamp observer if approved; keep localization-loss unknown where the frozen DLL supplies no loss rule'
            ],
            pending='No new ray or closed-loop run was performed by this audit'),
        source_anchors=[dict(path=str(p), line=line, evidence=e) for p, line, e in anchors],
        rows=items,
        no_SHA=True, active_inputs_modified=False, new_jobs_started=False)
    out.mkdir(parents=True)
    (out / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    value = review(args.out)
    print(json.dumps(dict(out=str(args.out), completed_read=value['completed_trace_rows'],
                          DLL_export_count=value['interface']['export_count'],
                          no_map_loss_free_proved=False, no_models_started=True), ensure_ascii=False))
