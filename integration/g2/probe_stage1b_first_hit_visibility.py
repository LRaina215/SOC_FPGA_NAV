"""Freeze/run a finite new-scene geometry probe, never a DLL or closed loop.

Uses unchanged frozen raycast/projection/structure/LF/package functions. Only
independent output files are written. Admission2GiB physical/4GiB commit.
"""
from pathlib import Path
from datetime import datetime
import argparse
import ctypes
import json
import math
import os
import shutil
import sys
import traceback

SOURCE = Path('F:/nav_integration_g2_20261023/nav3x5_N1c_final128_outer_height_stage1b_20261026')
GIB = 1024 ** 3


def write(p, value):
    p.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf8')


def read(p):
    return json.loads(p.read_text(encoding='utf8'))


def meta(p):
    s = p.stat()
    return dict(path=str(p.resolve()), bytes=s.st_size, mtime_ns=s.st_mtime_ns)


def memory():
    class M(ctypes.Structure):
        _fields_ = [('length', ctypes.c_uint32), ('load', ctypes.c_uint32)] + [(n, ctypes.c_uint64) for n in
                     ('total_phys', 'avail_phys', 'total_page', 'avail_page', 'total_virtual', 'avail_virtual', 'avail_extended')]
    v = M()
    v.length = ctypes.sizeof(v)
    assert ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(v))
    return dict(physical=v.avail_phys, commit=v.avail_page)


def prepare(out):
    assert not out.exists(), 'Use one fresh independent output'
    out.mkdir(parents=True)
    inputs = out / 'inputs'
    model = inputs / 'execution_tree/model'
    closed = inputs / 'execution_tree/sim/closed_loop'
    model.mkdir(parents=True)
    closed.mkdir(parents=True)
    files = []
    for p in (SOURCE / 'execution_tree/model').glob('*.py'):
        files.append((p, model / p.name))
    for name in ('world.py', 'stage1c_support.py', 'stage1b_window_math.py'):
        files.append((SOURCE / 'execution_tree/sim/closed_loop' / name, closed / name))
    files.append((SOURCE / 'outer_scene.json', inputs / 'outer_scene.json'))
    bindings = []
    for src, dest in files:
        shutil.copy2(src, dest)
        assert src.read_bytes() == dest.read_bytes()
        bindings.append(dict(source=meta(src), frozen=meta(dest), bytes_equal=True))
    poses = [
        ('west_end', .45, -.95, 0, 'original7'),
        ('middle_east', 2.5, -.6, 0, 'original7'),
        ('middle_west', 2.5, -.6, 180, 'original7'),
        ('east_end_west', 4.45, .95, 180, 'original7'),
        ('middle_north', 2.5, -.6, 90, 'original7'),
        ('middle_south', 2.5, -.6, -90, 'original7'),
        ('platform84_east', 1.45, .8, 0, 'original7_platform'),
        ('middle_turn45', 2.5, -.6, 45, 'turn_heading'),
        ('east_end_faces_person', 4.45, .95, math.degrees(math.atan2(1.525 - .95, 6.175 - 4.45)), 'face_new_person'),
        ('north_faces_person', 3.15, .85, 90, 'face_new_person'),
        ('platform84_faces_person', 1.45, .8, math.degrees(math.atan2(2.525 - .8, 3.175 - 1.45)), 'platform_face_person')
    ]
    write(out / 'poses.json', [dict(name=n, x=x, y=y, yaw_deg=a, group=g,
          odometry_world_origin_m=[.45, .85 if 'platform' in g else -.95, 0]) for n, x, y, a, g in poses])
    shutil.copy2(Path(__file__).resolve(), out / 'run_probe.py')
    cfg = dict(source=str(SOURCE), bindings=bindings, poses=meta(out / 'poses.json'), entry=meta(out / 'run_probe.py'),
               forward_mm=[1000, 1250, 1500], selected_forward_mm=None, ZHI=[350, 1500],
               estimated_peak_RSS_MiB=256, minimum_physical=2 * GIB, minimum_commit=4 * GIB,
               no_DLL=True, no_closed_loop=True, no_EDA=True,
               map_history='Cold map + each static frame repeated3 times as a warm self-history fixture; not an actual route',
               prior='Finite11-pose zero-noise fixture only, original package functions and innerXY crop; not the stage1b full survey cache')
    write(out / 'preparation.json', cfg)
    write(out / 'job.json', dict(phase='prepared_not_run', planned_poses=len(poses), new_closed_loop=False))
    return cfg


def run(out):
    cfg = read(out / 'preparation.json')
    for binding in cfg['bindings']:
        assert meta(Path(binding['frozen']['path'])) == binding['frozen']
    assert meta(out / 'poses.json') == cfg['poses'] and meta(out / 'run_probe.py') == cfg['entry']
    assert not (out / 'run_claim.json').exists(), 'No duplicate actual probe'
    mem = memory()
    if mem['physical'] < cfg['minimum_physical'] or mem['commit'] < cfg['minimum_commit']:
        write(out / 'job.json', dict(phase='waiting_memory_not_started', admission=mem,
              minimum_physical=cfg['minimum_physical'], minimum_commit=cfg['minimum_commit']))
        return dict(started=False, memory=mem)
    import psutil
    me = psutil.Process()
    me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    if 18 in me.cpu_affinity():
        me.cpu_affinity([18])
    identity = dict(pid=me.pid, created_epoch=me.create_time(), parent_pid=me.ppid(), executable=me.exe(), admission=mem)
    with (out / 'run_claim.json').open('x', encoding='utf8') as f:
        json.dump(identity, f, indent=2)
    def status(phase, **kw):
        write(out / 'job.json', dict(identity, phase=phase, updated_at=datetime.now().astimezone().isoformat(), **kw))
    try:
        status('importing_frozen_geometry')
        os.environ.update(SLF_SN_LOG2='7', SLF_LF_N='128', SLF_WINDOW_MODE='forward', SLF_ZLO='60', SLF_ZHI='1500',
                          SLF_NBR='3x3', SLF_W18='1', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
        tree = out / 'inputs/execution_tree'
        sys.path[:0] = [str(tree / 'model'), str(tree / 'sim/closed_loop')]
        import numpy as np
        from types import SimpleNamespace
        import world as W
        import sim_world as sw
        import slf_match as S
        import lidar_lut
        import nav_fast as fast
        import prior_map as P
        import stage1c_support as C
        from stage1b_window_math import forward_shift
        assert S.SN == S.LF_N == 128 and S.PB_MAX == 4096 and S.MAX_PTS == 1024
        spec = read(out / 'inputs/outer_scene.json')
        scene = C.make_scene(SimpleNamespace(W=W, sw=sw), spec)
        units = lidar_lut.rays().reshape(-1, 3)
        lut = np.array(lidar_lut.lut_entries(), dtype=np.int64)
        names = [p['name'] for p in spec['outer_primitives']]
        labels = ['inner_or_ground_or_unlabelled'] + names
        rows, captured = [], []
        def counts(which, lab):
            return {name: int(np.sum(which & (lab == i))) for i, name in enumerate(labels)}
        def physical_labels(hit, valid):
            # Exact unrounded ray-march hit, classified by the same raster cell
            # intervals/max-height composition used by the frozen World.box.
            lab = np.zeros(len(hit), dtype=np.int64)
            ties = np.zeros(len(hit), dtype=np.int64)
            ix, iy = scene.w._idx(hit[:, 0], hit[:, 1])
            heights = scene.w.height(hit[:, 0], hit[:, 1])
            for i, p in enumerate(spec['outer_primitives'], 1):
                x0, y0, x1, y1, h = p['args']
                (i0, j0), (i1, j1) = [scene.w._idx(np.array(x), np.array(y)) for x, y in ((x0, y0), (x1, y1))]
                m = valid & (ix >= i0) & (ix < i1) & (iy >= j0) & (iy < j1) & np.isclose(heights, h, atol=1e-6)
                ties[m] += 1
                lab[m & (lab == 0)] = i
            return lab, int(np.sum(ties > 1))
        def gate_view(sm, p, lab):
            ox, oy, occ, d3, *_ = sm.lf
            wx, wy = sm.cell(p[:, 0]) - ox, sm.cell(p[:, 1]) - oy
            margin = 0 if sm.prior else S.MARGIN
            core = (wx >= margin) & (wx < S.LF_N - margin) & (wy >= margin) & (wy < S.LF_N - margin)
            near = np.zeros(len(p), dtype=bool)
            near[core] = d3[wy[core], wx[core]] <= S.GATE_D
            score = sm.score(p[:, 0], p[:, 1], np.zeros((1, 3), dtype=np.int64), 0, 0)
            assert int(score[0, 0]) == int(near.sum()), 'Frozen score must agree with the measured gate masks'
            return dict(LF_origin=[int(ox), int(oy)], LF_N=S.LF_N, margin=margin, n_struct=int(occ.sum()),
                        core_points=int(core.sum()), effective_n_valid=int(near.sum()),
                        core_by_first_hit_object=counts(core, lab), gate_by_first_hit_object=counts(near, lab))
        for i, pose in enumerate(read(out / 'poses.json')):
            mem_now = memory()
            assert mem_now['physical'] >= GIB and mem_now['commit'] >= 2 * GIB, ('Runtime PNR reserve', mem_now)
            x, y, yaw = pose['x'], pose['y'], math.radians(pose['yaw_deg'])
            z = float(scene.height(np.array([x]), np.array([y]))[0])
            assert z <= .0841 and .22 < x < 4.78 and -1.28 < y < 1.28
            Rws, tws = sw.compose(sw.rotz(yaw), np.array([x, y, z]), *sw.mount_pose())
            directions = (Rws @ units.T).T
            distance = scene.w.raycast(tws, directions, tmax=8, step=.015)
            valid = np.isfinite(distance) & (distance >= .1) & (distance <= 8)
            hit = tws[None, :] + directions * np.where(valid, distance, 0)[:, None]
            lab, ambiguous = physical_labels(hit, valid)
            # Same zero-noise depth rounding/range checks as VirtualLidar.capture.
            depth = np.where(valid, np.clip(np.round(distance * 1000), 1, 65535), 0).astype(np.int64).reshape(60, 160)
            xyz, pv = fast.pc_project(depth, lut)
            origin = np.array(pose['odometry_world_origin_m'])
            Rq = np.round(Rws * 16384).astype(np.int64)
            tq = np.round((tws - origin) * 1000).astype(np.int64)
            p = fast.pc_transform(xyz, Rq, tq)
            worldp = fast.pc_transform(xyz, Rq, np.round(tws * 1000).astype(np.int64))
            zref = int(tq[2]) - 155
            vehicle_mm = np.round((np.array([x, y, z]) - origin) * 1000).astype(np.int64)
            center = (vehicle_mm[:2] * 1311) >> 16
            angle = int(round(yaw * 65536 / (2 * math.pi))) & 65535
            vertical = S.structure_mask(p, pv, zref, -32768, 32767)
            bands = []
            for upper in cfg['ZHI']:
                selected = S.structure_mask(p, pv, zref, 60, upper)
                idx = S.subsample(np.nonzero(selected)[0])
                points, plab = p[idx], lab[idx]
                warm = S.StructMap()
                for _ in range(3):
                    warm.update(points[:, :2])
                windows = []
                for d in cfg['forward_mm']:
                    shift = forward_shift(angle, d, 1311, S.nf.SIN)
                    cx, cy = center + np.array(shift['offset_cells'])
                    warm.build_lf(int(cx), int(cy))
                    windows.append(dict(forward_mm=d, selected_distance=False, shift=shift, **gate_view(warm, points, plab)))
                bands.append(dict(ZLO=60, ZHI=upper, selected_before_PB_cap=int(selected.sum()),
                                  PB_stored=min(int(selected.sum()), 4096), subsampled_points=len(idx),
                                  selected_by_first_hit_object=counts(selected, lab),
                                  subsampled_by_first_hit_object=counts(np.ones(len(idx), bool), plab),
                                  cold_no_LF_n_valid=0, warm_fixture='Three previous identical frames; not a real route history', windows=windows))
            captured.append(dict(p=worldp, pv=pv, lab=lab, zref=int(round(tws[2] * 1000)) - 155, foot_z_m=z))
            row = dict(pose=pose, actual_foot_height_m=z, exact_first_hit_pixels=int(valid.sum()),
                       fixed_project_valid_points=int(pv.sum()), first_hits_by_object=counts(valid, lab),
                       shared_equal_height_outer_corner_pixels=ambiguous,
                       vertical_pair_points=int(vertical.sum()), vertical_by_first_hit_object=counts(vertical, lab), bands=bands)
            rows.append(row)
            write(out / f"pose_{i:02d}_{pose['name']}.json", row)
            status('finite_pose_complete', completed_poses=len(rows), planned_poses=11, rss=me.memory_info().rss)
        prior = []
        for upper in cfg['ZHI']:
            sp, ap, pc = [], [], []
            for frame in captured:
                p, pv = frame['p'], frame['pv']
                m = S.structure_mask(p, pv, frame['zref'], S.Z_LO_PRIOR, upper)
                sp.append(p[m, :2])
                ap.append(p[pv & (p[:, 2] < (frame['foot_z_m'] + upper / 1000) * 1000)])
                pc.append(m)
            sp, ap, crop = C.crop_prior(np.concatenate(sp), np.concatenate(ap), [0, -1500, 5000, 1500])
            data, info = P.build_package(sp, ap, bounds_mm=[0, -1500, 5000, 1500])
            parsed = P.parse_package(data)
            sm = S.StructMap()
            sm.load_prior(parsed['ox'], parsed['oy'], parsed['slot'], parsed['word'])
            perpose = []
            for row, f, mask in zip(rows, captured, pc):
                idx = S.subsample(np.nonzero(mask)[0])
                perpose.append(dict(name=row['pose']['name'], selected_before_PB_cap=int(mask.sum()), subsampled_points=len(idx),
                                    **gate_view(sm, f['p'][idx], f['lab'][idx])))
            (out / f'finite11pose_prior_zhi{upper}.nvmp').write_bytes(data)
            prior.append(dict(ZLO=S.Z_LO_PRIOR, ZHI=upper, crop=crop, package_bytes=len(data),
                              n_struct=info['n_struct'], n_words=info['n_words'], actual_origin=[info['ox'], info['oy']],
                              fixture_not_full_prior_survey=True, perpose=perpose))
        totals = {n: sum(r['first_hits_by_object'][n] for r in rows) for n in names}
        result = dict(passed=True, recorded_at=datetime.now().astimezone().isoformat(), identity=identity,
                      scope='Finite noise-free11-pose first-hit/fixed-point/128LF diagnostic only; no DLL, CCPU, full survey or closed loop',
                      poses=len(rows), forward_mm=cfg['forward_mm'], selected_forward_mm=None,
                      actual_first_hit_totals_by_outer_object=totals,
                      outer_objects_with_zero_hits=[n for n in names if totals[n] == 0],
                      zero_new_human_visibility_claimed_as_success=False, rows=rows, finite_prior_fixture=prior,
                      limits=['Static rigid yaw-only body poses; no balance/IMU/slip/range noise or dynamic person motion',
                              'Object labels use exact unrounded first ray hit raster cells; equal-height wall corners can have shared identity and are reported separately',
                              'Warm near-gate counts use three repeated static frames, not evidence of a real route or long-patrol map history',
                              'Finite11-pose prior package follows original crop/encoding but is not the planned full noisy survey cache',
                              'No native firmware/DLL dispatch tested; prior actual C op5 evidence remains the earlier isolated proof',
                              'No localization-loss criterion or physical visibility guarantee is inferred from nonzero point counts'],
                      current_inputs_modified=False, model_DLL_rebuilt=False, EDA_started=False, new_closed_loop=False,
                      rss_at_finish=me.memory_info().rss, no_SHA=True)
        write(out / 'result.json', result)
        status('finite_geometry_probe_complete', completed_poses=len(rows), result=str(out / 'result.json'), rss=me.memory_info().rss)
        return result
    except Exception:
        status('failed_preserve_outputs', error=traceback.format_exc())
        raise


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--prepare', action='store_true')
    p.add_argument('--start', action='store_true')
    a = p.parse_args()
    if a.prepare:
        prepare(a.out)
        print(json.dumps(dict(prepared=str(a.out))))
    if a.start:
        r = run(a.out)
        print(json.dumps({k: r[k] for k in ('started', 'passed', 'poses', 'actual_first_hit_totals_by_outer_object', 'rss_at_finish') if k in r}, ensure_ascii=False))
