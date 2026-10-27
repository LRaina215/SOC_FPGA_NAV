"""Convert a measured small-field JSON description to XYZ points in metres.

This generates geometry, not a SLAM trajectory or a verified site survey.
Supported objects are wall segments, solid rectangular boxes/platforms and
straight rectangular ramps. A low box can describe a step.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def samples(start, end, spacing):
    return np.linspace(start, end, max(1, math.ceil(abs(end - start) / spacing)) + 1)


def segment(a, b, height, spacing, base=0.):
    a, b = np.asarray(a, float), np.asarray(b, float)
    ts, zs = np.meshgrid(samples(0., 1., spacing / max(np.linalg.norm(b - a), spacing)),
                         samples(base, base + height, spacing))
    xy = a + ts.reshape(-1, 1) * (b - a)
    return np.column_stack((xy, zs.reshape(-1)))


def box_points(o, surface_spacing, wall_spacing):
    center, size = np.asarray(o['center'], float), np.asarray(o['size'], float)
    if center.shape != (2,) or size.shape != (3,) or np.any(size <= 0):
        raise ValueError('box center=[x,y], size=[length,width,height], all sizes positive')
    angle = math.radians(float(o.get('yaw_deg', 0)))
    rot = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * size[:2] / 2
    corners = corners @ rot.T + center
    base = float(o.get('base_z', 0))
    sides = [segment(corners[i], corners[(i + 1) % 4], size[2], wall_spacing, base) for i in range(4)]
    u, v = np.meshgrid(samples(-size[0] / 2, size[0] / 2, surface_spacing),
                       samples(-size[1] / 2, size[1] / 2, surface_spacing))
    xy = np.column_stack((u.reshape(-1), v.reshape(-1))) @ rot.T + center
    top = np.column_stack((xy, np.full(len(xy), base + size[2])))
    return np.vstack([*sides, top]), (center, size, rot)


def ramp_points(o, surface_spacing, wall_spacing):
    a, b = np.asarray(o['start'], float), np.asarray(o['end'], float)
    width, rise, base = float(o['width']), float(o['rise']), float(o.get('base_z', 0))
    delta = b - a
    length = np.linalg.norm(delta)
    if a.shape != (2,) or b.shape != (2,) or length <= 0 or width <= 0 or rise <= 0:
        raise ValueError('ramp requires distinct start/end, positive width/rise')
    direction = delta / length
    lateral = np.array([-direction[1], direction[0]])
    u, v = np.meshgrid(samples(0, length, surface_spacing), samples(-width / 2, width / 2, surface_spacing))
    xy = a + u.reshape(-1, 1) * direction + v.reshape(-1, 1) * lateral
    parts = [np.column_stack((xy, base + rise * u.reshape(-1) / length))]
    # Side faces and high-end face; no hidden flat floor beneath the ramp.
    for dist in samples(0, length, wall_spacing):
        z = samples(base, base + rise * dist / length, wall_spacing)
        for side in (-1, 1):
            xy = a + dist * direction + side * width / 2 * lateral
            parts.append(np.column_stack((np.tile(xy, (len(z), 1)), z)))
    parts.append(segment(b - width / 2 * lateral, b + width / 2 * lateral, rise, wall_spacing, base))
    return np.vstack(parts), (a, direction, lateral, length, width)


def generate(scene):
    if scene.get('units') != 'm':
        raise ValueError('units must explicitly be m')
    x0, y0, x1, y1 = map(float, scene['bounds'])
    surface = float(scene.get('surface_spacing', .025))
    wall = float(scene.get('wall_spacing', .02))
    if not (x0 < x1 and y0 < y1 and 0 < surface <= .05 and 0 < wall <= .05):
        raise ValueError('invalid bounds or sampling spacing; use at most5cm')
    if not np.isfinite([x0, y0, x1, y1, surface, wall]).all():
        raise ValueError('all geometry values must be finite')
    x, y = np.meshgrid(samples(x0, x1, surface), samples(y0, y1, surface))
    floor_xy = np.column_stack((x.reshape(-1), y.reshape(-1)))
    floor_keep = np.ones(len(floor_xy), dtype=bool)
    parts = []
    for o in scene.get('boxes', []):
        cloud, (center, size, rot) = box_points(o, surface, wall)
        local = (floor_xy - center) @ rot
        floor_keep &= ~((np.abs(local[:, 0]) <= size[0] / 2 + 1e-9) &
                        (np.abs(local[:, 1]) <= size[1] / 2 + 1e-9))
        parts.append(cloud)
    for o in scene.get('ramps', []):
        cloud, (a, d, v, length, width) = ramp_points(o, surface, wall)
        local = floor_xy - a
        u, lat = local @ d, local @ v
        floor_keep &= ~((u >= -1e-9) & (u <= length + 1e-9) & (np.abs(lat) <= width / 2 + 1e-9))
        parts.append(cloud)
    for o in scene.get('walls', []):
        height = float(o['height'])
        if height <= 0:
            raise ValueError('wall height must be positive')
        parts.append(segment(o['start'], o['end'], height, wall, float(o.get('base_z', 0))))
    floor = np.column_stack((floor_xy[floor_keep], np.full(int(floor_keep.sum()), float(scene.get('floor_z', 0)))))
    points = np.vstack([floor, *parts])
    if not np.isfinite(points).all():
        raise ValueError('nonfinite object geometry')
    return points


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('scene', type=Path)
    ap.add_argument('-o', '--out', type=Path, required=True)
    args = ap.parse_args()
    scene = json.loads(args.scene.read_text(encoding='utf-8-sig'))
    points = generate(scene)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.out, points, fmt='%.5f')
    info = {'source_scene': str(args.scene.resolve()), 'survey_status': scene.get('survey_status', 'not_declared'),
            'units': 'm', 'points': len(points), 'bounds': scene['bounds'],
            'minimum_xyz': points.min(axis=0).tolist(), 'maximum_xyz': points.max(axis=0).tolist(),
            'source_pose': scene.get('start_pose'), 'real_site_validated': False}
    args.out.with_suffix('.survey.json').write_text(json.dumps(info, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    print(json.dumps(info, ensure_ascii=False))


if __name__ == '__main__':
    main()
