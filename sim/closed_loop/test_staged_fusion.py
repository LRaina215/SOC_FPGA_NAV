"""Directed two-pass contract: no first-pass map write, corrected second pass only.

Synthetic points isolate frame ordering from the terrain classifier and sensors.
"""
import json
from pathlib import Path
import numpy as np
from fpga_model import FpgaModel


def main():
    m = FpgaModel()
    m.xyz = np.array([[500, 0, 0], [500, 50, 150]], dtype=np.int64)
    m.pv = np.array([True, True])
    rt = [16384, 0, 0, 0, 16384, 0, 0, 0, 16384, 0, 0, 0]
    before = m.em.cnt.copy(), m.em.tag.copy(), m.em.stamp.copy(), m.cost.copy()
    m.ctl(6, 1, 0, 0)
    m.commit(rt, 0, 0, -600, 1200)
    assert m.seq == m.em.fc == 0
    for a, b in zip(before, [m.em.cnt, m.em.tag, m.em.stamp, m.cost]):
        np.testing.assert_array_equal(a, b)
    # A distinct marker proves FUSE_ONLY doesn't recapture structural points.
    m.spts = np.array([[123, 456, 789]], dtype=np.int64)
    m.ctl(6, 2, 0, 0)
    corrected = list(rt); corrected[9] = 100
    m.commit(corrected, 2, 0, -600, 1200)
    assert m.seq == m.em.fc == 1
    np.testing.assert_array_equal(m.spts, [[123, 456, 789]])
    # Control: directly fusing only the corrected pose must yield the same map.
    direct = FpgaModel(); direct.xyz = m.xyz; direct.pv = m.pv
    direct.commit(corrected, 2, 0, -600, 1200)
    for attr in ['cnt', 'tag', 'stamp', 'zmin', 'zmax']:
        np.testing.assert_array_equal(getattr(m.em, attr), getattr(direct.em, attr))
    np.testing.assert_array_equal(m.cost, direct.cost)
    uncorrected = FpgaModel(); uncorrected.xyz = m.xyz; uncorrected.pv = m.pv
    uncorrected.commit(rt, 0, 0, -600, 1200)
    assert not np.array_equal(m.em.cnt, uncorrected.em.cnt)
    result = {'pass': True, 'first_pass_writes': 0, 'fusion_count': m.em.fc,
              'map_generation': m.seq, 'corrected_projection_only': True}
    out = Path(__file__).resolve().parents[1] / 'build/nav_opt/staged_fusion_contract.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
