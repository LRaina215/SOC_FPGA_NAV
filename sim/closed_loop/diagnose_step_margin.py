"""Audit the declared 20mm capability against the current geometric classifier.

Fully observed, noiseless positive steps; this does not measure real capability.
It records known failures without turning the large-step contract into a pass.
"""
import json
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'model'))
import nav_fixed as nf
import trav_v3


def main():
    idx = np.arange(128) - 64
    x, y = np.meshgrid(idx, idx)
    slot = ((y & 127) << 7) | (x & 127)
    tag = (((y >> 7) & 15) << 4) | ((x >> 7) & 15)
    res = 65536 / 1311
    rows = []
    for height in (15, 20, 21, 25, 30, 31, 75):
        em = nf.ElevMap()
        h = np.where((x + 1) * res > 500, height, 0)
        em.zmin[slot] = em.zmax[slot] = h
        em.cnt[slot] = 1; em.tag[slot] = tag
        _, lethal, _ = trav_v3.trav_v3(em, 0, 0, dict(infl_r=4, step_high_only=1))
        detected = bool(lethal.any())
        rows.append(dict(height_mm=height, model_capability_exceeded=height > 20,
                         raw_hazard_detected=detected,
                         missed_transition=height > 20 and not detected))
    report = dict(scope='Fully observed noiseless positive half-plane; declared model capability20mm, not real-car evidence.',
                  classifier_step_max_mm=trav_v3.DEFAULT_V3['step_max'], results=rows,
                  known_missed_heights_mm=[r['height_mm'] for r in rows if r['missed_transition']])
    out = Path(__file__).resolve().parents[1] / 'build/nav_opt/step_capability_gap_audit.json'
    out.write_text(json.dumps(report, indent=2)); print(json.dumps(report))


if __name__ == '__main__': main()
