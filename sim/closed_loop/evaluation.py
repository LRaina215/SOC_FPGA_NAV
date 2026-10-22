"""Independent offline scoring; never used as navigation input.

Terrain counts are cell-observations, not unique world cells. Geometry labels
still depend on the declared vehicle capability assumptions in world.py.
"""
from dataclasses import asdict, dataclass
import math

import numpy as np

EVALUATION_VERSION = "nav-opt-v2"


def tracking_completion_candidate(found, mode, cmd, goal_error, position_limit):
    """Tracking has no ARRIVED state; require intent to stop both motion axes.

    Zero linear speed during target-facing rotation is not a completion claim.
    Real speed and angular rate are still checked independently afterwards.
    """
    return bool(found and mode == 2 and abs(cmd[0]) < 10 and cmd[2] == 0 and goal_error <= position_limit)


@dataclass(frozen=True)
class TerminalCriteria:
    position_m: float = 0.150
    speed_mps: float = 0.020
    yaw_rate_rps: float = 0.050
    dwell_s: float = 0.500
    observe_s: float = 2.000

    def __post_init__(self):
        vals = asdict(self).values()
        if not all(math.isfinite(x) and x > 0 for x in vals):
            raise ValueError("Terminal criteria must be finite and positive")
        if self.observe_s < self.dwell_s:
            raise ValueError("Observation window must cover stop dwell")


class TerminalJudge:
    def __init__(self, goal, criteria=None):
        self.goal = goal
        self.criteria = criteria or TerminalCriteria()
        self.declared_s = None
        self.declared_error_m = None
        self.final_error_m = None
        self.max_error_m = 0.0
        self.max_drift_m = 0.0
        self._declared_xy = None
        self._stable_since = None
        self._last_t = None
        self.finished = False
        self.success = False
        self.outcome = "timeout"

    def observe(self, t, x, y, speed, yaw_rate, declared=False):
        if self.finished or self.goal is None:
            return
        err = math.hypot(x - self.goal[0], y - self.goal[1])
        self.final_error_m = err
        if self.declared_s is None:
            if not declared:
                return
            self.declared_s = t
            self.declared_error_m = err
            self._declared_xy = (x, y)
        c = self.criteria
        self.max_error_m = max(self.max_error_m, err)
        self.max_drift_m = max(self.max_drift_m,
                              math.hypot(x - self._declared_xy[0], y - self._declared_xy[1]))
        stable = err <= c.position_m and abs(speed) <= c.speed_mps and abs(yaw_rate) <= c.yaw_rate_rps
        # Samples in run_cl are 100 ms apart. A gap is not evidence of a dwell.
        if self._last_t is not None and t - self._last_t > 0.150 + 1e-9:
            self._stable_since = None
        self._last_t = t
        if not stable:
            self._stable_since = None
        elif self._stable_since is None:
            self._stable_since = t
        if t - self.declared_s + 1e-9 < c.observe_s:
            return
        self.finished = True
        if self.max_error_m > c.position_m:
            self.outcome = "false_arrival" if self.declared_error_m > c.position_m else "terminal_drift"
        elif self._stable_since is None or t - self._stable_since + 1e-9 < c.dwell_s:
            self.outcome = "not_stopped"
        else:
            self.success = True
            self.outcome = "arrived"

    def report(self):
        return dict(criteria=asdict(self.criteria), declared_s=self.declared_s,
                    declared_error_m=self.declared_error_m, final_error_m=self.final_error_m,
                    max_error_m=self.max_error_m if self.declared_s is not None else None,
                    max_drift_m=self.max_drift_m if self.declared_s is not None else None,
                    observation_complete=self.finished, valid=self.success, outcome=self.outcome)


class TerrainCounts:
    def __init__(self):
        self.counts = {}

    def add(self, labels, cost, raw_lethal, observed, in_bounds=None):
        lab, cw = np.asarray(labels), np.asarray(cost)
        raw, obs = np.asarray(raw_lethal, dtype=bool), np.asarray(observed, dtype=bool)
        if any(x.shape != lab.shape for x in (cw, raw, obs)):
            raise ValueError("Terrain arrays must have the same shape")
        scope = np.ones(lab.shape, dtype=bool) if in_bounds is None else np.asarray(in_bounds, dtype=bool)
        if scope.shape != lab.shape:
            raise ValueError("Bounds mask shape mismatch")
        use = scope & (lab != 2)
        free, hazard = use & (lab == 0), use & (lab == 1)
        known = cw != 255
        masks = dict(total=scope, boundary=scope & (lab == 2), free=free, hazard=hazard,
                     unknown_free=free & ~obs, unknown_hazard=hazard & ~obs,
                     map_unknown_free=free & ~known, map_unknown_hazard=hazard & ~known,
                     raw_tp=hazard & obs & raw, raw_fn=hazard & obs & ~raw,
                     raw_fp=free & obs & raw, raw_tn=free & obs & ~raw,
                     post_tp=hazard & known & (cw >= 253),
                     post_fn=hazard & known & (cw < 253),
                     post_fp=free & known & (cw >= 253),
                     post_tn=free & known & (cw < 253),
                     inflation_free=free & (cw == 253),
                     inflation_masks_raw_miss=hazard & obs & ~raw & (cw == 253),
                     filled_free=free & ~obs & known, filled_hazard=hazard & ~obs & known,
                     boundary_observed=scope & (lab == 2) & obs,
                     boundary_raw_lethal=scope & (lab == 2) & raw,
                     boundary_blocked=scope & (lab == 2) & known & (cw >= 253))
        for key, mask in masks.items():
            self.counts[key] = self.counts.get(key, 0) + int(mask.sum())

    def report(self):
        c = self.counts.copy()

        def ratio(a, *denom):
            d = sum(c.get(k, 0) for k in denom)
            return c.get(a, 0) / d if d else None

        return dict(unit="cell_observations", counts=c,
                    raw_recall_observed=ratio("raw_tp", "raw_tp", "raw_fn"),
                    raw_false_positive_observed=ratio("raw_fp", "raw_fp", "raw_tn"),
                    post_block_recall_known=ratio("post_tp", "post_tp", "post_fn"),
                    post_block_false_positive_known=ratio("post_fp", "post_fp", "post_tn"),
                    unknown_free_fraction=ratio("unknown_free", "free"),
                    unknown_hazard_fraction=ratio("unknown_hazard", "hazard"),
                    map_unknown_free_fraction=ratio("map_unknown_free", "free"),
                    map_unknown_hazard_fraction=ratio("map_unknown_hazard", "hazard"))
