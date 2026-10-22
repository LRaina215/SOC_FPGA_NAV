"""Counterexamples for scoring, independent of firmware decision logic."""
import unittest
import numpy as np

from evaluation import TerminalCriteria, TerminalJudge, TerrainCounts, tracking_completion_candidate


class TerminalTests(unittest.TestCase):
    def test_tracking_rotation_is_not_completion(self):
        self.assertFalse(tracking_completion_candidate(True, 2, (0, 0, 300), 0.8, 1.0))
        self.assertTrue(tracking_completion_candidate(True, 2, (0, 0, 0), 0.8, 1.0))
        self.assertFalse(tracking_completion_candidate(True, 3, (0, 0, 0), 0.8, 1.0))
        self.assertFalse(tracking_completion_candidate(True, 2, (0, 0, 0), 1.2, 1.0))

    def run_trace(self, position, speed=0.0, yaw_rate=0.0):
        judge = TerminalJudge((0, 0))
        for i in range(21):
            t = i / 10
            x = position(t) if callable(position) else position
            judge.observe(t, x, 0, speed, yaw_rate, declared=i == 0)
        return judge

    def test_false_arrival_at_legacy_near_goal_distance(self):
        j = self.run_trace(0.55)
        self.assertEqual(j.outcome, "false_arrival")
        self.assertFalse(j.success)

    def test_small_error_does_not_prove_stop(self):
        self.assertEqual(self.run_trace(0.1, speed=0.1).outcome, "not_stopped")
        self.assertEqual(self.run_trace(0.1, yaw_rate=0.2).outcome, "not_stopped")

    def test_arrival_requires_full_observation(self):
        j = TerminalJudge((0, 0))
        for i in range(20):
            j.observe(i / 10, 0.1, 0, 0, 0, declared=i == 0)
        self.assertFalse(j.finished)
        j.observe(2.0, 0.1, 0, 0, 0)
        self.assertTrue(j.success)

    def test_terminal_drift_even_if_it_returns(self):
        j = self.run_trace(lambda t: 0.2 if 0.5 < t < 1 else 0.1)
        self.assertEqual(j.outcome, "terminal_drift")

    def test_last_instant_stop_is_not_a_dwell(self):
        j = TerminalJudge((0, 0))
        for i in range(21):
            j.observe(i / 10, 0.1, 0, 0.1 if i < 19 else 0, 0, declared=i == 0)
        self.assertEqual(j.outcome, "not_stopped")

    def test_observation_gap_does_not_count_as_stop(self):
        j = TerminalJudge((0, 0))
        j.observe(0, 0.1, 0, 0, 0, declared=True)
        j.observe(2, 0.1, 0, 0, 0)
        self.assertEqual(j.outcome, "not_stopped")

    def test_no_declaration_is_timeout(self):
        j = TerminalJudge((0, 0))
        j.observe(5, 0, 0, 0, 0)
        self.assertFalse(j.success)
        self.assertEqual(j.outcome, "timeout")

    def test_invalid_criteria_rejected(self):
        with self.assertRaises(ValueError):
            TerminalCriteria(position_m=float("nan"))
        with self.assertRaises(ValueError):
            TerminalCriteria(observe_s=0.1)


class TerrainTests(unittest.TestCase):
    def test_inflation_cannot_hide_raw_miss(self):
        c = TerrainCounts()
        c.add(np.array([1]), np.array([253]), np.array([False]), np.array([True]))
        r = c.report()
        self.assertEqual(r["raw_recall_observed"], 0)
        self.assertEqual(r["post_block_recall_known"], 1)
        self.assertEqual(r["counts"]["inflation_masks_raw_miss"], 1)

    def test_unknown_and_filled_unknown_are_explicit(self):
        c = TerrainCounts()
        c.add(np.array([0, 1, 1]), np.array([255, 255, 80]),
              np.array([False] * 3), np.array([False] * 3))
        r = c.report()
        self.assertIsNone(r["raw_recall_observed"])
        self.assertEqual(r["unknown_hazard_fraction"], 1)
        self.assertEqual(r["counts"]["filled_hazard"], 1)
        self.assertEqual(r["counts"]["map_unknown_hazard"], 1)

    def test_boundary_not_silently_discarded(self):
        c = TerrainCounts()
        c.add(np.array([0, 2, 1]), np.array([253, 254, 254]),
              np.array([False, True, True]), np.array([True] * 3))
        r = c.report()
        self.assertEqual(r["counts"]["boundary"], 1)
        self.assertEqual(r["counts"]["boundary_raw_lethal"], 1)
        self.assertEqual(r["counts"]["inflation_free"], 1)
        self.assertEqual(r["raw_false_positive_observed"], 0)
        self.assertEqual(r["post_block_false_positive_known"], 1)


if __name__ == "__main__":
    unittest.main()
