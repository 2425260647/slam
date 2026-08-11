#!/usr/bin/env python3
import importlib.util
import math
import unittest
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).with_name("evaluate_m3dgr_trajectory.py")
SPEC = importlib.util.spec_from_file_location("m3dgr_eval", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Se2EvaluationTest(unittest.TestCase):
    def test_relative_error_is_zero_for_identical_trajectory(self):
        times = np.arange(0.0, 4.0, 0.5)
        positions = np.column_stack((times, np.zeros_like(times), np.zeros_like(times)))
        yaws = np.zeros_like(times)
        metrics = MODULE.relative_error_metrics(
            times, positions, yaws, positions, yaws, 1.0, 0.01)
        self.assertEqual(metrics["pair_count"], 6)
        self.assertAlmostEqual(metrics["se2_translation_meters"]["rmse"], 0.0)
        self.assertAlmostEqual(metrics["se2_rotation_radians"]["rmse"], 0.0)

    def test_rpe_uses_relative_frame_and_yaw_wrap(self):
        times = np.array([0.0, 1.0, 2.0])
        reference = np.column_stack((times, np.zeros(3), np.zeros(3)))
        estimate = reference.copy()
        reference_yaw = np.array([math.pi - 0.1, -math.pi + 0.1, -math.pi + 0.1])
        estimate_yaw = reference_yaw.copy()
        metrics = MODULE.relative_error_metrics(
            times, estimate, estimate_yaw, reference, reference_yaw, 1.0, 0.01)
        self.assertAlmostEqual(metrics["se2_translation_meters"]["rmse"], 0.0)
        self.assertAlmostEqual(metrics["se2_rotation_radians"]["rmse"], 0.0)

    def test_se2_alignment_does_not_use_z_or_scale(self):
        source = np.array([[0., 0., 4.], [1., 0., 8.], [2., 0., 12.]])
        target = np.array([[3., -2., 100.], [3., -1., 200.], [3., 0., 300.]])
        source_yaw = np.zeros(3)
        aligned, aligned_yaw, _, _, _ = MODULE.rigid_align_se2(
            source, source_yaw, target)
        np.testing.assert_allclose(aligned[:, :2], target[:, :2], atol=1e-12)
        np.testing.assert_allclose(aligned_yaw, np.full(3, math.pi / 2), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
