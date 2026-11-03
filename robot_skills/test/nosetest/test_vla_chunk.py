import unittest

import numpy as np

from robot_skills.vla_chunk import ACTION_DIM, ChunkLimits, gripper_command, group_max_abs, integrate_arm, plan_chunk


class TestVLAChunk(unittest.TestCase):
    def setUp(self):
        self.limits = ChunkLimits(fps=10.0, arm_lower=(0.0, -1.0, -1.0, -1.0, -1.0), arm_upper=(0.5, 1.0, 1.0, 1.0, 1.0),
                                  arm_max_vel=(1.0, 1.0, 1.0, 1.0, 1.0), base_max_vel=(0.1, 0.1, 0.2))

    def test_integrate_arm_sums_deltas(self):
        deltas = np.full((3, 5), 0.01)
        positions = integrate_arm(deltas, [0.1, 0.0, 0.0, 0.0, 0.0], self.limits)
        np.testing.assert_allclose(positions[:, 0], [0.11, 0.12, 0.13])
        np.testing.assert_allclose(positions[:, 1], [0.01, 0.02, 0.03])

    def test_integrate_arm_clips_step_to_max_velocity(self):
        # max step is arm_max_vel / fps = 0.1
        deltas = np.array([[0.5, 0.0, 0.0, 0.0, 0.0]])
        positions = integrate_arm(deltas, [0.0] * 5, self.limits)
        self.assertAlmostEqual(positions[0, 0], 0.1)

    def test_integrate_arm_clips_to_joint_limits(self):
        deltas = np.full((10, 5), -0.1)
        positions = integrate_arm(deltas, [0.05, 0.0, 0.0, 0.0, 0.0], self.limits)
        self.assertAlmostEqual(positions[-1, 0], 0.0)
        self.assertAlmostEqual(positions[-1, 1], -1.0)

    def test_gripper_command_thresholds_and_hysteresis(self):
        self.assertEqual(gripper_command(1.2, self.limits), "open")
        self.assertEqual(gripper_command(-0.9, self.limits), "close")
        self.assertIsNone(gripper_command(0.0, self.limits))
        self.assertIsNone(gripper_command(1.2, self.limits, previous="open"))
        self.assertEqual(gripper_command(-0.9, self.limits, previous="open"), "close")

    def test_plan_chunk_prefix_and_base_clipping(self):
        chunk = np.zeros((50, ACTION_DIM))
        chunk[:, 8] = 1.0  # base_x far above the limit
        chunk[:, 5] = -1.0
        plan = plan_chunk(chunk, [0.0] * 5, self.limits, n_steps=15)
        self.assertEqual(plan.arm_positions.shape, (15, 5))
        self.assertEqual(plan.base_velocities.shape, (15, 3))
        self.assertAlmostEqual(plan.base_velocities[0, 0], 0.1)
        np.testing.assert_allclose(plan.times_from_start[:3], [0.1, 0.2, 0.3])
        self.assertEqual(plan.gripper[-1], -1.0)

    def test_plan_chunk_rejects_wrong_shape_and_nan(self):
        with self.assertRaises(ValueError):
            plan_chunk(np.zeros((50, 10)), [0.0] * 5, self.limits)
        bad = np.zeros((50, ACTION_DIM))
        bad[3, 2] = np.nan
        with self.assertRaises(ValueError):
            plan_chunk(bad, [0.0] * 5, self.limits)

    def test_group_max_abs(self):
        chunk = np.zeros((2, ACTION_DIM))
        chunk[1, 3] = -0.4
        chunk[0, 9] = 0.2
        groups = group_max_abs(chunk)
        self.assertAlmostEqual(groups["arm"], 0.4)
        self.assertAlmostEqual(groups["base"], 0.2)
        self.assertEqual(groups["gripper"], 0.0)


if __name__ == '__main__':
    unittest.main()
