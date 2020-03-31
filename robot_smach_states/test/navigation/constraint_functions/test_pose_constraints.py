#! /usr/bin/env python
import unittest

from robot_smach_states.navigation.constraint_functions.pose_constraints import pose_constraints


class TestPoseConstraintFunction(unittest.TestCase):
    def test_no_orientation(self):
        pc, oc = pose_constraints(0.5, 1.3, rz=None, radius=0.15, frame_id="/map")

        self.assertIsNotNone(pc)
        self.assertIsNone(oc)

    def test_orientation(self):
        pc, oc = pose_constraints(0.5, 1.3, rz=1.57, radius=0.15, frame_id="/map")

        self.assertIsNotNone(pc)
        self.assertIsNotNone(oc)


if __name__ == '__main__':
    unittest.main()
