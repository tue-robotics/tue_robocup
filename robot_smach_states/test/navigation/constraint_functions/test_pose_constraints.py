#! /usr/bin/env python
import unittest

from robot_smach_states.navigation.constraint_functions.pose_constraints import pose_constraints


class TestPoseConstraintFunction(unittest.TestCase):
    def test_no_orientation(self):
        # parameters
        x_coordinate = 2.1
        y_coordinate = 3.7
        frame_id = "/map"

        pc, oc = pose_constraints(x_coordinate, y_coordinate, rz=None, radius=0.15, frame_id=frame_id)

        self.assertIsNotNone(pc)
        self.assertIsNone(oc)

        # verify positionconstraint
        self.assertIn("x", pc.constraint)
        self.assertIn("y", pc.constraint)
        self.assertEqual(pc.frame, frame_id)

        # verify orientationconstraint
        self.assertIsNone(oc)

    def test_orientation(self):
        # parameters
        x_coordinate = 2.1
        y_coordinate = 3.7
        rz_orientation = 1.57
        frame_id = "/map"

        pc, oc = pose_constraints(x_coordinate, y_coordinate, rz=rz_orientation, radius=0.15, frame_id=frame_id)

        # verify positionconstraint
        self.assertIn("x", pc.constraint)
        self.assertIn("y", pc.constraint)
        self.assertEqual(pc.frame, frame_id)

        # verify orientationconstraint
        self.assertEqual(oc.angle_offset, rz_orientation)
        self.assertEqual(oc.frame, frame_id)


if __name__ == '__main__':
    unittest.main()
