#! /usr/bin/env python
import unittest
from numpy import linspace

from robot_smach_states.navigation.constraint_functions.pose_constraints import pose_constraints


class TestPoseConstraintFunction(unittest.TestCase):
    def test_no_orientation(self):
        # parameters
        x_coordinate = 2.1
        y_coordinate = 3.7
        radius = 0.15
        frame_id = "/map"

        pc, oc = pose_constraints(x_coordinate, y_coordinate, rz=None, radius=radius, frame_id=frame_id)

        self.assertIsNotNone(pc)
        self.assertIsNone(oc)

        # verify positionconstraint
        constraint_string = pc.constraint
        constraint_string = constraint_string.replace("^", "**")
        self.assertEqual(pc.frame, frame_id)

        verification_string = "(x-{})**2+(y-{})**2 < {}**2".format(x_coordinate, y_coordinate, radius)
        for x in linspace(-5, 5, 20):
            for y in linspace(-5, 5, 20):
                self.assertEqual(eval(constraint_string), eval(verification_string))

        # verify orientationconstraint
        self.assertIsNone(oc)

    def test_with_orientation(self):
        # parameters
        x_coordinate = 2.1
        y_coordinate = 3.7
        rz_orientation = 1.57
        radius = 0.15
        frame_id = "/map"

        pc, oc = pose_constraints(x_coordinate, y_coordinate, rz=rz_orientation, radius=radius, frame_id=frame_id)

        # verify positionconstraint
        constraint_string = pc.constraint
        constraint_string = constraint_string.replace("^", "**")
        self.assertEqual(pc.frame, frame_id)

        verification_string = "(x-{})**2+(y-{})**2 < {}**2".format(x_coordinate, y_coordinate, radius)
        for x in linspace(-5, 5, 20):
            for y in linspace(-5, 5, 20):
                self.assertEqual(eval(constraint_string), eval(verification_string))

        # verify orientationconstraint
        self.assertEqual(oc.angle_offset, rz_orientation)
        self.assertEqual(oc.frame, frame_id)


if __name__ == '__main__':
    unittest.main()
