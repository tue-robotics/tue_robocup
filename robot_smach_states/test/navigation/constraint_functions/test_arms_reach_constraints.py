#! /usr/bin/env python
import unittest
import math

from robot_skills.mockbot import Mockbot
from robot_skills.util.kdl_conversions import frame_stamped
from robot_smach_states.util.designators.core import Designator
from robot_smach_states.util.designators.arm import ArmDesignator

from robot_smach_states.navigation.constraint_functions.arms_reach_constraints import arms_reach_constraint
from .util import constraint_strings_equal


class TestArmsReachConstraintFunction(unittest.TestCase):
    def test_base(self):
        # parameters
        x_coordinate = 2.1
        y_coordinate = 3.7
        z_coordinate = 0
        frame_id = "/map"

        robot = Mockbot()
        arm = ArmDesignator(robot, {}, name="arm_designator")
        frame = Designator(frame_stamped(frame_id, x_coordinate, y_coordinate, z_coordinate), name="frame_designator")

        pc, oc = arms_reach_constraint(frame, arm)

        # verify positionconstraint
        self.assertEqual(pc.frame, frame_id)

        publicarm = arm.resolve()
        radius = math.hypot(publicarm.base_offset.x(), publicarm.base_offset.y())
        margin = 0.075
        ro = "(x-{})^2+(y-{})^2 < {}^2".format(x_coordinate, y_coordinate, radius + margin)
        ri = "(x-{})^2+(y-{})^2 > {}^2".format(x_coordinate, y_coordinate, radius - margin)
        verification_string = ri + " and " + ro
        equal, msg = constraint_strings_equal(pc.constraint, verification_string)
        self.assertTrue(equal, msg)

        # verify orientationconstraint
        self.assertEqual(oc.look_at.x, x_coordinate)
        self.assertEqual(oc.look_at.y, y_coordinate)
        # The z coordinate is irrelevant in the orientation constraint
        self.assertEqual(oc.frame, frame_id)

    def test_no_look(self):
        # parameters
        x_coordinate = 2.1
        y_coordinate = 3.7
        z_coordinate = 0
        frame_id = "/map"

        robot = Mockbot()
        arm = ArmDesignator(robot, {}, name="arm_designator")
        frame = Designator(frame_stamped(frame_id, x_coordinate, y_coordinate, z_coordinate), name="frame_designator")

        pc, oc = arms_reach_constraint(frame, arm, look=False)

        # verify positionconstraint
        self.assertEqual(pc.frame, frame_id)

        publicarm = arm.resolve()
        radius = math.hypot(publicarm.base_offset.x(), publicarm.base_offset.y())
        margin = 0.075
        ro = "(x-{})^2+(y-{})^2 < {}^2".format(x_coordinate, y_coordinate, radius + margin)
        ri = "(x-{})^2+(y-{})^2 > {}^2".format(x_coordinate, y_coordinate, radius - margin)
        verification_string = ri + " and " + ro
        equal, msg = constraint_strings_equal(pc.constraint, verification_string)
        self.assertTrue(equal, msg)

        # verify orientationconstraint
        self.assertIsNone(oc)


if __name__ == '__main__':
    unittest.main()
