#! /usr/bin/env python
import unittest

from robot_skills.mockbot import Mockbot
from robot_skills.util.kdl_conversions import frame_stamped
from robot_smach_states.util.designators.core import Designator
from robot_smach_states.util.designators.arm import ArmDesignator

from robot_smach_states.navigation.constraint_functions.arms_reach_constraints import arms_reach_constraint


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
        self.assertIn("x", pc.constraint)
        self.assertIn("y", pc.constraint)
        self.assertEqual(pc.frame, frame_id)

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
        self.assertIn("x", pc.constraint)
        self.assertIn("y", pc.constraint)
        self.assertEqual(pc.frame, frame_id)

        # verify orientationconstraint
        self.assertIsNone(oc)


if __name__ == '__main__':
    unittest.main()
