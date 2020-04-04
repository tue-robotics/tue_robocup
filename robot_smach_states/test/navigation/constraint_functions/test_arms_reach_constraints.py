#! /usr/bin/env python
import unittest

from robot_skills.mockbot import Mockbot
from robot_skills.util.kdl_conversions import frame_stamped
from robot_smach_states.util.designators.core import Designator
from robot_smach_states.util.designators.arm import ArmDesignator

from robot_smach_states.navigation.constraint_functions.arms_reach_constraints import arms_reach_constraint


class TestArmsReachConstraintFunction(unittest.TestCase):
    def test_base(self):
        robot = Mockbot()
        arm = ArmDesignator(robot, {}, name="arm_designator")
        frame = Designator(frame_stamped("/map", 0, 0, 0), name="frame_designator")

        pc, oc = arms_reach_constraint(frame, arm)

        self.assertIsNotNone(pc)
        self.assertIsNotNone(oc)

    def test_no_look(self):
        robot = Mockbot()
        arm = ArmDesignator(robot, {}, name="arm_designator")
        frame = Designator(frame_stamped("/map", 0, 0, 0), name="frame_designator")

        pc, oc = arms_reach_constraint(frame, arm, look=False)

        self.assertIsNotNone(pc)
        self.assertIsNone(oc)


if __name__ == '__main__':
    unittest.main()
