#! /usr/bin/env python
import unittest

from robot_skills.mockbot import Mockbot
from robot_skills.util.kdl_conversions import frame_stamped
from robot_smach_states.util.designators.core import VariableDesignator
from robot_smach_states.util.designators.arm import ArmDesignator

from robot_smach_states.util.designators.navigation.arm_constraints import ArmsreachConstraintsDesignator


class TestArmsreachConstraintDesignator(unittest.TestCase):
    def test_resolve(self):
        robot = Mockbot()
        arm = ArmDesignator(robot, {}, name="arm_designator")
        frame = VariableDesignator(frame_stamped("\map", 0, 0, 0), name="frame_designator")
        n = ArmsreachConstraintsDesignator(robot, frame, arm, name="constraint_designator")
        constr = n.resolve()
        self.assertIsNotNone(constr.pc)
        self.assertIsNotNone(constr.oc)

    def test_no_look(self):
        robot = Mockbot()
        arm = ArmDesignator(robot, {}, name="arm_designator")
        frame = VariableDesignator(frame_stamped("\map", 0, 0, 0), name="frame_designator")
        n = ArmsreachConstraintsDesignator(robot, frame, arm, look=False, name="constraint_designator")
        constr = n.resolve()
        self.assertIsNotNone(constr.pc)
        self.assertIsNone(constr.oc)


if __name__ == '__main__':
    unittest.main()
