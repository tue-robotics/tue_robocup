#! /usr/bin/env python
import unittest

from robot_skills.mockbot import Mockbot
from robot_skills.util.entity import Entity
from robot_smach_states.util.designators.core import Designator

from robot_smach_states.navigation.constraint_functions.symbolic_constraints import symbolic_constraint


class TestArmsReachConstraintFunction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def test_base(self):
        dummy_id = "dummy"
        e = Entity(dummy_id, "dummy_type", "/map", None, None, None, None, None)
        entity = Designator(e, name="entity designator")
        area_name = "area"

        pc, oc = symbolic_constraint(self.robot, {entity: area_name})

        self.assertIsNotNone(pc)
        self.assertIsNone(oc)
        self.robot.parts["ed"].navigation.get_position_constraint.assert_called_once_with({dummy_id: area_name})


if __name__ == '__main__':
    unittest.main()
