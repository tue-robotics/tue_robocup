#! /usr/bin/env python
import unittest

from robot_skills.mockbot import Mockbot
from robot_skills.util.entity import Entity
from robot_smach_states.util.designators.core import Designator

from robot_smach_states.navigation.constraint_functions.symbolic_constraints import symbolic_constraint, room_constraint


class TestSymbolicConstraintFunction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def test_base(self):
        dummy_id = "dummy"
        e = Entity(dummy_id, "dummy_type", "/map", None, None, None, None, None)
        entity = Designator(e, name="entity designator")
        area_name = "area"

        pc, oc = symbolic_constraint(self.robot, {entity: area_name})

        # symbolic constraint should only generate a positionconstraint
        self.assertIsNotNone(pc)
        self.assertIsNone(oc)
        self.robot.parts["ed"].navigation.get_position_constraint.assert_called_with({dummy_id: area_name})

    def test_base_multiple_entries(self):
        dummy_id_1 = "dummy1"
        e1 = Entity(dummy_id_1, "dummy_type", "/map", None, None, None, None, None)
        entity1 = Designator(e1, name="entity designator")
        area_name1 = "area1"

        dummy_id_2 = "dummy2"
        e2 = Entity(dummy_id_2, "dummy_type", "/map", None, None, None, None, None)
        entity2 = Designator(e2, name="entity designator")
        area_name2 = "area2"

        pc, oc = symbolic_constraint(self.robot, {entity1: area_name1, entity2: area_name2})

        # symbolic constraint should only generate a positionconstraint
        self.assertIsNotNone(pc)
        self.assertIsNone(oc)
        self.robot.parts["ed"].navigation.get_position_constraint.assert_called_with({dummy_id_1: area_name1})
        self.robot.parts["ed"].navigation.get_position_constraint.assert_called_with({dummy_id_2: area_name2})

    def test_room(self):
        dummy_id = "dummy_room"
        r = Entity(dummy_id, "dummy_type", "/map", None, None, None, None, None)
        room = Designator(r, name="room designator")

        pc, oc = room_constraint(self.robot, room)

        # room constraint should only generate a positionconstraint
        self.assertIsNotNone(pc)
        self.assertIsNone(oc)
        self.robot.parts["ed"].navigation.get_position_constraint.assert_called_with({dummy_id: "in"})


if __name__ == '__main__':
    unittest.main()
