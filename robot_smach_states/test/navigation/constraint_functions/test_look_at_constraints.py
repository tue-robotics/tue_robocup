#! /usr/bin/env python
import unittest

from robot_skills.util.entity import Entity
import PyKDL as kdl
from robot_smach_states.util.designators.core import VariableDesignator

from robot_smach_states.navigation.constraint_functions.look_at_constraints import look_at_constraint


class TestLookAtConstraintFunction(unittest.TestCase):
    def test_base(self):
        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(1, 2, 1.3))
        e = Entity("dummy", "dummy_type", "/map", pose, None, None, None, None)
        entity = VariableDesignator(e, name="entity designator")

        pc, oc = look_at_constraint(entity)

        self.assertIsNone(pc)
        self.assertIsNotNone(oc)

    def test_offset(self):
        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(1, 2, 1.3))
        e = Entity("dummy", "dummy_type", "/map", pose, None, None, None, None)
        entity = VariableDesignator(e, name="entity designator")

        pc, oc = look_at_constraint(entity, 1.57)

        self.assertIsNone(pc)
        self.assertIsNotNone(oc)


if __name__ == '__main__':
    unittest.main()
