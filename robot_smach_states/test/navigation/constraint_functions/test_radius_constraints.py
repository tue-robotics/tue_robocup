#! /usr/bin/env python
import unittest

from robot_skills.util.entity import Entity
import PyKDL as kdl
from robot_smach_states.util.designators.core import Designator

from robot_smach_states.navigation.constraint_functions.radius_constraints import radius_constraint


class TestRadiusConstraintFunction(unittest.TestCase):
    def test_no_shape(self):
        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(1, 2, 0.0))
        e = Entity("dummy", "dummy_type", "/map", pose, None, None, None, None)
        entity = Designator(e, name="entity designator")

        pc, oc = radius_constraint(entity, 0.7, 0.3)

        self.assertIsNotNone(pc)
        self.assertIsNone(oc)


if __name__ == '__main__':
    unittest.main()
