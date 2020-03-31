#! /usr/bin/env python
import unittest

from robot_skills.util.entity import Entity
import PyKDL as kdl
from robot_smach_states.util.designators.core import VariableDesignator

from robot_smach_states.navigation.constraint_functions.waypoint_constraints import waypoint_constraint


class TestWaypointConstraintFunction(unittest.TestCase):
    def test_base(self):
        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(1, 2, 0.0))
        e = Entity("dummy", "dummy_type", "/map", pose, None, None, None, None)
        entity = VariableDesignator(e, name="entity designator")

        pc, oc = waypoint_constraint(entity, 0.3)

        self.assertIsNotNone(pc)
        self.assertIsNotNone(oc)

    def test_no_look(self):
        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(1, 2, 0.0))
        e = Entity("dummy", "dummy_type", "/map", pose, None, None, None, None)
        entity = VariableDesignator(e, name="entity designator")
        pc, oc = waypoint_constraint(entity, 0.3, look=False)

        self.assertIsNotNone(pc)
        self.assertIsNone(oc)


if __name__ == '__main__':
    unittest.main()
