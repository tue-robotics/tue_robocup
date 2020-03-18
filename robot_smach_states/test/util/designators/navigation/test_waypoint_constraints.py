#! /usr/bin/env python
import unittest

from robot_skills.util.entity import Entity
import PyKDL as kdl
from robot_smach_states.util.designators.core import VariableDesignator

from robot_smach_states.util.designators.navigation.waypoint_constraints import WayPointConstraintsDesignator


class TestLookAtConstraintDesignator(unittest.TestCase):
    def test_resolve(self):
        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(1, 2, 0.0))
        e = Entity("dummy", "dummy_type", "/map", pose, None, None, None, None)
        entity = VariableDesignator(e, name="entity designator")
        n = WayPointConstraintsDesignator(entity, 0.3, name="waypoint designator")
        constr = n.resolve()
        self.assertIsNotNone(constr.pc)
        self.assertIsNotNone(constr.oc)

    def test_no_look(self):
        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(1, 2, 0.0))
        e = Entity("dummy", "dummy_type", "/map", pose, None, None, None, None)
        entity = VariableDesignator(e, name="entity designator")
        n = WayPointConstraintsDesignator(entity, 0.3, look= False, name="waypoint designator")
        constr = n.resolve()
        self.assertIsNotNone(constr.pc)
        self.assertIsNone(constr.oc)


if __name__ == '__main__':
    unittest.main()
