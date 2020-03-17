#! /usr/bin/env python
import unittest

from robot_skills.util.entity import Entity
import PyKDL as kdl
from robot_smach_states.util.designators.core import VariableDesignator

from robot_smach_states.util.designators.navigation.look_at_constraints import LookAtConstraintsDesignator


class TestLookAtConstraintDesignator(unittest.TestCase):
    def test_resolve(self):
        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(1, 2, 1.3))
        e = Entity("dummy", "dummy_type", "/map", pose, None, None, None, None)
        entity = VariableDesignator(e, name="entity designator")
        n = LookAtConstraintsDesignator(entity, name="look at designator")
        constr = n.resolve()
        self.assertIsNone(constr.pc)
        self.assertIsNotNone(constr.oc)

    def test_resolve_offset(self):
        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(1, 2, 1.3))
        e = Entity("dummy", "dummy_type", "/map", pose, None, None, None, None)
        entity = VariableDesignator(e, name="entity designator")
        n = LookAtConstraintsDesignator(entity, 1.57, name="look at designator")
        constr = n.resolve()
        self.assertIsNone(constr.pc)
        self.assertIsNotNone(constr.oc)


if __name__ == '__main__':
    unittest.main()
