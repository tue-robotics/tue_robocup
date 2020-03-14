#! /usr/bin/env python
import unittest

from robot_smach_states.util.designators.navigation.navigation import NavigationConstraintsDesignator, CompoundConstraintsDesignator, PoseConstraint
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint


class DummyConstraintDesignator(NavigationConstraintsDesignator):
    def _resolve(self):
        pc = PositionConstraint(frame="/map")
        oc = OrientationConstraint(frame="/map")
        return PoseConstraint(pc, oc)


class TestCompoundConstraintDesignator(unittest.TestCase):
    def test_empty(self):
        n = CompoundConstraintsDesignator()
        self.assertEqual(n.resolve(), None)

    def test_resolve_one(self):
        n = CompoundConstraintsDesignator()
        n1 = DummyConstraintDesignator()
        n.add(n1, "dummy_constraints")
        self.assertEqual(n.resolve(), n1.resolve())

    def test_resolve_multiple(self):
        n = CompoundConstraintsDesignator()
        n1 = DummyConstraintDesignator()
        n.add(n1, "dummy_constraint_1")
        n.add(n1, "dummy_constraint_2")
        pose = n.resolve()
        assert type(pose) is PoseConstraint
        assert type(pose.pc) is PositionConstraint
        assert type(pose.oc) is OrientationConstraint


if __name__ == '__main__':
    unittest.main()
