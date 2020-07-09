#! /usr/bin/env python
import unittest

# ROS
from geometry_msgs.msg import Point
from robot_smach_states.navigation.constraint_functions.compound_constraints import combine_constraints
from cb_base_navigation_msgs.msg import OrientationConstraint, PositionConstraint


def dummy_constraint(name="test"):
    pc = PositionConstraint(constraint="constraint_"+name, frame="/map")
    oc = OrientationConstraint(look_at=Point(0, 0, 0), frame="/map")
    return pc, oc


class TestCombineConstraintsFunction(unittest.TestCase):
    def test_empty(self):
        res = combine_constraints([])
        self.assertEqual(res, None)

    def test_one(self):
        pc, oc = combine_constraints([dummy_constraint])
        pcref, ocref = dummy_constraint()

        self.assertEqual(pc, pcref)
        self.assertEqual(oc, ocref)

    def test_multiple(self):
        pc, oc = combine_constraints([lambda: dummy_constraint("1"),
                                      lambda: dummy_constraint("2"),
                                      lambda: dummy_constraint("3")])
        pc1, oc1 = dummy_constraint("1")
        pc2, oc2 = dummy_constraint("2")
        pc3, oc3 = dummy_constraint("3")

        self.assertIn(pc1.constraint, pc.constraint)
        self.assertIn(pc2.constraint, pc.constraint)
        self.assertIn(pc3.constraint, pc.constraint)
        self.assertEqual(pc.constraint.count("and"), 2)

        # only one orientation constraint is allowed under current system.
        self.assertTrue(oc == oc1 or oc == oc2 or oc == oc3)


if __name__ == '__main__':
    unittest.main()
