#! /usr/bin/env python
import unittest

import rospy

import PyKDL as kdl

from ed.entity import Entity
from ed.shape import Shape

from robot_smach_states.util.designators.core import Designator
from robot_smach_states.navigation.constraint_functions.radius_constraints import radius_constraint
from .util import constraint_strings_equal


class TestRadiusConstraintFunction(unittest.TestCase):
    def test_no_shape(self):
        # parameters
        x_coordinate = 2.1
        y_coordinate = 3.7
        z_coordinate = 0
        radius = 0.7
        margin = 0.3
        frame_id = "map"

        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(x_coordinate, y_coordinate, z_coordinate))
        e = Entity("dummy", "dummy_type", frame_id, pose, Shape(), None, None, rospy.Time())
        entity = Designator(e, name="entity designator")

        pc, oc = radius_constraint(entity, radius, margin)

        # verify positionconstraint
        self.assertEqual(pc.frame, frame_id)

        ro = "(x-{})^2+(y-{})^2 < {}^2".format(x_coordinate, y_coordinate, radius + margin)
        ri = "(x-{})^2+(y-{})^2 > {}^2".format(x_coordinate, y_coordinate, radius - margin)
        verification_string = ri + " and " + ro
        equal, msg = constraint_strings_equal(pc.constraint, verification_string)
        self.assertTrue(equal, msg)

        # verify orientationconstraint
        self.assertIsNone(oc)


if __name__ == '__main__':
    unittest.main()
