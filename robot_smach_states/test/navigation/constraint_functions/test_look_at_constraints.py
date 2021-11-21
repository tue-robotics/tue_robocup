#! /usr/bin/env python
import unittest

import rospy

import PyKDL as kdl

from ed.entity import Entity

from robot_smach_states.util.designators.core import Designator
from robot_smach_states.navigation.constraint_functions.look_at_constraints import look_at_constraint


class TestLookAtConstraintFunction(unittest.TestCase):
    def test_base(self):
        # parameters
        x_coordinate = 2.1
        y_coordinate = 3.7
        z_coordinate = 0
        frame_id = "map"

        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(x_coordinate, y_coordinate, z_coordinate))
        e = Entity("dummy", "dummy_type", frame_id, pose, None, None, None, rospy.Time())
        entity = Designator(e, name="entity designator")

        pc, oc = look_at_constraint(entity)

        # verify positionconstraint
        self.assertIsNone(pc)

        # verify orientationconstraint
        self.assertEqual(oc.look_at.x, x_coordinate)
        self.assertEqual(oc.look_at.y, y_coordinate)
        # The z coordinate is irrelevant in the orientation constraint
        self.assertEqual(oc.angle_offset, 0)
        self.assertEqual(oc.frame, frame_id)

    def test_offset(self):
        # parameters
        x_coordinate = 2.1
        y_coordinate = 3.7
        z_coordinate = 0
        frame_id = "map"
        offset = 1.57

        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(x_coordinate, y_coordinate, z_coordinate))
        e = Entity("dummy", "dummy_type", frame_id, pose, None, None, None, rospy.Time())
        entity = Designator(e, name="entity designator")

        pc, oc = look_at_constraint(entity, offset)

        # verify positionconstraint
        self.assertIsNone(pc)

        # verify orientationconstraint
        self.assertEqual(oc.look_at.x, x_coordinate)
        self.assertEqual(oc.look_at.y, y_coordinate)
        # The z coordinate is irrelevant in the orientation constraint
        self.assertEqual(oc.angle_offset, offset)
        self.assertEqual(oc.frame, frame_id)


if __name__ == '__main__':
    unittest.main()
