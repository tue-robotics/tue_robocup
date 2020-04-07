#! /usr/bin/env python
import unittest
from numpy import linspace

from robot_skills.util.entity import Entity
import PyKDL as kdl
from robot_smach_states.util.designators.core import Designator

from robot_smach_states.navigation.constraint_functions.waypoint_constraints import waypoint_constraint


class TestWaypointConstraintFunction(unittest.TestCase):
    def test_base(self):
        # parameters
        x_coordinate = 2.1
        y_coordinate = 3.7
        z_coordinate = 0
        radius = 0.3
        yaw = 1.57
        frame_id = "/map"

        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, yaw), kdl.Vector(x_coordinate, y_coordinate, z_coordinate))
        e = Entity("dummy", "dummy_type", frame_id, pose, None, None, None, None)
        entity = Designator(e, name="entity designator")

        pc, oc = waypoint_constraint(entity, radius)

        # verify positionconstraint
        constraint_string = pc.constraint
        constraint_string = constraint_string.replace("^", "**")
        self.assertEqual(pc.frame, frame_id)

        verification_string = "(x-{})**2+(y-{})**2 < {}**2".format(x_coordinate, y_coordinate, radius)
        for x in linspace(-5, 5, 20):
            for y in linspace(-5, 5, 20):
                self.assertEqual(eval(constraint_string), eval(verification_string))

        # verify orientationconstraint
        self.assertEqual(oc.angle_offset, yaw)
        self.assertEqual(oc.frame, frame_id)

    def test_no_look(self):
        # parameters
        x_coordinate = 2.1
        y_coordinate = 3.7
        z_coordinate = 0
        radius = 0.3
        yaw = 1.57
        frame_id = "/map"

        pose = kdl.Frame(kdl.Rotation.RPY(0, 0, yaw), kdl.Vector(x_coordinate, y_coordinate, z_coordinate))
        e = Entity("dummy", "dummy_type", frame_id, pose, None, None, None, None)
        entity = Designator(e, name="entity designator")

        pc, oc = waypoint_constraint(entity, radius, look=False)

        # verify positionconstraint
        constraint_string = pc.constraint
        constraint_string = constraint_string.replace("^", "**")
        self.assertEqual(pc.frame, frame_id)

        verification_string = "(x-{})**2+(y-{})**2 < {}**2".format(x_coordinate, y_coordinate, radius)
        for x in linspace(-5, 5, 20):
            for y in linspace(-5, 5, 20):
                self.assertEqual(eval(constraint_string), eval(verification_string))

        # verify orientationconstraint
        self.assertIsNone(oc)


if __name__ == '__main__':
    unittest.main()
