# System
import math
import unittest

# ROS
import PyKDL as kdl

# MoveToGrasp
# noinspection PyProtectedMember
from robot_smach_states.manipulation.move_to_grasp import _point_between_points_at_distance, _create_straight_line_plan
from robot_smach_states.manipulation.move_to_grasp import MoveToGrasp


class TestMoveToGraspPose(unittest.TestCase):
    def test_point_between_points_at_distance(self):
        """
        Tests if the point between points at distance behaves as expected
        """

        p0 = kdl.Vector()
        distance = 0.25
        for x1, y1 in zip([1, 1, 0, -1, -1, -1, 0, 1],
                          [0, 1, 1, 1, 0, -1, -1, -1]):
            p1 = kdl.Vector(x1, y1, 0)
            result = _point_between_points_at_distance(p0, p1, distance)
            self.assertAlmostEqual(kdl.diff(p0, result).Norm(), distance)
            if p1.x() == 0.0:
                self.assertAlmostEqual(result.x(), 0.0)
            else:
                self.assertGreater(result.x() * p1.x(), 0.0)
            if p1.y() == 0.0:
                self.assertAlmostEqual(result.y(), 0.0)
            else:
                self.assertGreater(result.y() * p1.y(), 0.0)

    def test_goal_position_and_orientation(self):
        """
        Test the desired robot position and orientation for four pre-defined entity positions
        """
        robot_position = kdl.Vector()
        radius = 0.25
        angle_offset = 0.0
        entity_positions = [
            kdl.Vector(2.0, 0.0, 0.0),
            kdl.Vector(0.0, 2.0, 0.0),
            kdl.Vector(-2.0, 0.0, 0.0),
            kdl.Vector(0.0, -2.0, 0.0),
        ]
        expected_positions = [
            kdl.Vector(1.75, 0.0, 0.0),
            kdl.Vector(0.0, 1.75, 0.0),
            kdl.Vector(-1.75, 0.0, 0.0),
            kdl.Vector(0.0, -1.75, 0.0),
        ]
        expected_orientations = [
            kdl.Rotation.RPY(0.0, 0.0, 0.0),
            kdl.Rotation.RPY(0.0, 0.0, 0.5 * math.pi),
            kdl.Rotation.RPY(0.0, 0.0, math.pi),
            kdl.Rotation.RPY(0.0, 0.0, 1.5 * math.pi),
        ]
        for entity_pos, expected_goal_pos, expected_goal_orientation in zip(
          entity_positions,
          expected_positions,
          expected_orientations):
            goal_pos = MoveToGrasp.compute_goal_position(robot_position, entity_pos, radius)
            self.assertAlmostEqual(
                (expected_goal_pos - goal_pos).Norm(),
                0.0,
                msg="Goal position {} does not correspond to expected goal position {}".format(
                    goal_pos, expected_goal_pos)
            )
            goal_orientation = MoveToGrasp.compute_goal_orientation(robot_position, entity_pos, goal_pos, angle_offset)
            self.assertAlmostEqual(
                kdl.diff(expected_goal_orientation, goal_orientation).Norm(),
                0.0,
                msg="Goal orientation {} does not correspond to expected goal orientation {}".format(
                    goal_orientation, expected_goal_orientation)
            )

    def test_closer_than_radius(self):
        """
        Tests what happens if the robot is closer to the entity than the desired entity
        """
        robot_position = kdl.Vector()
        radius = 0.25
        angle_offset = 0.0
        entity_positions = [
            kdl.Vector(0.2, 0.0, 0.0),
            kdl.Vector(0.0, 0.2, 0.0),
            kdl.Vector(-0.2, 0.0, 0.0),
            kdl.Vector(0.0, -0.2, 0.0),
        ]
        expected_positions = [
            kdl.Vector(-0.05, 0.0, 0.0),
            kdl.Vector(0.0, -0.05, 0.0),
            kdl.Vector(0.05, 0.0, 0.0),
            kdl.Vector(0.0, 0.05, 0.0),
        ]
        expected_orientations = [
            kdl.Rotation.RPY(0.0, 0.0, 0.0),
            kdl.Rotation.RPY(0.0, 0.0, 0.5 * math.pi),
            kdl.Rotation.RPY(0.0, 0.0, math.pi),
            kdl.Rotation.RPY(0.0, 0.0, 1.5 * math.pi),
        ]
        for entity_pos, expected_goal_pos, expected_goal_orientation in zip(
          entity_positions,
          expected_positions,
          expected_orientations):
            goal_pos = MoveToGrasp.compute_goal_position(robot_position, entity_pos, radius)
            self.assertAlmostEqual(
                (expected_goal_pos - goal_pos).Norm(),
                0.0,
                msg="Goal position {} does not correspond to expected goal position {}".format(
                    goal_pos, expected_goal_pos)
            )
            goal_orientation = MoveToGrasp.compute_goal_orientation(robot_position, entity_pos, goal_pos, angle_offset)
            self.assertAlmostEqual(
                kdl.diff(expected_goal_orientation, goal_orientation).Norm(),
                0.0,
                msg="Goal orientation {} does not correspond to expected goal orientation {}".format(
                    goal_orientation, expected_goal_orientation)
            )

