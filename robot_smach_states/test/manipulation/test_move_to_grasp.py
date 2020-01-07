# System
import unittest

# ROS
import PyKDL as kdl

# MoveToGrasp
# noinspection PyProtectedMember
from robot_smach_states.manipulation.move_to_grasp import _point_between_points_at_distance, _create_straight_line_plan


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
