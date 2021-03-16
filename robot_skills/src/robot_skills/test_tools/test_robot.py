
import rospy
import unittest


class TestRobot(unittest.TestCase):
    def test_robot(self):
        rospy.init_node("test_robot")
        # noinspection PyUnresolvedReferences
        robot = self.ROBOT_CLASS()
