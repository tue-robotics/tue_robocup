#! /usr/bin/env python

import rospy
import unittest

# robot skills
from robot_skills.hero import Hero


class TestHeroInterface(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_hero')

    def test_construct_interface(self):
        robot = Hero()
        self.assertIsNotNone(robot)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('robot_skills', 'test_hero_interface', TestHeroInterface)
