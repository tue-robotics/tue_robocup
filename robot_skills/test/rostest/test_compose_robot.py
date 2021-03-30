#! /usr/bin/env python

import unittest

import rospy

from robot_skills.functionalities import RobotFunc, add_functionalities
# robot skills
from robot_skills.robot import Robot
from robot_skills.robot_part import RobotPart


# robotpart subclass
class TestPart(RobotPart):
    def some_function(self):
        pass

# functionality which can be added to the subclass
class TestFunc1(RobotFunc):
    def __init__(self):
        super(TestFunc1, self).__init__("extra_functionality",
                                        TestPart,
                                        {"testfunction1": testfunction1})
    def check_requirements(self, part):
        return True

class TestFunc2(RobotFunc):
    def __init__(self):
        super(TestFunc2, self).__init__("extra_functionality_with_harsh_requirements",
                                        TestPart,
                                        {"testfunction2": testfunction2})
    def check_requirements(self, part):
        return False

def testfunction1(self):
    return 'test 1 called'

def testfunction2(self):
    return 'test 2 called'


class TestComposeRobot(unittest.TestCase):
    def test_compose_robot(self):
        robot = Robot("testbot")
        part = RobotPart("testbot", robot.tf_listener)

        robot.add_body_part('mypart', part)
        self.assertTrue(hasattr(robot, 'mypart'))
        self.assertIs(robot.mypart, part)

    def test_compose_functionality(self):
        robot = Robot("testbot")
        part = RobotPart("testbot", robot.tf_listener)
        specialpart = TestPart("testbot", robot.tf_listener)

        robot.add_body_part('regular_part', part)
        robot.add_body_part('special_part', specialpart)

        add_functionalities(robot, [TestFunc1(), TestFunc2()])
        self.assertTrue(hasattr(robot.special_part, 'testfunction1'))
        self.assertFalse(hasattr(robot.regular_part, 'testfunction1'))
        self.assertEqual(robot.special_part.testfunction1(), 'test 1 called')

        self.assertFalse(hasattr(robot.special_part, 'testfunction2'))


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_composable_robot_interface')
    rostest.rosrun('robot_skills', 'test_composable_robot_interface', TestComposeRobot)
