"""
Contains test tools enabling testing of deriving robot API classes to make sure they meet the required API.
"""

import rospy
import unittest


class TestRobot(unittest.TestCase):
    """
    Testcase to include in deriving <robot_name>_skills packages

    To use this, create a Python file (in this case: test_amigo.py) with the following (example) contents:
    #! /usr/bin/env python

    import rostest
    from robot_skills.test_tools import TestRobot
    from amigo_skills import Amigo


    class TestAmigo(TestRobot):
        ROBOT_CLASS = Amigo


    if __name__ == '__main__':
        rostest.rosrun('amigo_skills', 'test_amigo', TestAmigo)

    The corresponding rostest file (in this case: test_amigo.test) with the following contents:
    <?xml version="1.0"?>
    <launch>
        <!-- N.B.: test-name MUST be amigo in order for namespacing to work -->
        <test test-name="amigo" pkg="amigo_skills" type="test_amigo.py">
            <rosparam file="$(find amigo_description)/custom/skills.yaml" command="load"/>
        </test>
    </launch>

    As is clear from the rostest file, some parameters (in this case included in the amigo_description package) need
    to be loaded on the parameter server.
    """
    def test_robot(self):
        """
        Top level test method. Calls other functions in this class
        """
        rospy.init_node("test_robot")
        robot = self.ROBOT_CLASS()
        self.lights(robot)
        self.speech(robot)

    def part_commons(self, part):
        self.assert_has_attr(part, "close")
        self.assert_has_attr(part, "selfreset")

    def lights(self, robot):
        self.assert_has_attr(robot, "lights")
        self.part_commons(robot.lights)
        self.assert_has_attr(robot.lights, "set_color")
        self.assert_has_attr(robot.lights, "set_color_rgba_msg")

    def speech(self, robot):
        self.assert_has_attr(robot, "speech")
        self.part_commons(robot.speech)
        self.assert_has_attr(robot.speech, "speak")

    def assert_has_attr(self, obj, intended_attr):
        test_bool = hasattr(obj, intended_attr)

        # self.assertTrue(test_bool, msg=f"{obj} lacks attribute {intended_attr}")  # Python3
        self.assertTrue(test_bool, msg="{} lacks attribute {}".format(obj, intended_attr))


