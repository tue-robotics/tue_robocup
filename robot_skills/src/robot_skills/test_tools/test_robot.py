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
        rospy.init_node("test_robot")
        # noinspection PyUnresolvedReferences
        robot = self.ROBOT_CLASS()
