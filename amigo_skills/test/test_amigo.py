#! /usr/bin/env python

import rostest
from robot_skills.test_tools import TestRobot
from amigo_skills import Amigo


class TestAmigo(TestRobot):
    ROBOT_CLASS = Amigo


if __name__ == '__main__':
    rostest.rosrun('amigo_skills', 'test_amigo', TestAmigo)
