#! /usr/bin/env python

import rostest
from robot_skills.test_tools import TestRobot
from sergio_skills import Sergio


class TestSergio(TestRobot):
    ROBOT_CLASS = Sergio


if __name__ == '__main__':
    rostest.rosrun('sergio_skills', 'test_sergio', TestSergio)
