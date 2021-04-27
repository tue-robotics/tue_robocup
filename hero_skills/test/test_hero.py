#! /usr/bin/env python

import rostest
from robot_skills.test_tools import TestRobot
from hero_skills import Hero


class TestHero(TestRobot):
    ROBOT_CLASS = Hero


if __name__ == '__main__':
    rostest.rosrun('hero_skills', 'test_hero', TestHero)
