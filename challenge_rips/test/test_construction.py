import unittest

from robot_skills.get_robot import get_robot

from rips import setup_statemachine


class TestChallengeConstruction(unittest.TestCase):
    def test_something(self):
        robot = get_robot("mockbot")
        setup_statemachine(robot)


if __name__ == '__main__':
    unittest.main()
