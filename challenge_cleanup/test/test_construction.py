import unittest

from robot_skills.mockbot import Mockbot

from challenge_cleanup.cleanup import setup_statemachine


class TestChallengeConstruction(unittest.TestCase):
    def test_construction(self):
        """
        If no exception is raised, this test will succeed
        """
        robot = Mockbot()
        setup_statemachine(robot)


if __name__ == '__main__':
    unittest.main()
