import unittest

from robot_skills.mockbot import Mockbot

from challenge_hand_hand_me_that import setup_state_machine


class TestChallengeConstruction(unittest.TestCase):
    def test_construction(self):
        """
        If no exception is raised, this test will succeed
        """
        robot = Mockbot()
        setup_state_machine(robot)


if __name__ == '__main__':
    unittest.main()
