from smach import StateMachine
import unittest

from robot_skills.mockbot import Mockbot
from challenge_cleanup.cleanup import setup_statemachine


class TestChallengeConstruction(unittest.TestCase):
    def test_construction(self):
        """
        If no exception is raised, this test will succeed
        """
        robot = Mockbot()
        self.assertIsInstance(setup_statemachine(robot), StateMachine)


if __name__ == '__main__':
    unittest.main()
