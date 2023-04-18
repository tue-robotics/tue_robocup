import os
import unittest


class TestChallengeConstruction(unittest.TestCase):
    def test_construction(self):
        """
        If no exception is raised, this test will succeed
        """
        os.environ["ROBOT_ENV"] = "impuls"
        from robot_skills.mockbot import Mockbot
        from challenge_clean_the_table.clean_the_table import setup_statemachine
        robot = Mockbot()
        setup_statemachine(robot)


if __name__ == '__main__':
    unittest.main()
