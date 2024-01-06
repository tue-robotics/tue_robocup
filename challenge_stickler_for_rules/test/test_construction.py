import os
import unittest


class TestChallengeConstruction(unittest.TestCase):
    def test_construction(self):
        """
        If no exception is raised, this test will succeed
        """
        os.environ["ROBOT_ENV"] = "impuls"
        from robot_skills.mockbot import Mockbot
        from challenge_stickler_for_rules.stickler_for_rules import SticklerForRules
        robot = Mockbot()
        SticklerForRules(robot)


if __name__ == '__main__':
    unittest.main()
