import os
import unittest


class TestChallengeConstruction(unittest.TestCase):
    def test_construction(self):
        """
        If no exception is raised, this test will succeed
        """
        os.environ["ROBOT_ENV"] = "robotics_testlabs"
        from robot_skills.mockbot import Mockbot
        from challenge_receptionist.receptionist import ChallengeReceptionist
        robot = Mockbot()
        ChallengeReceptionist(robot)


if __name__ == '__main__':
    unittest.main()
