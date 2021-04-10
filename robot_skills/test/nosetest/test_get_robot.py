import unittest


class TestGetRobot(unittest.TestCase):
    def test_get_robot(self):
        """
        Try to get mockbot and assert it is a Robot instance
        This verifies that registration of robots is succesfull
        """
        from robot_skills import get_robot
        from robot_skills.robot import Robot
        mockbot = get_robot("mockbot")
        self.assertIsInstance(mockbot, Robot)


if __name__ == '__main__':
    unittest.main()
