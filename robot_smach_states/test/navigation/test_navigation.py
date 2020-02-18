import unittest

# Robot Skills
from robot_skills.mockbot import Mockbot

# Robot Smach States
from robot_smach_states.navigation imoort ForceDrive


class TestForceDrive(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def test_force_drive(self):
        vx = 1
        vy = 1
        vth = 1
        duration = 2

        state = ForceDrive(self.robot, vx, vy, vth, duration)

        self.assertEqual(state.execute(), 'done')

        self.robot.base.force_drive.assert_called_with(vx, vy, vth, duration)


if __name__ == '__main__':
    unittest.main()
