#! /usr/bin/env python

import unittest

# Robot Skills
from robot_skills.mockbot import Mockbot

# Robot Smach States
import robot_smach_states as states


class TestForceDrive(unittest.TestCase):
    def setUp(self):
        self.robot = Mockbot()

    def test_force_drive(self):
        vx = 1
        vy = 1
        vth = 1
        duration = 2

        state = states.ForceDrive(self, self.robot, vx, vy, vth, duration)

        state.execute()

        v = [vx, vy, vth]

        self._cmd_vel.publish.assert_called_with(v)


if __name__ == '__main__':
    unittest.main()
