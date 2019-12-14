import os
import unittest

from robot_skills.mockbot import Mockbot
from robot_smach_states import ShowImageState


class TestShowImage(unittest.TestCase):
    def test_show_image(self):
        robot = Mockbot()

        # Test if state returns succeeded if file exists
        with open("/tmp/foo", "w") as f:
            f.write("bar")
        state = ShowImageState(robot, "/tmp/foo")
        self.assertEqual(state.execute(), "succeeded")
        os.remove("/tmp/foo")

        # Test if state returns failed if file does not exist
        state = ShowImageState(robot, "/tmp/bar")
        self.assertEqual(state.execute(), "failed")


if __name__ == '__main__':
    unittest.main()
