import unittest
import mock

# Robot Skills
from robot_skills.mockbot import Mockbot
from robot_skills.util.entity import Entity

# Robot Smach States
import robot_smach_states as states
from robot_smach_states.util import designators as ds


class TestHandOverToHuman(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def setUp(self):
        entity = Entity("12345", "dummy", "/map",
                             None, None, {}, None, 0)
        self.robot.arms["leftArm"].occupied_by = entity
        self.arm_ds = ds.OccupiedArmDesignator(self.robot, {"required_goals": ['handover_to_human', 'reset']})

    def test_handover_to_human(self):
        state = states.HandoverToHuman(self.robot, self.arm_ds)
        state.check_consistency()
        self.assertEqual(state.execute(), "succeeded")

        self.robot.arms["rightArm"].send_joint_goal.assert_not_called()
        self.robot.arms["rightArm"].send_gripper_goal.assert_not_called()

        self.robot.arms["leftArm"].send_joint_goal.assert_any_call('handover_to_human', max_joint_vel=mock.ANY, timeout=mock.ANY)
        self.robot.arms["leftArm"].send_joint_goal.assert_any_call('reset', max_joint_vel=mock.ANY, timeout=mock.ANY)

        self.robot.arms["leftArm"].send_gripper_goal.assert_any_call('open', mock.ANY, max_torque=mock.ANY)
        self.assertIsNone(self.robot.arms["leftArm"].occupied_by)


if __name__ == '__main__':
    unittest.main()
