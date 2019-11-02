import unittest
import mock

# Robot Skills
from robot_skills.mockbot import Mockbot
from robot_skills.util.entity import Entity

# Robot Smach States
import robot_smach_states as states
from robot_smach_states.util import designators as ds


class TestHandOverFromHuman(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def setUp(self):
        entity = Entity("123", "dummy", "/map",
                        None, None, {}, None, 0)
        self.robot.arms["leftArm"].occupied_by = entity
        self.arm_ds = ds.UnoccupiedArmDesignator(self.robot, {"required_goals": ['handover_to_human']})
        self.entity = Entity("456", "dummy", "/map",
                             None, None, {}, None, 0)

    def test_handover_from_human(self):
        entitydes = ds.VariableDesignator(self.entity)
        state = states.HandoverFromHuman(self.robot, self.arm_ds, "", entitydes)
        state.check_consistency()
        self.assertEqual(state.execute(), "succeeded")

        self.robot.arms["leftArm"].send_joint_goal.assert_not_called()
        self.robot.arms["leftArm"].send_gripper_goal.assert_not_called()

        self.robot.arms["rightArm"].send_joint_goal.assert_any_call('handover_to_human', max_joint_vel=mock.ANY, timeout=mock.ANY)

        self.robot.arms["rightArm"].handover_to_robot.assert_called_once()

        self.robot.arms["rightArm"].send_gripper_goal.assert_any_call('open', mock.ANY, max_torque=mock.ANY)
        self.robot.arms["rightArm"].send_gripper_goal.assert_called_with('close', mock.ANY, max_torque=mock.ANY)
        self.assertEqual(self.robot.arms["rightArm"].occupied_by, self.entity)


if __name__ == '__main__':
    unittest.main()
