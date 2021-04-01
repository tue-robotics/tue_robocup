import unittest
import mock

# Robot Skills
from robot_skills.arm import arms
from robot_skills.mockbot import Mockbot
from robot_skills.util.entity import Entity

# Robot Smach States
from robot_smach_states.manipulation import CloseGripperOnHandoverToRobot, HandoverFromHuman, HandoverToHuman, SetGripper
from robot_smach_states.util import designators as ds


class TestHandOverToHuman(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def setUp(self):
        entity = Entity("12345", "dummy", "map", None, None, {}, None, 0)
        self.robot.arms["leftArm"].gripper.occupied_by = entity
        self.arm_ds = ds.OccupiedArmDesignator(self.robot, {"required_goals": ['handover_to_human', 'reset'],
                                                            "required_gripper_types": [arms.GripperTypes.GRASPING]})

    def test_handover_to_human(self):
        state = HandoverToHuman(self.robot, self.arm_ds)
        state.check_consistency()
        self.assertEqual(state.execute(), "succeeded")

        self.robot.arms["rightArm"].send_joint_goal.assert_not_called()
        self.robot.arms["rightArm"].gripper.send_goal.assert_not_called()

        self.robot.arms["leftArm"].send_joint_goal.assert_any_call('handover_to_human', max_joint_vel=mock.ANY, timeout=mock.ANY)
        self.robot.arms["leftArm"].send_joint_goal.assert_any_call('reset', max_joint_vel=mock.ANY, timeout=mock.ANY)

        self.robot.arms["leftArm"].gripper.send_goal.assert_any_call('open')
        self.assertIsNone(self.robot.arms["leftArm"].gripper.occupied_by)


class TestHandOverFromHuman(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def setUp(self):
        entity = Entity("123", "dummy", "map",
                        None, None, {}, None, 0)
        self.robot.arms["leftArm"].gripper.occupied_by = entity
        self.arm_ds = ds.UnoccupiedArmDesignator(self.robot, {"required_goals": ['handover_to_human'],
                                                              "required_gripper_types": [arms.GripperTypes.GRASPING]})
        self.entity = Entity("456", "dummy", "map",
                             None, None, {}, None, 0)

    def test_handover_from_human(self):
        entitydes = ds.VariableDesignator(self.entity)
        state = HandoverFromHuman(self.robot, self.arm_ds, "", entitydes)
        state.check_consistency()
        self.assertEqual(state.execute(), "succeeded")

        self.robot.arms["leftArm"].send_joint_goal.assert_not_called()
        self.robot.arms["leftArm"].gripper.send_goal.assert_not_called()

        self.robot.arms["rightArm"].send_joint_goal.assert_any_call('handover_to_human', max_joint_vel=mock.ANY,
                                                                    timeout=mock.ANY)

        self.robot.arms["rightArm"].handover_detector.handover_to_robot.assert_called_once()

        self.robot.arms["rightArm"].gripper.send_goal.assert_any_call('open')
        self.robot.arms["rightArm"].gripper.send_goal.assert_called_with('close')
        self.assertEqual(self.robot.arms["rightArm"].gripper.occupied_by, self.entity)


class TestSetGripperOpen(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def setUp(self):
        self.arm_ds = ds.ArmDesignator(self.robot, {"required_gripper_types": [arms.GripperTypes.GRASPING],
                                                    'required_arm_name': 'leftArm'})

    def test_set_open(self):
        state = SetGripper(self.robot, self.arm_ds, 'open')
        self.assertEqual(state.execute(), "succeeded")
        self.robot.arms["leftArm"].gripper.send_goal.assert_called_once_with('open')


class TestSetGripperClose(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def setUp(self):
        self.arm_ds = ds.ArmDesignator(self.robot, {'required_arm_name': 'leftArm',
                                                    "required_gripper_types": [arms.GripperTypes.GRASPING]})
        self.entity = Entity("12345", "dummy", "map", None, None, {}, None, 0)
        self.entity_ds = ds.VariableDesignator(self.entity)

    def test_set_close(self):
        state = SetGripper(self.robot, self.arm_ds, 'close', self.entity_ds)
        self.assertEqual(state.execute(), "succeeded")
        self.robot.arms["leftArm"].gripper.send_goal.assert_called_once_with('close')
        self.assertEqual(self.robot.arms["leftArm"].gripper.occupied_by, self.entity)


class TestSetGripperFail(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def setUp(self):
        self.arm_ds = ds.ArmDesignator(self.robot, {'required_arm_name': 'leftArm',
                                                    "required_gripper_types": [arms.GripperTypes.GRASPING]})

    def test_invalid_arm_designator(self):
        invalid_arm_ds = ds.ArmDesignator(self.robot, {'required_arm_name': 'there_is_no_arm_with_this_name'})
        state = SetGripper(self.robot, invalid_arm_ds, 'close')
        self.assertEqual(state.execute(), "failed")


class TestCloseGripper(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def setUp(self):
        self.arm_ds = ds.ArmDesignator(self.robot, {'required_arm_name': 'leftArm',
                                                    "required_gripper_types": [arms.GripperTypes.GRASPING]})
        self.entity_label = "entity_label"

    def test_open_gripper(self):
        state = CloseGripperOnHandoverToRobot(self.robot, self.arm_ds, self.entity_label)
        self.assertEqual(state.execute(), "succeeded")
        self.robot.arms["leftArm"].handover_detector.handover_to_robot.assert_called_once()
        self.assertEqual(self.robot.arms["leftArm"].gripper.occupied_by.id, self.entity_label)


class TestCloseGripperFail(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = Mockbot()

    def setUp(self):
        self.arm_ds = ds.ArmDesignator(self.robot, {'required_arm_name': 'leftArm',
                                                    "required_gripper_types": [arms.GripperTypes.GRASPING]})

    def test_missing_input(self):
        # no entity label or entity designator
        state = CloseGripperOnHandoverToRobot(self.robot, self.arm_ds)
        self.assertEqual(state.execute(), "failed")
        self.robot.arms["leftArm"].handover_detector.handover_to_robot.assert_not_called()


if __name__ == '__main__':
    unittest.main()
