import unittest

# ROS
import smach
import rospy

# TU/e
from robot_skills import get_robot
from robot_skills.arm.arms import GripperTypes
from robot_smach_states.utility import check_arm_requirements, collect_arm_requirements


class FirstState(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING],
                               "required_trajectories": ["prepare_grasp"], }

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])

        check_arm_requirements(self, robot)

    @staticmethod
    def execute():
        return 'succeeded'


class SecondState(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.PINCH],
                               "required_trajectories": ["prepare_place"], }

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])

        check_arm_requirements(self, robot)

    @staticmethod
    def execute():
        return 'succeeded'


class ThirdState(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.PARALLEL],
                               "required_trajectories": ["wave"], }

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])

        check_arm_requirements(self, robot)

    @staticmethod
    def execute():
        return 'succeeded'


class DynamicConfigState(smach.State):
    def __init__(self, robot, configuration):
        smach.State.__init__(self, outcomes=['succeeded'])

        self.REQUIRED_ARM_PROPERTIES = {'required_goals': [configuration]}

        check_arm_requirements(self, robot)

    @staticmethod
    def execute():
        return 'succeeded'


class FirstStateMachine(smach.StateMachine):
    def __init__(self, robot):

        smach.StateMachine.__init__(self, outcomes=['succeeded'])

        with self:
            smach.StateMachine.add('FOO', FirstState(robot), transitions={'succeeded': 'BAR'})
            smach.StateMachine.add('BAR', SecondState(robot), transitions={'succeeded': 'succeeded'})


class SecondStateMachine(smach.StateMachine):
    def __init__(self, robot):

        smach.StateMachine.__init__(self, outcomes=['succeeded'])

        with self:
            smach.StateMachine.add('FOO', ThirdState(robot), transitions={'succeeded': 'BAR'})
            smach.StateMachine.add('BAR', FirstStateMachine(robot), transitions={'succeeded': 'succeeded'})


class ThirdStateMachine(smach.StateMachine):
    def __init__(self, robot):

        smach.StateMachine.__init__(self, outcomes=['succeeded'])

        with self:
            smach.StateMachine.add('FOO', FirstState(robot), transitions={'succeeded': 'DYNAMIC1'})
            smach.StateMachine.add('DYNAMIC1', DynamicConfigState(robot, 'config1'),
                                   transitions={'succeeded': 'succeeded'})


class FourthStateMachine(smach.StateMachine):
    def __init__(self, robot):

        smach.StateMachine.__init__(self, outcomes=['succeeded'])

        with self:
            smach.StateMachine.add('FOO', FirstState(robot), transitions={'succeeded': 'DYNAMIC1'})
            smach.StateMachine.add('DYNAMIC1', DynamicConfigState(robot, 'config1'),
                                   transitions={'succeeded': 'DYNAMIC2'})
            smach.StateMachine.add('DYNAMIC2', DynamicConfigState(robot, 'config2'),
                                   transitions={'succeeded': 'succeeded'})


class TestCollectArmRequirements(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = get_robot('mockbot')

    def test_collect_arm_requirements(self):
        requirements = collect_arm_requirements(FirstStateMachine(self.robot))
        reference = {'required_trajectories': set(['prepare_grasp', 'prepare_place']),
                     'required_gripper_types': set([GripperTypes.GRASPING, 'gripper-type-pinch'])}
        self.assertEqual(requirements, reference)

    def test_collect_arm_requirements_recursive(self):
        requirements = collect_arm_requirements(SecondStateMachine(self.robot))
        reference = {'required_trajectories': set(['wave', 'prepare_grasp', 'prepare_place']),
                     'required_gripper_types': set([GripperTypes.PARALLEL, GripperTypes.GRASPING, GripperTypes.PINCH])}
        self.assertEqual(requirements, reference)

    def test_collect_arm_requirements_dynamic1(self):
        requirements = collect_arm_requirements(ThirdStateMachine(self.robot))
        reference = {'required_goals': set(['config1']),
                     'required_trajectories': set(['prepare_grasp']),
                     'required_gripper_types': set([GripperTypes.GRASPING])}
        self.assertEqual(requirements, reference)

    def test_collect_arm_requirements_dynamic2(self):
        requirements = collect_arm_requirements(FourthStateMachine(self.robot))
        reference = {'required_goals': set(['config1', 'config2']),
                     'required_trajectories': set(['prepare_grasp']),
                     'required_gripper_types': set([GripperTypes.GRASPING])}
        self.assertEqual(requirements, reference)


if __name__ == "__main__":
    rospy.init_node("test_collect_requirements")
    unittest.main()
