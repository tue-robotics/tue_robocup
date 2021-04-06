import unittest

# ROS
import smach
import rospy

# TU/e
from robot_skills import get_robot
from robot_skills.arm.arms import GripperTypes
from robot_smach_states.utility import collect_arm_requirements


class FirstState(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING],
                               "required_trajectories": ["prepare_grasp"], }

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])

        assert robot.get_arm(
            **self.REQUIRED_ARM_PROPERTIES), "None of the available arms meets all this class's" \
                                             "requirements: {}".format(self.REQUIRED_ARM_PROPERTIES)

    @staticmethod
    def execute():
        return 'succeeded'


class SecondState(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.PINCH],
                               "required_trajectories": ["prepare_place"], }

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])

        assert robot.get_arm(
            **self.REQUIRED_ARM_PROPERTIES), "None of the available arms meets all this class's" \
                                             "requirements: {}".format(self.REQUIRED_ARM_PROPERTIES)

    @staticmethod
    def execute():
        return 'succeeded'


class ThirdState(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.PARALLEL],
                               "required_trajectories": ["wave"], }

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])

        assert robot.get_arm(
            **self.REQUIRED_ARM_PROPERTIES), "None of the available arms meets all this class's" \
                                             "requirements: {}".format(self.REQUIRED_ARM_PROPERTIES)

    @staticmethod
    def execute():
        return 'succeeded'


class FirstStateMachine(smach.StateMachine):
    REQUIRED_ARM_PROPERTIES = {"required_goals": ["reset"], }

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


def compare_lists_in_dicts(reference, result):
    """ Compares a reference dict with the result as produced by the system

    :param reference: Ground truth dict (strings as keys and lists as values)
    :param result: Dict according to the system
    :return: reference and result dicts containing the non overlapping key, value pairs
    """
    for k in list(reference.keys()):
        if k in list(result.keys()):
            if len(reference[k]) == len(result[k]) and set(reference[k]) == set(result[k]):
                del reference[k], result[k]


class TestCollectArmRequirements(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = get_robot('mockbot')

    def test_collect_arm_requirements(self):
        requirements = collect_arm_requirements(FirstStateMachine(self.robot))
        reference = {'required_trajectories': ['prepare_grasp', 'prepare_place'],
                     'required_gripper_types': ['pseudo-gripper-type-any-grasping-will-do',
                                                'gripper-type-pinch'],
                     "required_goals": ["reset"]}
        compare_lists_in_dicts(reference, requirements)
        self.assertEqual(requirements, reference)

    def test_collect_arm_requirements_recursive(self):
        requirements = collect_arm_requirements(SecondStateMachine(self.robot))
        reference = {'required_trajectories': ['wave', 'prepare_grasp', 'prepare_place'],
                     'required_gripper_types': ['gripper-type-parallel', 'pseudo-gripper-type-any-grasping-will-do',
                                                'gripper-type-pinch'],
                     "required_goals": ["reset"]}
        compare_lists_in_dicts(requirements, reference)
        self.assertEqual(requirements, reference)


if __name__ == "__main__":
    rospy.init_node("test_collect_requirements")
    unittest.main()
