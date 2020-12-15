import unittest

# ROS
import smach
import rospy

# TU/e
from robot_skills import get_robot
from robot_skills.arms import GripperTypes, collect_arm_requirements


class FirstState(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING],
                               "required_trajectories": ["prepare_grasp"], }

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])

        assert robot.get_arm(
            **self.REQUIRED_ARM_PROPERTIES), "None of the available arms meets all this class's" \
                                             "requirements: {}".format(self.REQUIRED_ARM_PROPERTIES)

    def execute(self, userdata):
        return 'succeeded'


class SecondState(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.PINCH],
                               "required_trajectories": ["prepare_place"], }

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])

        assert robot.get_arm(
            **self.REQUIRED_ARM_PROPERTIES), "None of the available arms meets all this class's" \
                                             "requirements: {}".format(self.REQUIRED_ARM_PROPERTIES)

    def execute(self, userdata):
        return 'succeeded'


class ThirdState(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.PARALLEL],
                               "required_trajectories": ["wave"], }

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])

        assert robot.get_arm(
            **self.REQUIRED_ARM_PROPERTIES), "None of the available arms meets all this class's" \
                                             "requirements: {}".format(self.REQUIRED_ARM_PROPERTIES)

    def execute(self, userdata):
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


class TestCollectArmRequirements(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = get_robot('hero')

    def test_collect_arm_requirements(self):
        requirements = collect_arm_requirements(FirstStateMachine(self.robot))
        self.assertEqual(requirements, {'required_trajectories': ['prepare_grasp', 'prepare_place'],
                                        'required_gripper_types': ['pseudo-gripper-type-any-grasping-will-do',
                                                                   'gripper-type-pinch']})

    def test_collect_arm_requirements_recursive(self):
        requirements = collect_arm_requirements(SecondStateMachine(self.robot))
        rospy.logerr(requirements)
        # {'required_trajectories': ['wave'], 'required_gripper_types': ['gripper-type-parallel']}

        # self.assertEqual(requirements, {'required_trajectories': ['prepare_grasp', 'prepare_place', 'wave'],
        #                                 'required_gripper_types': ['pseudo-gripper-type-any-grasping-will-do',
        #                                                            'gripper-type-pinch', 'gripper-type-grasping']})


if __name__ == "__main__":
    rospy.init_node("test_collect_requirements")
    unittest.main()

