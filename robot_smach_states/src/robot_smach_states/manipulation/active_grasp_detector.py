from __future__ import absolute_import

# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.arm.arms import GripperTypes
from robot_skills.robot import Robot
from robot_smach_states.util.designators.arm import ArmDesignator


class ActiveGraspDetector(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING], }

    def __init__(self, robot: Robot, arm_designator: ArmDesignator, threshold_difference: float = 0.075,
                 max_torque: float = 0.15) -> None:
        """
        State for detecting whether the robot is holding something using the gripper position.

        Stores current position of the hand motor joint, slightly closes the gripper and then compares the new
        position with the first one. If the difference is bigger than the threshold, robot is holding something

        If the object is too small/thin it will not be able to determine

        :param robot: Robot to execute the state with
        :param arm_designator: designator that resolves to arm to check
        :param threshold_difference: Difference between base and final position
        :param max_torque: Max torque of the gripper to perform the test with
        """

        smach.State.__init__(self, outcomes=['true', 'false', 'failed', 'cannot_determine'])
        self.robot = robot

        self.arm_designator = arm_designator

        self.threshold_difference = threshold_difference
        self.max_torque = max_torque

    def execute(self, userdata=None) -> str:

        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return 'failed'

        first_position = arm._arm.gripper_position_detector.detect()  # First position of the gripper

        if first_position is None:
            rospy.logerr("Cannot retrieve first position")
            return 'failed'
        elif first_position < arm.gripper_position_detector.minimum_position:
            rospy.logdebug("First position is {}".format(first_position))
            return 'cannot_determine'

        else:
            if not arm.gripper.send_goal('close', max_torque=self.max_torque):  # Attempts to close the gripper
                rospy.logerr("Error while closing the gripper")
                return 'failed'

            second_position = arm._arm.gripper_position_detector.detect()

            if second_position is not None:
                if abs(first_position - second_position) < self.threshold_difference:
                    rospy.logdebug("First position is {}\n"
                                   "Second position is {}".format(first_position, second_position))
                    return 'true'
                else:
                    rospy.logdebug("First position is {}\n"
                                   "Second position is {}".format(first_position, second_position))
                    return 'false'

            else:
                rospy.logerr("Cannot retrieve second position")
                return 'failed'
