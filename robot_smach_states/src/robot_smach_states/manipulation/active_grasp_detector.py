from __future__ import absolute_import

# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.arm.arms import GripperTypes


class ActiveGraspDetector(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING], }

    def __init__(self, arm_designator, robot, threshold_difference=0.075, minimum_position=0.2, max_torque=0.015):
        """
        State for detecting whether the robot is holding something

        :param arm_designator: designator that resolves to arm to check
        :param robot: Robot to execute the state with
        :param threshold_difference: Difference between base and final position
        :param minimum_position: Minimum position to assume that the gripper is holding something
        :param max_torque: Max torque of the gripper to perform the test with
        """
        smach.State.__init__(self, outcome=['true', 'false', 'failed'])
        self.robot = robot

        self.arm_designator = arm_designator

        self.threshold_difference = threshold_difference
        self.minimum_position = minimum_position
        self.max_torque = max_torque

    def execute(self, userdata=None):
        first_position, second_position = None  # Init position variables

        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return 'failed'

        first_position = arm._arm.grasp_position_detector.detect()  # First position of the gripper

        if first_position is None:
            return 'failed'
        else:

            if not arm.gripper.send_goal('close', max_torque=self.max_torque):
                rospy.logerr("Error while closing the gripper")  # Ignores the error but notifies it

            self.second_position = arm._arm.grasp_position_detector.detect()

            if self.second_position is not None:
                if abs(self.first_position - self.second_position) < self.threshold_difference:
                    return 'true'
                else:
                    return 'false'
            else:
                return 'failed'
