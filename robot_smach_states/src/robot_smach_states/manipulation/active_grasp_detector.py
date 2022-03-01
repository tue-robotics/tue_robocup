from __future__ import absolute_import

# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.arm.arms import GripperTypes


class ActiveGraspDetector(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING], }

    def __init__(self, arm_designator, robot):
        """
        State for detecting whether the robot is holding something

        :param arm_designator: designator that resolves to arm to check
        :param robot: Robot to execute the state with
        :param timeout: float amount of time the procedure may take
        """
        smach.State.__init__(self, outcome=['succeeded', 'failed', 'timeout'])
        self.robot = robot

        self.arm_designator = arm_designator

        self.threshold_difference = 0.075  # Difference between base and final position
        self.minimum_position = 0.2  # To be determined by testing

        self.first_position = None  # First position of the gripper
        self.second_position = None  # Position of the gripper after measuring

        self.store_position = False  # Flag to store the position

    def execute(self, userdata=None):
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return 'failed'

        self.first_position = arm._arm.grasp_position_detector.detect()

        if self.first_position is None:
            return 'failed'
        else:
            arm.gripper.send_goal('close', max_torque=0.015)

            self.second_position = arm._arm.grasp_position_detector.detect()

            if self.second_position is not None:
                if abs(self.first_position - self.second_position) < self.threshold_difference:
                    return 'succeeded'
                else:
                    return 'failed'
            else:
                return 'failed'
