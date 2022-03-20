from __future__ import absolute_import

# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.arm.arms import GripperTypes
from robot_skills.arm.gripper_position_detector import GripperPositionDetector


class ActiveGraspDetector(smach.State):
    REQUIRED_ARM_PROPERTIES = {"required_gripper_types": [GripperTypes.GRASPING], }

    def __init__(self, robot, arm_designator, threshold_difference=0.075, minimum_position=-0.82, max_torque=0.15):
        """
        State for detecting whether the robot is holding something using the gripper position.
        
        Stores current position of the hand motor joint, slightly closes the gripper and then compares the new
        position with the first one. If the difference is bigger than the threshold, robot is holding something
        
        If the object is too small/thin it will not be able to determine

        :param robot: Robot to execute the state with
        :param arm_designator: designator that resolves to arm to check
        :param threshold_difference: Difference between base and final position
        :param minimum_position: Minimum position to assume that the gripper is holding something
        :param max_torque: Max torque of the gripper to perform the test with
        """
        smach.State.__init__(self, outcomes=['true', 'false', 'failed', 'Cannot determine'])
        self.robot = robot

        self.arm_designator = arm_designator

        self.threshold_difference = threshold_difference
        self.minimum_position = minimum_position
        self.max_torque = max_torque

        
    def execute(self, userdata=None):

        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return 'failed'

        first_position = arm._arm.gripper_position_detector.detect()  # First position of the gripper

        if first_position is None:
            rospy.loginfo("Cannot retrieve first position")
            return 'failed'
        elif first_position < self.minimum_position:
            rospy.loginfo("First position is {}".format(first_position))
            return 'Cannot determine'

        else:
            if not arm.gripper.send_goal('close', max_torque=self.max_torque):
                rospy.logerr("Error while closing the gripper")  # Ignores the error but notifies it

            second_position = arm._arm.gripper_position_detector.detect()

            if second_position is not None:
                
                if abs(first_position - second_position) < self.threshold_difference:
                    rospy.loginfo("First position is {}\n"
                                  "Second position is {}".format(first_position, second_position))
                    return 'true'
                
                else:
                    rospy.loginfo("First position is {}\n"
                                  "Second position is {}".format(first_position, second_position))
                    return 'false'
            
            else:
                rospy.loginfo("Cannot retrieve second position")
                return 'failed'
