#!/usr/bin/env python
#ROS
import rospy
import smach
import argparse

from robot_smach_states.manipulation import ArmToJointConfig
from robot_smach_states.util.designators import ArmDesignator
from robot_skills.get_robot import get_robot


class TestArmToJointConfig(smach.StateMachine):
    """StateMachine used to test the ArmToJointConfig in robot_smach_states"""
    def __init__(self, robot):
        """
        :param robot: robot object used to test the arm (type:string)
        """
        super().__init__(outcomes=["succeeded", "failed"])
        self.robot = robot
        self.arm = ArmDesignator(self.robot, {'required_goals': {'prepare_grasp', 'reset'}})
        with self:
            self.add("MOVE_TO_GRASP",
                     ArmToJointConfig(self.robot, self.arm, "prepare_grasp"),
                     transitions={"failed": "failed",
                                  "succeeded": "RESET_ARM"}
                     )
            self.add("RESET_ARM",
                     ArmToJointConfig(self.robot,self.arm, 'reset'),
                     transitions={"failed": "failed",
                                  "succeeded": "succeeded"}
                     )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Test moving Arm to a default configuration")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("Test_Arm_to_Joint_config")
    robot = get_robot(args.robot)

    rospy.loginfo("Starting the StateMachine to Test Arm")
    sm = TestArmToJointConfig(robot)
    rospy.loginfo("Executing the StateMachine to Test Arm")
    sm.execute()
    rospy.loginfo("Done")
