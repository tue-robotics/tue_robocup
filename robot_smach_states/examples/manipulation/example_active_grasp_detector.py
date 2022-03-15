import rospy
import smach

# TU/e Robotics
from robot_skills import get_robot

# Robot Smach States
import robot_smach_states.util.designators as ds
from robot_smach_states.manipulation.active_grasp_detector import ActiveGraspDetector

# System
import argparse

rospy.init_node("example_active_grasp_detector")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Detects if the robot is holding something")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    # Create node and robot
    rospy.init_node("test_active_grasp_detector")

    robot = get_robot(args.robot)
    arm = ds.ArmDesignator(robot).lockable()

    agd = ActiveGraspDetector(arm, robot)

    rospy.loginfo("Detecting...")
    agd.execute()

