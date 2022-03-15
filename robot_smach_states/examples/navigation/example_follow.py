# System
import argparse

# ROS
import rospy

# TU/e Robotics
from robot_skills.get_robot import get_robot

# Robot Smach States
from robot_smach_states.navigation.follow_operator_machine import FollowOperator

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Test the FollowOperator state")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    # Create node, robot and toggle interface
    rospy.init_node("test_following")
    r = get_robot(args.robot, connection_timeout=1.0)

    state = FollowOperator(robot=r)
    state.execute()
