#!/usr/bin/python

# ROS
import rospy
import argparse

# Detect face
from robot_skills import get_robot
from robot_smach_states import DetectFace


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test if the robot can detect the face of the person standing in front"
                                                 "of it")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node('test_detect_faces')
    robot = get_robot(args.robot)

    detect_state = DetectFace(robot=robot)
    detect_state.execute(None)
