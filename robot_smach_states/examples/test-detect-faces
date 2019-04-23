#!/usr/bin/python

# ROS
import rospy

# Detect face
from robot_skills import get_robot_from_argv
from robot_smach_states import DetectFace


if __name__ == "__main__":

    rospy.init_node('test_detect_faces')

    robot = get_robot_from_argv(index=1)

    detect_state = DetectFace(robot=robot)
    detect_state.execute(None)
