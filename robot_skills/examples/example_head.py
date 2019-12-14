#! /usr/bin/env python

import sys
import rospy

from robot_skills.util.robot_constructor import robot_constructor

if len(sys.argv) < 2:
    print("Please specify a robot name")
    sys.exit()

robot_name = sys.argv[1]

rospy.init_node("head_test")

robot = robot_constructor(robot_name)
head = robot.head
# Keep track of which errors occur, so we can report them at the end
failed_actions = []

for failed_item in failed_actions:
    rospy.logerr(failed_item)
