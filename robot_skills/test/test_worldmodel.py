#! /usr/bin/env python

import sys
import rospy

import robot_skills.util.kdl_conversions as kdl_conversions

from robot_skills.util.robot_constructor import robot_constructor

if len(sys.argv) < 2:
    print "Please specify a robot name"
    sys.exit()

robot_name = sys.argv[1]

rospy.init_node("worldmodel_test")

robot = robot_constructor(robot_name)
