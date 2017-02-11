#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')

import random
import time

import sys

import rospy

if len(sys.argv) < 2:
    print "Please specify a robot name"
    sys.exit()

robot_name = sys.argv[1]

rospy.init_node("audio_test")
if robot_name == "amigo":
    from robot_skills.amigo import Amigo
    robot = Amigo()
elif robot_name == "sergio":
    from robot_skills.sergio import Sergio
    robot = Sergio()
else:
    print "Unknown robot '%s'"%robot_name
    sys.exit()

e = robot.ears
s = robot.speech
robot.head.look_at_standing_person()

# data
print e.recognize("one", {})
print e.recognize("two", {})
print e.recognize("three", {})
print e.recognize("four", {})
print e.recognize("five", {})
print e.recognize("six", {})
