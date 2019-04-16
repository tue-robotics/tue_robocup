#! /usr/bin/env python
import sys
import rospy
from robot_skills import get_robot

if len(sys.argv) < 2:
    print("Please specify a robot name")
    sys.exit()

robot_name = sys.argv[1]

rospy.init_node("audio_test")
robot = get_robot(robot_name)

e = robot.ears
s = robot.speech
robot.head.look_at_standing_person()

# data
print(e.recognize("one", {}))
print(e.recognize("two", {}))
print(e.recognize("three", {}))
print(e.recognize("four", {}))
print(e.recognize("five", {}))
print(e.recognize("six", {}))
