#! /usr/bin/env python
import rospy
from robot_skills import get_robot_from_argv

rospy.init_node("audio_test")
robot = get_robot_from_argv(index=1)

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
