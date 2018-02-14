#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

import robot_skills
import robot_skills.amigo

import geometry_msgs
import sensor_msgs
import amigo_msgs
import random
import time

if __name__ == "__main__":
    rospy.init_node('amigo_skills_test_full')

    amigo = robot_skills.amigo.Amigo(wait_services=True)
    amigo.leftArm.reset()
    amigo.rightArm.reset()
    time.sleep(1.0)
    while not rospy.is_shutdown():
        amigo.torso._send_goal([random.random()])
        rospy.sleep(rospy.Duration(0.1))


