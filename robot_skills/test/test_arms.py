#! /usr/bin/env python

import sys
import rospy

import robot_skills.util.kdl_conversions as kdl_conversions

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

if not robot.leftArm.operational:
    rospy.logerr("Arm is not operational")
    sys.exit(-1)

goal1 = kdl_conversions.kdlFrameStampedFromXYZRPY(0.192, 0.125, 0.748, 0, 0, 0, "/"+robot.robot_name+"/base_link")

robot.leftArm.send_goal(goal1)

robot.leftArm.wait_for_motion_done()

# TODO: Now check that the hand frame is within some tolerance to the desired goal.

robot.leftArm.reset()
