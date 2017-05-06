#! /usr/bin/env python

import sys
import rospy

from sensor_msgs.msg import RegionOfInterest
import robot_skills.util.kdl_conversions as kdl_conversions

from robot_skills.util.robot_constructor import robot_constructor

if len(sys.argv) < 2:
    print "Please specify a robot name"
    sys.exit()

robot_name = sys.argv[1]

rospy.init_node("arm_test")

robot = robot_constructor(robot_name)
head = robot.head
# Keep track of which errors occur per arm, so we can report them at the end
failed_actions = []

center_unrect = RegionOfInterest(320, 240, 1, 1, False)
topleft_unrect = RegionOfInterest(0, 0, 1, 1, False)

test_points = [center_unrect, topleft_unrect]

try:
    camera_center = head.project_roi(center_unrect, "/amigo/top_kinect/rgb_frame")
    if camera_center.vector.x() != 0.0 and camera_center.vector.y() != 0.0:
        failed_actions += ["Center pixel of sensor is not zero in camera frame"]
except:
    print "Projecting {} to /amigo/top_kinect/rgb_frame didn't work".format(center_unrect)

try:
    camera_top_left = head.project_roi(topleft_unrect, "/amigo/top_kinect/rgb_frame")
    if camera_top_left.vector.x() != 0.0 and camera_top_left.vector.y() != 0.0:
        failed_actions += ["topleft pixel of sensor is not zero in camera frame"]
except:
    print "Projecting {} to /amigo/top_kinect/rgb_frame didn't work".format(topleft_unrect)

for failed_item in failed_actions:
    rospy.logerr(failed_item)
