#!/usr/bin/env python

import rospy
from robot_skills.force_sensor import ForceSensor

rospy.init_node('test_force_sensor_edge_up', anonymous=True)
rospy.sleep(1.0)  # In simulation time is not yet available ...
ForceSensor('/hero/wrist_wrench/raw').wait_for_edge_up()
rospy.loginfo("Edge up detected")
