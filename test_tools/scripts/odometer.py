#! /usr/bin/env python

import rospy
from test_tools import odometer

rospy.init_node("odometer")

r = float(rospy.get_param("~rate", 1/60.0))
length = int(rospy.get_param("~buffer_length", 1))
path = rospy.get_param("~path", odometer.DEFAULT_PATH)
filename = rospy.get_param("~filename", odometer.DEFAULT_FILENAME)

meter = odometer.Odometer(path, filename)
rate = rospy.Rate(max(r, 1e-3))

while not rospy.is_shutdown():
    meter.sample()
    meter.activate_write(length)
    rate.sleep()
