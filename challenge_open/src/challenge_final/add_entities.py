#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import itertools
import os

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    W = client.SimWorld()

    env = os.environ['ROBOT_ENV']

    if env == "rwc2015":

        # Right_bookcase
        W.add_object("sjoerd", "loy", 6.125, -2.28, 0)
        W.add_object("luis", "tim", 8.556, -8.869, 0)
        W.add_object("object-1", "sim-papaya_milk", 7.511, 0.5, 0.75)
