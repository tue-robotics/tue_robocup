#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

import sys
import os
import itertools

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_challenge_tester')

    W = client.SimWorld()

    env = os.environ['ROBOT_ENV']

    if env == "robotics_testlabs":

        W.add_object("coke-1","coke", 3.19,4.1,0.85)


    elif env == "rwc2015":

        # Put stuff on the dinner table
        W.add_object("coke11", "sim-coke", 1.15, -4.13, 1.10)
        W.add_object("coke12", "sim-coke", 1.15, -4.13, 0.75)
        W.add_object("coke13", "sim-coke", 1.15, -4.33, 0.75)
        W.add_object("coke14", "sim-coke", 1.15, -4.7,  0.75)
