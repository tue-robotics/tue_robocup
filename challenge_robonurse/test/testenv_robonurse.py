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
        W.add_object("coke_d1", "coke",         7.500, 0.4, 0.9)
        W.add_object("coke_d2", "tea_pack",     7.400, 0.6, 0.9)
        W.add_object("coke_d3", "marmalade",    7.350, 0.8, 0.9)
        W.add_object("coke_d4", "cif",          7.400, 1.0, 0.9)