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

       # Put stuff in the cabinet

        W.add_object("coke_d1", "sim-coke",         1.150, -5.418, 0.71)
        W.add_object("coke_d2", "sim-papaya_milk",  1.134, -5.218, 0.71)
        W.add_object("coke_d3", "sim-pure_milk",    1.110, -5.018, 0.71)

