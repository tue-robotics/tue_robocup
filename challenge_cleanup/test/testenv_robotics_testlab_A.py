#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('open_challenge_rgo2013_tester')

    W = client.SimWorld()

    table = W.add_object("coke-1", "coke", 4.965, -1.49, 0.5)    
