#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('open_challenge_rgo2013_tester')

    W = client.SimWorld()

    person = W.add_object("person-1", "person", 1, -0.7, 0)    
    coke = W.add_object("coke-1", "coke", 5, 2, 0.8)    
