#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('challenge_tester')

    W = client.SimWorld()

    person1 = W.add_object("person-1", "person", 1, -4, 0)    
    person2 = W.add_object("person-2", "person", 3, -4, 0)    
    person3 = W.add_object("person-3", "person", 2, -1, 0)    
    person4 = W.add_object("person-4", "person", 4, -4, 0)    
