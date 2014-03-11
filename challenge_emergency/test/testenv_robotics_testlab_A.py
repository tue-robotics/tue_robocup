#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import os

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('challenge_tester')

    W = client.SimWorld()

    person1 = W.add_object("person-1", "person", 3.821, -1.379, 0)    
    person2 = W.add_object("person-2", "person", 5.872, 1.256, 0)    
    person3 = W.add_object("person-3", "person", 1.611, 1.908, 0)    
    person4 = W.add_object("person-4", "person", 0.455, 1.501, 0)    
    person5 = W.add_object("person-5", "person", 0.820, -1.741, 0)
    person6 = W.add_object("person_6", "person", 5.0, -0.1, 0.0)  
