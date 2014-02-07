#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import os

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fetch_and_carry_tester')

    W = client.SimWorld()

    env = os.environ['ROBOT_ENV']

    if env == "rwc2013":
        person = W.add_object("person-1", "person", 4.35, -0.8, 0)
        person = W.add_object("person-2", "person", 5.19, -2.7, 0)   
        person = W.add_object("person-3", "person", 9.44, -2.8, 0)   
        person = W.add_object("person-4", "person", 10.07, -0.1, 0)   
        person = W.add_object("person-5", "person", 7.54, -0.7, 0)   

        coke = W.add_object("coke-1", "seven_up", 5.385, -7.455, 0.82)
        milk = W.add_object("milk-1", "milk", 5.385, -6.831, 0.84)     
        apple_juice = W.add_object("apple_juice-1", "apple_juice", 5.385, -7.1, 0.84)  

    elif env == "robotics_testlab_A":
        coke = W.add_object("coke-1", "coke", 5.107,  1.952, 0.87)
        milk = W.add_object("milk-1", "milk", 4.857,  1.952, 0.87)    
        fanta = W.add_object("fanta-1", "fanta", 4.57,  1.942, 0.87)   
        person = W.add_object("person-1", "person", 1.0, 0.0, 0.0)  

    else:
        print "No test scenario specified for environment {0}.".format(env)
