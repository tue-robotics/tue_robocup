#!/usr/bin/python
import rospy
import os

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('cocktailparty_rgo2013_tester')

    W = client.SimWorld()

    env = os.environ['ROBOT_ENV']

    if env == "rwc2013":

        person = W.add_object("person-1", "person", 4.1, 0, 0)


        coke = W.add_object("coke-1", "seven_up", 5.385, -7.455, 0.82)
        milk = W.add_object("milk-1", "milk", 5.385, -6.831, 0.84)     
        apple_juice = W.add_object("apple_juice-1", "apple_juice", 5.385, -7.1, 0.84)  

    else:
        print "No test scenario specified for environment {0}.".format(env)
