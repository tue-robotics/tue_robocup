#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import os

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fetch_and_carry_tester')

    W = client.SimWorld()

    env = os.environ['ROBOT_ENV']

    if env == "robotics_testlab_B":
        person = W.add_object("david-sim", "person", 5.5, 1.5, 0.8)
        person = W.add_object("david-sim2", "validated_person", 5.5, 0.5, 0.8)

    else:
        print "No test scenario specified for environment {0}.".format(env)
