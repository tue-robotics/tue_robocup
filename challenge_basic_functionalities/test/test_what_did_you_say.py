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
        person = W.add_object("person-1", "person", 5.5, 2.5, 0.8)
        #human = W.add_object("human", "humanlike_box2", 5.5, 2.5, 0)
        #human2 = W.add_object("human2", "humanlike_box_2", 5, 1.5, 0)
        # table = W.add_object("table-1", "table", 5.5,1.5, 0)
        # table = W.add_object("table-2", "table", 5.2,1.5, 0)

        table = W.add_object("table-1", "table", 8.5,1.5, 0)
        table = W.add_object("table-2", "table", 8.2,1.5, 0)

    else:
        print "No test scenario specified for environment {0}.".format(env)
