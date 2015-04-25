#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('challenge_robonurse_object_spawner')

    W = client.SimWorld()

    W.add_object("bottle-1", "coke", 6.224, 8.306, 1.495)
    W.add_object("bottle-2", "bubblemint", 5.9, 8.306, 1.095)
    W.add_object("bottle-3", "mints",  5.6, 8.306, 1.095)

    W.add_object("granny", "loy", 7.224, 7.306, 0.095)
