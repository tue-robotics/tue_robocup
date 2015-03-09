#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    W = client.SimWorld()

    # person at the door 
    W.add_object("loy-1", "loy", 1.114, 4.233, 0)

    # crowd in the main room
    W.add_object("loy-2", "loy", 0.713, 0.219, 0.000)
    W.add_object("loy-3", "loy", 0.397, -0.090, 0.000)
    W.add_object("loy-4", "loy", 0.613, 0.517, 0.000)
