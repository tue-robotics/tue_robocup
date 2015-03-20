#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    W = client.SimWorld()

    # person at the door 
    # W.add_object("loy-1", "loy", 1.114, 4.233, 0)

    # crowd in the main room
    # W.add_object("loy-2", "loy", 0.140, -0.612, 0.000)
    # W.add_object("loy-3", "loy", 0.105, 0.215, 0.000)
    # W.add_object("loy-4", "loy", 0.138, 0.714, 0.000)

    W.add_object("loy-2", "loy", 0.234, 0.912, 0.000)
    W.add_object("loy-3", "sjoerd", 0.385, 0.252, 0.000)
    W.add_object("loy-4", "tim", 0.266, -0.280, 0.000)

    #loy, erik, tim, sjoerd, rob