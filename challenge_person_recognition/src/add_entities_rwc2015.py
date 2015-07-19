#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    W = client.SimWorld()

    # people in the room
    W.add_object("person-1", "loy", 7.222, -0.907, 0.000)
    W.add_object("person-2", "erik", 7.701, -1.069, 0.000)
    W.add_object("person-3", "tim", 8.353, -1.008, 0.000)
    W.add_object("person-4", "sjoerd", 9.204, -0.906, 0.000)
    # W.add_object("person-5", "sjoerd", 7.151, -2.278, 0.000)
    # W.add_object("person-6", "rob", 6.618, -1.725, 0.000)

    # operator
    W.add_object("person-7", "sjoerd", 7.125, -5.639, 0.000)
    
    #loy, erik, tim, sjoerd, rob
