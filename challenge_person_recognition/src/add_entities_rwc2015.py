#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    W = client.SimWorld()

    # people in the room
    W.add_object("person-1", "loy", 9.005, -1.867, 0.000)
    W.add_object("person-2", "erik", 8.525, -1.894, 0.000)
    W.add_object("person-3", "tim", 7.970, -1.909, 0.000)
    W.add_object("person-4", "rob", 7.604, -1.873, 0.000)
    W.add_object("person-5", "sjoerd", 7.151, -2.278, 0.000)
    W.add_object("person-6", "rob", 6.618, -1.725, 0.000)

    # operator
    W.add_object("person-7", "sjoerd", 10.842, -5.057, 0.000)
    
    #loy, erik, tim, sjoerd, rob