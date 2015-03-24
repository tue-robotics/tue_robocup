#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    W = client.SimWorld()

    W.add_object("loy-2", "loy", 0.0, 0.0, 0.000)

    #loy, erik, tim, sjoerd, rob