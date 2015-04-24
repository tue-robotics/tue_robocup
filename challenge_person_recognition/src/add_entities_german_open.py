#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    W = client.SimWorld()

    # person to be learned
    W.add_object("loy-operator", "loy", 8.561, 0.934, 0)

    # # main group
    W.add_object("tim-crowd", "tim", 3.093, -0.393, 0.000)
    W.add_object("loy-crowd", "loy", 3.690, -0.362, 0.000)
    W.add_object("sjoerd-crowd", "sjoerd", 4.500, -0.243, 0.000)

    W.add_object("rob-fake-crowd", "rob", 3.375, 2.527, 0.000)

    #loy, erik, tim, sjoerd, rob