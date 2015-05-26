#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    W = client.SimWorld()

    # person at the learning spot
    W.add_object("loy-1",   "tim",      2.157, 4.681, 0)

    W.add_object("sjoerd-2","sjoerd",   1.443, 0.794, 0.000)
    W.add_object("sjoerd-3","sjoerd",   2.512, 0.868, 0.000)
    W.add_object("tim-4",   "tim",      2.244, 0.972, 0.000)
    W.add_object("rob-5",   "rob",      1.976, 0.781, 0.000)
    W.add_object("loy-6",   "loy",      1.862, 0.955, 0.000)

    #loy, erik, tim, sjoerd, rob