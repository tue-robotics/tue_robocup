#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('challenge_robonurse_object_spawner')

    W = client.SimWorld()

 
    W.add_object("bottle-1", "sim-coke", 3.05, 2.513, 1.095) 
    W.add_object("bottle-2", "apple_juice", 3.05, 2.70, 1.095)
    W.add_object("bottle-3", "tea_pack", 3.05, 2.913, 1.095)

    W.add_object("granny", "loy", 0.76, 1.08, 0.0)