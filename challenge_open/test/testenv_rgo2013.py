#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('open_challenge_rgo2013_tester')

    W = client.SimWorld()

    table_x = 3
    table_y = -0.5

    table = W.add_object("table-1", "table", table_x, table_y, 0)
    table = W.add_object("coke-1",  "coke",  table_x, table_y, 0.85)
