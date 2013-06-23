#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('open_challenge_rgo2013_tester')

    W = client.SimWorld()

    table_x = 1.80
    table_y = -1.0

    table = W.add_object("table-1", "table", table_x, table_y, 0)
    table = W.add_object("coke-1",  "coke",  table_x - 0.3, table_y - 0.2, 0.86)
    table = W.add_object("teapack-1",  "tea_pack",  table_x + 0.3, table_y + 0.2, 0.85)
#    table = W.add_object("cif-1",  "cif",  table_x + 0.4, table_y + 0.1, 0.92)
