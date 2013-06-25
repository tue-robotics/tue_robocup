#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('open_challenge_rgo2013_tester')

    W = client.SimWorld()

    table_x = 5.683
    table_y = -2.637

    table = W.add_object("table-1", "table", table_x, table_y, 0)
    W.add_object("coke-1",  "coke",  table_x - 0.5, table_y + 1.1, 0.8)
    W.add_object("teapack-1",  "tea_pack",  table_x - 0.5, table_y + 1.4, 0.8)
    W.add_object("person-1", "person", 3.032, -6.051, 0)
#    table = W.add_object("cif-1",  "cif",  table_x + 0.4, table_y + 0.1, 0.92)
