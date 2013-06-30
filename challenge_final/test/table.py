#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

import sys

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_challenge_tester_table')

   

    W = client.SimWorld()

    freq = 20

    table_x = 5.7805
    table_y = -1.4685
    table_vy = 0 #0.03
    table_angle = 0
    table_angle_v = 0#.03   

    table = W.add_object("dinner_table", "dinner_table", table_x, table_y, 0)        

    while not rospy.is_shutdown():
            
        while table_angle < 0.5:
            table.set_position(table_x, table_y, 0, rz = table_angle)
            rospy.sleep(1 / freq)
            table_angle += float(table_angle_v) / freq
            table_y += float(table_vy) / freq

        while table_angle > -0.2:
            table.set_position(table_x, table_y, 0, rz = table_angle)
            rospy.sleep(1 / freq)
            table_angle -= float(table_angle_v) / freq
            table_y -= float(table_vy) / freq

