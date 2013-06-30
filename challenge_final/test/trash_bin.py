#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

import sys

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_challenge_tester_trash_bin')

    W = client.SimWorld()

    freq = 20

    trash_x = 6.473
    trash_y = -3.972
    trash_vx = 0
    trash_vy = 0.1

    trash = W.add_object("trash_bin-1", "trash_bin", trash_x, trash_y, 0)
    
    while not rospy.is_shutdown():
            
        #while trash_x < 6.99:
        while trash_y < 0.5:
            trash.set_position(trash_x, trash_y, 0)
            rospy.sleep(1 / freq)
            trash_x += float(trash_vx) / freq
            trash_y += float(trash_vy) / freq

        #while trash_x > 6.473:
        while trash_y > -5:
            trash.set_position(trash_x, trash_y, 0)
            rospy.sleep(1 / freq)
            trash_x -= float(trash_vx) / freq
            trash_y -= float(trash_vy) / freq


