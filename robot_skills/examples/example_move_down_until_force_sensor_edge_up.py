#!/usr/bin/env python
import rospy

from robot_skills.get_robot import get_robot

rospy.init_node('test_mode_down_until_force_sensor_edge_up', anonymous=True)
robot = get_robot("hero")
arm = robot.get_arm(force_sensor_required=True)

arm.move_down_until_force_sensor_edge_up()
