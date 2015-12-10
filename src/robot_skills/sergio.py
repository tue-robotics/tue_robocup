#! /usr/bin/env python
import robot
import rospy


class Sergio(robot.Robot):
    """docstring for Sergio"""
    def __init__(self, wait_services=False):
        super(Sergio, self).__init__(robot_name="sergio", wait_services=False)
