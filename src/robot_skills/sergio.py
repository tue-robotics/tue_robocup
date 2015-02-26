#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import robot

import rospy

class Sergio(robot.Robot):
    """docstring for Sergio"""
    def __init__(self, wait_services=False):
        super(Sergio, self).__init__(robot_name="sergio", wait_services=False)
