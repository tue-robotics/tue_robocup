#! /usr/bin/env python

import rospy
from robot_smach_states.state import State

# ----------------------------------------------------------------------------------------------------

class SetPlugins(State):
    def __init__(self, robot, enable=None, disable=None):
        State.__init__(self, locals(), outcomes=["done"])

    def run(self, robot, enable, disable):
        if disable:  
            robot.ed.disable_plugins(disable)

        if enable:
            robot.ed.enable_plugins(enable)

        return 'done'