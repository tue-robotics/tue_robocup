#! /usr/bin/env python

import rospy
import smach

# ----------------------------------------------------------------------------------------------------

class SetPlugins(smach.State):
    def __init__(self, robot, enable=None, disable=None):
        smach.State.__init__(self, outcomes=["done"])
        self.enable = enable
        self.disable = disable
        self.robot = robot

    def execute(self, userdata=None):
        if self.disable:  
            robot.ed.disable_plugins(self.disable)

        if self.enable:
            robot.ed.enable_plugins(self.enable)

        return 'done'



'''
	Initialize world model with a certain configuration.
	Set perception mode to non-continuos and disable laser_integration.
'''
class InitializeWorldModel(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot

    def execute(self, userdata=None):
    	# TODO: the state was crashing when calling these arguments,
    	# 		ASK SJOERD HOW TO USE THIS NOW!
        # self.robot.ed.configure_kinect_segmentation(continuous=False)
        # self.robot.ed.configure_perception(continuous=False)
        # self.robot.ed.disable_plugins(plugin_names=["laser_integration"])
        self.robot.ed.reset()

        return 'done'

