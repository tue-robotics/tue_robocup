#! /usr/bin/env python
import rospy 
import smach

import random

class FlashLights(smach.State):
    def __init__(self, robot, duration=10):
        smach.State.__init__(self, outcomes=["Done"])
        self.robot = robot
        self.duration = duration

    def execute(self, userdata=None):
        l = self.robot.lights
        start = rospy.Time.now()
        end = start+rospy.Duration(self.duration)

        while rospy.Time.now() < end:
            color = (random.random(),random.random(),random.random())

            l.set_color(*color)

        l.set_color(0,0,1)
        return "Done"
