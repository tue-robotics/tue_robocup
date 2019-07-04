#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Janno Lunenburg

# ROS
import smach


class ReportPeople(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self._robot = robot

    def execute(self, ud):
        self._robot.speech.speak("Look at my screen to see where you're mates are at", mood="excited")

