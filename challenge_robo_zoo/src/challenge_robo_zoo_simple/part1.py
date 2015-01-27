#! /usr/bin/env python
import smach

class TurnAround(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot

    def execute(self, userdata=None):
        b = self.robot.base
        #b.force_drive(0.25, 0, 0, 3)
        b.force_drive(0, 0, 0.5, 12) #turn yourself around
        #b.force_drive(-0.25, 0, 0, 3)
        return "Done"