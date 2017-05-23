#! /usr/bin/env python
import rospy
import smach

from human_interaction import Say

class CheckEButton(smach.State):
    """Check if the robot's Emergency button is pressed"""
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=["pressed",
                                             "released"])
        self.robot = robot

    def execute(self, userdata=None):
        if self.robot.ebutton.read_ebutton():
            return "pressed"
        else:
            return "released"

class NotifyEButton(smach.StateMachine):
    """Alert the operator that the robot's Emergency button is still pressed"""

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["succeeded"])
        assert hasattr(robot, "ebutton")
        assert hasattr(robot, "speech")

        with self:
            smach.StateMachine.add( "CHECK_EBUTTON",
                                    CheckEButton(robot),
                                    transitions={   "pressed"   :"SAY_PRESSED",
                                                    "released"  :"succeeded"})

            smach.StateMachine.add("SAY_PRESSED",
                                    Say(robot, ["My Emergency button is still pressed", "E-Button still pressed"], block=False),
                                    transitions={   "spoken":"succeeded"})
