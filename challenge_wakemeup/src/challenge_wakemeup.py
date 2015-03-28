#!/usr/bin/python

"""This challenge is defined in https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/master/WakeMeUp.tex

In short, the robot must go the the homeowners bedroom, wake him up gently and ask what he want for breakfast.
Then of course fetch that breakfast, bring it to the homeowner and optionally make the bed.
"""

import rospy
import smach
import sys
import random

from robot_smach_states.util.designators import EdEntityDesignator
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states import Grab
from robot_smach_states import Place
from robot_skills.util import msg_constructors as geom
from robot_skills.util import transformations

ROOM = "room_bedroom"

class WakeMeUp(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        homeowner = EdEntityDesignator(robot, type='human')

        with self:
            smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'INTRO',
                                                'abort':'Aborted'})

            smach.StateMachine.add( "INTRO",
                                    states.Say(robot, ["I will wake you up!"]), 
                                    transitions={"spoken":"GOTO_GRANNY"})

            smach.StateMachine.add( "GOTO_BEDROOM",
                                    #states.NavigateToPose(robot, 0, 0, 0),
                                    states.NavigateToSymbolic(robot, { homeowner:"near", EdEntityDesignator(robot, id=ROOM) : "in"}, homeowner),
                                    transitions={   'arrived'           :'SAY_GOODMORNING',
                                                    'unreachable'       :'SAY_GOODMORNING',
                                                    'goal_not_defined'  :'SAY_GOODMORNING'})

            smach.StateMachine.add( "SAY_GOODMORNING",
                                    states.Say(robot, ["Are you awake?"]),
                                    transitions={   'spoken'            :'Done'})


############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('wakemeup_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE MANIPULATION] Please provide robot name as argument."
        exit(1)

    startup(WakeMeUp, robot_name=robot_name)
