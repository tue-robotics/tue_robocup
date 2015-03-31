#!/usr/bin/python

"""This challenge is defined in https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/master/WakeMeUp.tex

In short, the robot must go the the homeowners bedroom, wake him up gently and ask what he want for breakfast.
Then of course fetch that breakfast, bring it to the homeowner and optionally make the bed.
"""

import rospy
import smach
import sys
import random

from robot_smach_states.util.designators import Designator, VariableDesignator, EdEntityDesignator
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states import Grab
from robot_smach_states import Place
from robot_skills.util import msg_constructors as geom
from robot_skills.util import transformations
from dragonfly_speech_recognition.srv import GetSpeechResponse

ROOM = "room_bedroom"

class WakeMeUp(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])


        def is_awake(entity):
            #Check that the operator is awake
            return True
        homeowner = EdEntityDesignator(robot, type='human', criteriafuncs=[is_awake])
        bed = EdEntityDesignator(robot, type='bed')

        # TODO
        spec = Designator("I want <fruit_snack> with <cereal> and <milk> for breakfast")
        choices = Designator({  "fruit_snack"  : ["apple" ], "cereal" : ["cereal", "choco-flakes"], "milk": ["whole-milk"]})
        answer = VariableDesignator(resolve_type=GetSpeechResponse) #???

        with self:
            smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'INTRO',
                                                'abort':'Aborted'})

            smach.StateMachine.add( "INTRO",
                                    states.Say(robot, ["I will wake you up!"]), 
                                    transitions={"spoken":"GOTO_BEDROOM"})

            smach.StateMachine.add( "GOTO_BEDROOM",
                                    #states.NavigateToPose(robot, 0, 0, 0),
                                    states.NavigateToSymbolic(robot, { bed:"near", EdEntityDesignator(robot, id=ROOM) : "in"}, bed),
                                    transitions={   'arrived'           :'SAY_GOODMORNING',
                                                    'unreachable'       :'SAY_GOODMORNING',
                                                    'goal_not_defined'  :'SAY_GOODMORNING'})

            smach.StateMachine.add( "SAY_GOODMORNING",
                                    states.Say(robot, ["Are you awake?", "Rise and shine!", "Wakey wakey!", "Hello there sleepy head!", "Time for breakfast!"]),
                                    transitions={   'spoken'            :'AWAIT_HUMAN_AWAKE'})

            #TODO: Add concurrence to play music to wake someone up and monitor whether the dude is awake
            smach.StateMachine.add( "AWAIT_HUMAN_AWAKE",
                                    states.WaitForDesignator(robot, homeowner, attempts=20, sleep_interval=3),  # Wait 60 seconds
                                    transitions={   'success'            :'ASK_ABOUT_BREAKFAST',
                                                    'failed'             :'ASK_ABOUT_BREAKFAST'})

            smach.StateMachine.add( "ASK_ABOUT_BREAKFAST",
                                    states.HearOptionsExtra(robot, spec, choices, answer),
                                    transitions={   'heard'            :'Done',
                                                    'no_result'        :'Aborted'})

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('wakemeup_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE MANIPULATION] Please provide robot name as argument."
        exit(1)

    startup(WakeMeUp, robot_name=robot_name)
