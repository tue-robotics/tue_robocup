#! /usr/bin/env python
import rospy
import sys

import avoid_that
import pick_and_place
import what_did_you_say

from robot_smach_states.highlevel import StartChallengeRobust
from robot_smach_states.navigation import NavigateToObserve
from robot_smach_states.navigation import NavigateToWaypoint
from robot_smach_states.human_interaction import Say

from robot_skills.amigo import Amigo
from robot_skills.sergio import Sergio

from robot_smach_states.designators.designator import Designator, VariableDesignator

import smach
import smach_ros

# ----------------------------------------------------------------------------------------------------

PICK_AND_PLACE_TABLE_ID="dinner_table"

# ----------------------------------------------------------------------------------------------------

class ChallengeBasicFunctionalities(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:

            smach.StateMachine.add( "START_CHALLENGE",
                                    StartChallengeRobust(robot, "initial_pose"),
                                    transitions={   "Done":"INIT_PICK_AND_PLACE",
                                                    "Aborted":"Aborted",
                                                    "Failed":"INIT_PICK_AND_PLACE"})

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                            PICK AND PLACE
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            smach.StateMachine.add( 'INIT_PICK_AND_PLACE',
                                    Say(robot, [ "I will go to the first task: pick and place!"], block=False),
                                    transitions={   'spoken':'GOTO_PICK_AND_PLACE'})

            smach.StateMachine.add( 'GOTO_PICK_AND_PLACE',
                                    NavigateToObserve(robot, Designator(PICK_AND_PLACE_TABLE_ID), radius=0.7),
                                    transitions={   "arrived":"PICK_AND_PLACE",
                                                    "unreachable":'Aborted',
                                                    "goal_not_defined":'Aborted'})

            smach.StateMachine.add( 'PICK_AND_PLACE',
            	                    pick_and_place.PickAndPlace(robot),
            	                    transitions={	"Done":		"INIT_AVOID_THAT",
            	                                    "Aborted":	"Aborted",
            	                                    "Failed":	"INIT_AVOID_THAT"})

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                             AVOID THAT
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            smach.StateMachine.add( 'INIT_AVOID_THAT',
                                    Say(robot, [ "I will go to the second task: avoid that!"], block=False),
                                    transitions={   'spoken':'GOTO_AVOID_THAT'})

            smach.StateMachine.add( 'GOTO_AVOID_THAT',
                                    NavigateToWaypoint(robot, Designator("avoid_that_start")),
                                    transitions={   "arrived":"AVOID_THAT",
                                                    "unreachable":'Aborted',
                                                    "goal_not_defined":'Aborted'})

            smach.StateMachine.add( 'AVOID_THAT',
                                    avoid_that.AvoidThat(robot),
                                    transitions={   "Done":     "INIT_WHAT_DID_YOU_SAY",
                                                    "Aborted":  "Aborted"})

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                          WHAT DID YOU SAY
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            smach.StateMachine.add( 'INIT_WHAT_DID_YOU_SAY',
                                    Say(robot, [ "I will go to the last task: what did you say!"], block=False),
                                    transitions={   'spoken':'GOTO_WHAT_DID_YOU_SAY'})

            smach.StateMachine.add( 'GOTO_WHAT_DID_YOU_SAY',
                                    NavigateToWaypoint(robot, Designator("what_did_you_say_start")),
                                    transitions={   "arrived":"WHAT_DID_YOU_SAY",
                                                    "unreachable":'Aborted',
                                                    "goal_not_defined":'Aborted'})

            smach.StateMachine.add( 'WHAT_DID_YOU_SAY',
            	                    what_did_you_say.WhatDidYouSay(robot),
            	                    transitions={	"Done" : "SAY_GO_TO_EXIT",
                                                    "Aborted" : "SAY_GO_TO_EXIT",
                                                    "Failed" : "SAY_GO_TO_EXIT"})

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                                 EXIT
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            smach.StateMachine.add("SAY_GO_TO_EXIT",
                                    Say(robot, [ "I will now go to the exit"]),
                                    transitions={   'spoken':'GO_TO_EXIT'})

            smach.StateMachine.add('GO_TO_EXIT',
                                    NavigateToWaypoint(robot, Designator("exit")),
                                    transitions={   "arrived":"END_CHALLENGE",
                                                    "unreachable":'Aborted',
                                                    "goal_not_defined":'Aborted'})

            smach.StateMachine.add("END_CHALLENGE",
                                   Say(robot,"I finished this challenge, goodbye!"),
                                   transitions={'spoken':'Done'})

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
if __name__ == "__main__":
    rospy.init_node('basic_functionalities_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE BASIC FUNCTIONALITIES] Please provide robot name as argument."
        exit(1)

    if robot_name == 'amigo':
        robot = Amigo(wait_services=True)
    elif robot_name == 'sergio':
        robot = Sergio(wait_services=True)
    else:
        print "[CHALLENGE BASIC FUNCTIONALITIES] Don't know robot name " + robot_name

    ''' If necessary: set initial state '''
    rospy.loginfo("Sys.argv = {0}, Length = {1}".format(sys.argv,len(sys.argv)))
    if  len(sys.argv) > 2:
        if int(sys.argv[2]) == 1:
            initial_state = ["INIT_PICK_AND_PLACE"]
        elif int(sys.argv[2]) == 2:
            initial_state = ["INIT_AVOID_THAT"]
        elif int(sys.argv[2]) == 3:
            initial_state = ["INIT_WHAT_DID_YOU_SAY"]

    ''' Setup state machine'''
    machine = ChallengeBasicFunctionalities(robot)
    if  len(sys.argv) > 2:
        #initial_state = [str(sys.argv[1])]
        rospy.logwarn("Setting initial state to {0}, please make sure the reasoner is reset and the robot is localized correctly".format(initial_state))
        machine.set_initial_state(initial_state)

    # for using smach viewer
    introserver = smach_ros.IntrospectionServer('server_name', machine, '/SM_ROOT_PRIMARY')
    introserver.start()

    try:
        machine.execute()
    except Exception, e:
        amigo.speech.speak(e)

    introserver.stop()
