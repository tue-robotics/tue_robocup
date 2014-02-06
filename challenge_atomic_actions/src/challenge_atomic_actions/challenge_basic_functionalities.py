#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_cleanup')
import rospy

import smach

from robot_smach_states import *
from challenge_atomic_actions import *

# ToDo: start at intermediate 

class ChallengeBasicFunctionalities(smach.StateMachine):
	def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:

            smach.StateMachine.add( "START_CHALLENGE",
                                    StartChallengeRobust(robot, "initial"), 
                                    transitions={   "Done":"GOTO_PICK_AND_PLACE", 
                                                    "Aborted":"Aborted", 
                                                    "Failed":"GOTO_PICK_AND_PLACE"})

            smach.StateMachine.add( 'GOTO_PICK_AND_PLACE',
                                    NavigateGeneric(robot, goal_query=query_exploration_target),
                                    transitions={   "arrived":"GOTO_FETCH_AND_CARRY",
                                                    "unreachable":'CANNOT_GOTO_CHALLENGE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            # ToDo: additional arguments?
            smach.StateMachine.add( 'PICK_AND_PLACE',
            	                    PickAndPlace(robot),
            	                    transitions={	"Done":		"GOTO_FETCH_AND_CARRY", 
            	                                    "Aborted":	"Aborted", 
            	                                    "Failed":	"GOTO_FETCH_AND_CARRY"})

            smach.StateMachine.add( 'GOTO_FETCH_AND_CARRY',
                                    NavigateGeneric(robot, goal_query=query_exploration_target),
                                    transitions={   "arrived":"GOTO_FIND_ME_AND_GO_OVER_THERE",
                                                    "unreachable":'CANNOT_GOTO_CHALLENGE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            smach.StateMachine.add( 'FETCH_AND_CARRY',
            	                    FetchAndCarry(robot),
            	                    transitions={	"Done":		"GOTO_FIND_ME_AND_GO_OVER_THERE", 
            	                                    "Aborted":	"Aborted", 
            	                                    "Failed":	"GOTO_FIND_ME_AND_GO_OVER_THERE"})

            smach.StateMachine.add( 'GOTO_FIND_ME_AND_GO_OVER_THERE',
                                    NavigateGeneric(robot, goal_query=query_exploration_target),
                                    transitions={   "arrived":"SAY_LOOK_FOR_OBJECTS",
                                                    "unreachable":'CANNOT_GOTO_CHALLENGE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            smach.StateMachine.add( 'FIND_ME_AND_GO_OVER_THERE',
            	                    FindMe(robot),
            	                    transitions={	"Done":		"GOTO_AVOID_THAT", 
            	                                    "Aborted":	"Aborted", 
            	                                    "Failed":	"GOTO_AVOID_THAT"})

            smach.StateMachine.add( 'GOTO_AVOID_THAT',
                                    NavigateGeneric(robot, goal_query=query_exploration_target),
                                    transitions={   "arrived":"SAY_LOOK_FOR_OBJECTS",
                                                    "unreachable":'CANNOT_GOTO_CHALLENGE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            smach.StateMachine.add( 'AVOID_THAT',
            	                    AvoidThat(robot),
            	                    transitions={	"Done":		"GOTO_WHAT_DID_YOU_SAY", 
            	                                    "Aborted":	"Aborted"})

            smach.StateMachine.add( 'GOTO_WHAT_DID_YOU_SAY',
                                    NavigateGeneric(robot, goal_query=query_exploration_target),
                                    transitions={   "arrived":"WHAT_DID_YOU_SAY",
                                                    "unreachable":'CANNOT_GOTO_CHALLENGE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            smach.StateMachine.add( 'WHAT_DID_YOU_SAY',
            	                    WhatDidYouSay(robot),
            	                    transitions={	"succeeded": "Done", 
            	                                    "failed":	"Done"})

            smach.StateMachine.add("CANNOT_GOTO_CHALLENGE", 
                                    Say(robot, [ "I can't find a way to my next challenge, can you please take me there "]),
                                    transitions={   'spoken':'Aborted'})

if __name__ == "__main__":
    rospy.init_node('basic_functionalities_exec')
    
    startup(ChallengeBasicFunctionalities)