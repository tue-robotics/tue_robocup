#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_cleanup')
import rospy
import sys

import smach

import avoid_that
import fetch_and_carry
import find_me
import pick_and_place
import what_did_you_say

from robot_smach_states import *
from robot_skills.amigo import Amigo

# ToDo: start at intermediate 

class ChallengeBasicFunctionalities(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        query_goto_pick   = Compound("waypoint", "pick_loc", Compound("pose_2d", "X", "Y", "Phi"))
        query_goto_fetch  = Compound("waypoint", "fetch_loc", Compound("pose_2d", "X", "Y", "Phi"))
        query_goto_find   = Compound("waypoint", "find_loc", Compound("pose_2d", "X", "Y", "Phi"))
        query_goto_avoid  = Compound("waypoint", "avoid_loc", Compound("pose_2d", "X", "Y", "Phi"))
        query_goto_what   = Compound("waypoint", "what_loc", Compound("pose_2d", "X", "Y", "Phi"))

        with self:

            smach.StateMachine.add( "START_CHALLENGE",
                                    StartChallengeRobust(robot, "initial"), 
                                    transitions={   "Done":"GOTO_PICK_AND_PLACE", 
                                                    "Aborted":"Aborted", 
                                                    "Failed":"GOTO_PICK_AND_PLACE"})

            smach.StateMachine.add( 'GOTO_PICK_AND_PLACE',
                                    NavigateGeneric(robot, goal_query=query_goto_pick, goal_area_radius=0.2),
                                    transitions={   "arrived":"PICK_AND_PLACE",
                                                    "unreachable":'CANNOT_GOTO_CHALLENGE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            # ToDo: additional arguments?
            smach.StateMachine.add( 'PICK_AND_PLACE',
            	                    pick_and_place.PickAndPlace(robot),
            	                    transitions={	"Done":		"GOTO_AVOID_THAT", 
            	                                    "Aborted":	"Aborted", 
            	                                    "Failed":	"GOTO_AVOID_THAT"})

            #smach.StateMachine.add( 'GOTO_FETCH_AND_CARRY',
            #                        NavigateGeneric(robot, goal_query=query_goto_fetch, goal_area_radius=0.2),
            #                        transitions={   "arrived":"GOTO_FIND_ME_AND_GO_OVER_THERE",
            #                                        "unreachable":'CANNOT_GOTO_CHALLENGE',
            #                                        "preempted":'Aborted',
            #                                        "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            #smach.StateMachine.add( 'FETCH_AND_CARRY',
            #	                    fetch_and_carry.FetchAndCarry(robot),
            #	                    transitions={	"Done":		"GOTO_FIND_ME_AND_GO_OVER_THERE", 
            #	                                    "Aborted":	"Aborted", 
            #	                                    "Failed":	"GOTO_FIND_ME_AND_GO_OVER_THERE"})

            #smach.StateMachine.add( 'GOTO_FIND_ME_AND_GO_OVER_THERE',
            #                        NavigateGeneric(robot, goal_query=query_goto_find, goal_area_radius=0.2),
            #                        transitions={   "arrived":"SAY_LOOK_FOR_OBJECTS",
            #                                       "unreachable":'CANNOT_GOTO_CHALLENGE',
            #                                       "preempted":'Aborted',
            #                                       "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            #smach.StateMachine.add( 'FIND_ME_AND_GO_OVER_THERE',
            #	                    find_me.FindMe(robot),
            #	                    transitions={	"Done":		"GOTO_AVOID_THAT", 
            #	                                    "Aborted":	"Aborted", 
            #	                                    "Failed":	"GOTO_AVOID_THAT"})

            smach.StateMachine.add( 'GOTO_AVOID_THAT',
                                    NavigateGeneric(robot, goal_query=query_goto_avoid, goal_area_radius=0.2),
                                    transitions={   "arrived":"AVOID_THAT",
                                                    "unreachable":'CANNOT_GOTO_CHALLENGE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            smach.StateMachine.add( 'AVOID_THAT',
            	                    avoid_that.AvoidThat(robot),
            	                    transitions={	"Done":		"GOTO_WHAT_DID_YOU_SAY", 
            	                                    "Aborted":	"Aborted"})

            smach.StateMachine.add( 'GOTO_WHAT_DID_YOU_SAY',
                                    NavigateGeneric(robot, goal_query=query_goto_what, goal_area_radius=0.2),
                                    transitions={   "arrived":"WHAT_DID_YOU_SAY",
                                                    "unreachable":'CANNOT_GOTO_CHALLENGE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            smach.StateMachine.add( 'WHAT_DID_YOU_SAY',
            	                    what_did_you_say.WhatDidYouSay(robot),
            	                    transitions={	"Done": "GOTO_PICK_AND_PLACE"})

            smach.StateMachine.add("CANNOT_GOTO_CHALLENGE", 
                                    Say(robot, [ "I can't find a way to my next challenge, can you please take me there "]),
                                    transitions={   'spoken':'Aborted'})

if __name__ == "__main__":
    rospy.init_node('basic_functionalities_exec')
 
    amigo = Amigo(wait_services=True)

    # Retract all old facts
    amigo.reasoner.query(Compound("retractall", Compound("challenge", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("goal", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("explored", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
    amigo.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("current_object", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("current_person", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("visited", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("registered", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("type", "X", "Y")))

    # Load locations and objects from knowledge files
    amigo.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
    amigo.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))

    # Assert current challenge
    amigo.reasoner.assertz(Compound("challenge", "basic_functionalities"))

    ''' Setup state machine'''
    machine = ChallengeBasicFunctionalities(amigo)

    ''' If necessary: set initial state '''
    rospy.loginfo("Sys.argv = {0}, Length = {1}".format(sys.argv,len(sys.argv)))
    if  len(sys.argv) > 1:
        if int(sys.argv[1]) == 1:
            initial_state = ["PICK_AND_PLACE"]
        elif int(sys.argv[1]) == 2:
            initial_state = ["AVOID_THAT"]
        elif int(sys.argv[1]) == 3:
            initial_state = ["WHAT_DID_YOU_SAY"]
        #initial_state = [str(sys.argv[1])]
        rospy.logwarn("Setting initial state to {0}, please make sure the reasoner is reset and the robot is localized correctly".format(initial_state))
        machine.set_initial_state(initial_state)

    try:
        machine.execute()
    except Exception, e:
        amigo.speech.speak(e)