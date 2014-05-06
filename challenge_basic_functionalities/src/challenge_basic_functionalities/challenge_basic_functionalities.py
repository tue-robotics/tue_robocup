#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_basic_functionalities')
import rospy
import sys

import smach

import avoid_that
#import fetch_and_carry
#import find_me
import pick_and_place
import what_did_you_say

from robot_smach_states import *
from robot_skills.amigo import Amigo


class Ask_continue(smach.State):
    def __init__(self, robot, tracking=True, rate=2):
        smach.State.__init__(self, outcomes=["done", "no_continue"])

        self.robot = robot
        self.preempted = False
        self.rate = rate
        self.ask_user_service_continue = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata):

        self.response = self.ask_user_service_continue("continue", 4 , rospy.Duration(180))

        for x in range(0,len(self.response.keys)):
            if self.response.keys[x] == "answer":
                if self.response.values[x] == "true":
                    return "done"
                else: 
                    return "no_continue"

        rospy.loginfo("answer was not found in response of interpreter. Should not happen!!")
        return "no_continue"

class HeadLookUp(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])

        self.robot = robot
        self.preempted = False

    def execute(self, userdata):

        self.robot.head.look_up(tilt_vel=0.75)
        return "done"

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
            	                    transitions={	"Done":		"SAY_GOTO_AVOID_THAT", 
            	                                    "Aborted":	"Aborted", 
            	                                    "Failed":	"SAY_GOTO_AVOID_THAT"})

            smach.StateMachine.add("SAY_GOTO_AVOID_THAT", 
                                    Say(robot, [ "I will go to the next task"], block=False),
                                    transitions={   'spoken':'GOTO_AVOID_THAT'})

            smach.StateMachine.add( 'GOTO_AVOID_THAT',
                                    NavigateGeneric(robot, goal_query=query_goto_avoid, goal_area_radius=0.4),
                                    transitions={   "arrived":"RESET_HEAD1",
                                                    "unreachable":'CANNOT_GOTO_CHALLENGE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            smach.StateMachine.add("RESET_HEAD1",
                                    HeadLookUp(robot),
                                    transitions={'done':'SAY_ASK_CONTINUE'})

            smach.StateMachine.add("SAY_ASK_CONTINUE", 
                                    Say(robot, [ "Please say continue."]),
                                    transitions={   'spoken':'ASK_CONTINUE'})

            smach.StateMachine.add("ASK_CONTINUE",
                                    Ask_continue(robot),
                                    transitions={   'done':'AVOID_THAT',
                                                    'no_continue':'AVOID_THAT'})

            smach.StateMachine.add( 'AVOID_THAT',
            	                    avoid_that.AvoidThat(robot),
            	                    transitions={	"Done":		"GOTO_WHAT_DID_YOU_SAY", 
            	                                    "Aborted":	"Aborted"})

            smach.StateMachine.add("SAY_GOTO_WHAT_DID_YOU_SAY", 
                                    Say(robot, [ "I will go to the next task"], block=False),
                                    transitions={   'spoken':'GOTO_WHAT_DID_YOU_SAY'})

            smach.StateMachine.add( 'GOTO_WHAT_DID_YOU_SAY',
                                    NavigateGeneric(robot, goal_query=query_goto_what, goal_area_radius=0.4),
                                    transitions={   "arrived":"RESET_HEAD2",
                                                    "unreachable":'SAY_ASK_CONTINUE2',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'SAY_ASK_CONTINUE2'})

            smach.StateMachine.add("RESET_HEAD2",
                                    HeadLookUp(robot),
                                    transitions={'done':'SAY_ASK_CONTINUE2'})

            smach.StateMachine.add("SAY_ASK_CONTINUE2", 
                                    Say(robot, [ "Please say continue."]),
                                    transitions={   'spoken':'ASK_CONTINUE2'})

            smach.StateMachine.add("ASK_CONTINUE2",
                                    Ask_continue(robot),
                                    transitions={   'done':'WHAT_DID_YOU_SAY',
                                                    'no_continue':'WHAT_DID_YOU_SAY'})

            smach.StateMachine.add( 'WHAT_DID_YOU_SAY',
            	                    what_did_you_say.WhatDidYouSay(robot),
            	                    transitions={	"Done": "FINISHED"})

            smach.StateMachine.add("CANNOT_GOTO_CHALLENGE", 
                                    Say(robot, [ "I can't find a way to my next challenge, can you please take me there "]),
                                    transitions={   'spoken':'Aborted'})

            smach.StateMachine.add("FINISHED", 
                                    Say(robot, [ "I finished this challenge"]),
                                    transitions={   'spoken':'Done'})

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
            amigo.reasoner.reset()
        elif int(sys.argv[1]) == 2:
            initial_state = ["AVOID_THAT"]
            amigo.reasoner.reset()
        elif int(sys.argv[1]) == 3:
            initial_state = ["WHAT_DID_YOU_SAY"]
            amigo.reasoner.reset()
        #initial_state = [str(sys.argv[1])]
        rospy.logwarn("Setting initial state to {0}, please make sure the reasoner is reset and the robot is localized correctly".format(initial_state))
        machine.set_initial_state(initial_state)

    try:
        machine.execute()
    except Exception, e:
        amigo.speech.speak(e)
