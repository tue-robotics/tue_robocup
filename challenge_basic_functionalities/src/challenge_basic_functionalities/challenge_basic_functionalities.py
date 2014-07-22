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

import smach_ros # added for using smach_viewer

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

        query_goto_pick            = Compound("waypoint", "pick_loc", Compound("pose_2d", "X", "Y", "Phi"))
        query_goto_ready_for_avoid = Compound("waypoint", "ready_for_avoid_loc", Compound("pose_2d", "X", "Y", "Phi"))
        query_goto_avoid           = Compound("waypoint", "avoid_loc", Compound("pose_2d", "X", "Y", "Phi"))
        query_goto_what            = Compound("waypoint", "what_loc", Compound("pose_2d", "X", "Y", "Phi"))

        with self:

            smach.StateMachine.add( "START_CHALLENGE",
                                    StartChallengeRobust(robot, "initial_exit"),
                                    transitions={   "Done":"GOTO_PICK_AND_PLACE", 
                                                    "Aborted":"Aborted", 
                                                    "Failed":"GOTO_PICK_AND_PLACE"})

            smach.StateMachine.add( 'GOTO_PICK_AND_PLACE',
                                    NavigateGeneric(robot, goal_query=query_goto_pick, goal_area_radius=0.2),
                                    transitions={   "arrived":"PICK_AND_PLACE",
                                                    "unreachable":'SAY_PLEASE_MOVE_1',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'SAY_PLEASE_MOVE_1'})

            smach.StateMachine.add("SAY_PLEASE_MOVE_1", 
                                    Say(robot, [ "I am not able to go to the next part of the challenge, please clear the way"], block=True),
                                    transitions={   'spoken':'GIVE_TIME_TO_UPDATE_COSTMAP_1'})

            smach.StateMachine.add( 'GIVE_TIME_TO_UPDATE_COSTMAP_1',
                                    states.Wait_time(robot, waittime=3),
                                    transitions={   'waited':'GOTO_PICK_AND_PLACE_2ND_TRY',
                                                    'preempted':'GOTO_PICK_AND_PLACE_2ND_TRY'})

            smach.StateMachine.add( 'GOTO_PICK_AND_PLACE_2ND_TRY',
                                    NavigateGeneric(robot, goal_query=query_goto_pick, goal_area_radius=0.2),
                                    transitions={   "arrived":"PICK_AND_PLACE",
                                                    "unreachable":'CANNOT_GOTO_CHALLENGE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            smach.StateMachine.add("RESET_HEAD_FOR_PICK_AND_PLACE", # THIS STATE IS USED IF challenge is started with "rosrun challenge_basic.. challe...py 1"
                                    HeadLookUp(robot),
                                    transitions={'done':'SAY_ASK_CONTINUE_0'})

            smach.StateMachine.add("SAY_ASK_CONTINUE_0", 
                                    Say(robot, [ "Please say continue when I can start"]),
                                    transitions={   'spoken':'ASK_CONTINUE_0'})

            smach.StateMachine.add("ASK_CONTINUE_0",
                                    Ask_continue(robot),
                                    transitions={   'done':'PICK_AND_PLACE',
                                                    'no_continue':'PICK_AND_PLACE'})

            # ToDo: additional arguments?
            smach.StateMachine.add( 'PICK_AND_PLACE',
            	                    pick_and_place.PickAndPlace(robot),
            	                    transitions={	"Done":		"READY_FOR_GOTO_AVOID_THAT", 
            	                                    "Aborted":	"Aborted", 
            	                                    "Failed":	"READY_FOR_GOTO_AVOID_THAT"})


            smach.StateMachine.add( 'READY_FOR_GOTO_AVOID_THAT',
                                    NavigateGeneric(robot, goal_query=query_goto_ready_for_avoid, goal_area_radius=0.4),
                                    transitions={   "arrived":"READY_FOR_RESET_HEAD_FOR_AVOID_THAT",
                                                    "unreachable":'READY_FOR_SAY_PLEASE_MOVE_2',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'READY_FOR_SAY_PLEASE_MOVE_2'})

            smach.StateMachine.add("READY_FOR_SAY_PLEASE_MOVE_2", 
                                    Say(robot, [ "I am not able to go to the next part of the challenge, please clear the way"], block=True),
                                    transitions={   'spoken':'READY_FOR_GIVE_TIME_TO_UPDATE_COSTMAP_2'})

            smach.StateMachine.add( 'READY_FOR_GIVE_TIME_TO_UPDATE_COSTMAP_2',
                                    states.Wait_time(robot, waittime=3),
                                    transitions={   'waited':'READY_FOR_GOTO_AVOID_THAT_2ND_TRY',
                                                    'preempted':'READY_FOR_GOTO_AVOID_THAT_2ND_TRY'})

            smach.StateMachine.add( 'READY_FOR_GOTO_AVOID_THAT_2ND_TRY',
                                    NavigateGeneric(robot, goal_query=query_goto_ready_for_avoid, goal_area_radius=0.4),
                                    transitions={   "arrived":"READY_FOR_RESET_HEAD_FOR_AVOID_THAT",
                                                    "unreachable":'CANNOT_GOTO_CHALLENGE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            smach.StateMachine.add("READY_FOR_RESET_HEAD_FOR_AVOID_THAT",
                                    HeadLookUp(robot),
                                    transitions={'done':'SAY_ASK_CONTINUE'})

            smach.StateMachine.add("SAY_ASK_CONTINUE", 
                                    Say(robot, [ "Please say continue when I can go to the next task"]),
                                    transitions={   'spoken':'ASK_CONTINUE'})

            smach.StateMachine.add("ASK_CONTINUE",
                                    Ask_continue(robot),
                                    transitions={   'done':'SAY_GOTO_AVOID_THAT',
                                                    'no_continue':'SAY_GOTO_AVOID_THAT'})

            smach.StateMachine.add("SAY_GOTO_AVOID_THAT", 
                                    Say(robot, [ "I will go to the next task"], block=False),
                                    transitions={   'spoken':'GOTO_AVOID_THAT'})

            smach.StateMachine.add( 'GOTO_AVOID_THAT',
                                    NavigateGeneric(robot, goal_query=query_goto_avoid, goal_area_radius=0.4),
                                    transitions={   "arrived":"AVOID_THAT",
                                                    "unreachable":'SAY_PLEASE_MOVE_2',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'SAY_PLEASE_MOVE_2'})

            smach.StateMachine.add("SAY_PLEASE_MOVE_2", 
                                    Say(robot, [ "I am not able to go to the next part of the challenge, please clear the way"], block=True),
                                    transitions={   'spoken':'GIVE_TIME_TO_UPDATE_COSTMAP_2'})

            smach.StateMachine.add( 'GIVE_TIME_TO_UPDATE_COSTMAP_2',
                                    states.Wait_time(robot, waittime=3),
                                    transitions={   'waited':'GOTO_AVOID_THAT_2ND_TRY',
                                                    'preempted':'GOTO_AVOID_THAT_2ND_TRY'})

            smach.StateMachine.add( 'GOTO_AVOID_THAT_2ND_TRY',
                                    NavigateGeneric(robot, goal_query=query_goto_avoid, goal_area_radius=0.4),
                                    transitions={   "arrived":"AVOID_THAT",
                                                    "unreachable":'CANNOT_GOTO_CHALLENGE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'CANNOT_GOTO_CHALLENGE'})

            smach.StateMachine.add( 'AVOID_THAT',
            	                    avoid_that.AvoidThat(robot),
            	                    transitions={	"Done":		"RESET_HEAD_FOR_WHAT_DID_YOU_SAY", 
            	                                    "Aborted":	"Aborted"})

            smach.StateMachine.add("RESET_HEAD_FOR_WHAT_DID_YOU_SAY",
                                    HeadLookUp(robot),
                                    transitions={'done':'SAY_ASK_CONTINUE2'})

            smach.StateMachine.add("SAY_ASK_CONTINUE2", 
                                    Say(robot, [ "Please say continue when I can go to the next task"]),
                                    transitions={   'spoken':'ASK_CONTINUE2'})

            smach.StateMachine.add("ASK_CONTINUE2",
                                    Ask_continue(robot),
                                    transitions={   'done':'SAY_GOTO_WHAT_DID_YOU_SAY',
                                                    'no_continue':'SAY_GOTO_WHAT_DID_YOU_SAY'})

            smach.StateMachine.add("SAY_GOTO_WHAT_DID_YOU_SAY", 
                                    Say(robot, [ "I will go to the next task"], block=False),
                                    transitions={   'spoken':'GOTO_WHAT_DID_YOU_SAY'})

            smach.StateMachine.add( 'GOTO_WHAT_DID_YOU_SAY',
                                    NavigateGeneric(robot, goal_query=query_goto_what, goal_area_radius=0.4),
                                    transitions={   "arrived":"WHAT_DID_YOU_SAY",
                                                    "unreachable":'SAY_PLEASE_MOVE_3',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'SAY_PLEASE_MOVE_3'})

            smach.StateMachine.add("SAY_PLEASE_MOVE_3", 
                                    Say(robot, [ "I am not able to go to the next part of the challenge, please clear the way"], block=True),
                                    transitions={   'spoken':'GIVE_TIME_TO_UPDATE_COSTMAP_3'})

            smach.StateMachine.add( 'GIVE_TIME_TO_UPDATE_COSTMAP_3',
                                    states.Wait_time(robot, waittime=3),
                                    transitions={   'waited':'GOTO_WHAT_DID_YOU_SAY_2ND_TRY',
                                                    'preempted':'GOTO_WHAT_DID_YOU_SAY_2ND_TRY'})


            smach.StateMachine.add( 'GOTO_WHAT_DID_YOU_SAY_2ND_TRY',
                                    NavigateGeneric(robot, goal_query=query_goto_what, goal_area_radius=0.4),
                                    transitions={   "arrived":"WHAT_DID_YOU_SAY",
                                                    "unreachable":'WHAT_DID_YOU_SAY',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'WHAT_DID_YOU_SAY'})

            smach.StateMachine.add( 'WHAT_DID_YOU_SAY',
            	                    what_did_you_say.WhatDidYouSay(robot),
            	                    transitions={	"Done": "SAY_GO_TO_EXIT"})

            smach.StateMachine.add("CANNOT_GOTO_CHALLENGE", 
                                    Say(robot, [ "I can't find a way to my next challenge, can you please take me there "]),
                                    transitions={   'spoken':'Aborted'})

            smach.StateMachine.add("SAY_GO_TO_EXIT", 
                                    Say(robot, [ "I will now go to the exit"]),
                                    transitions={   'spoken':'GO_TO_FRONT_OF_EXIT'})

            navigate_exit_3 = Conjunction(  Compound("=", "Waypoint", Compound("exit", "c")),
                                            Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))

            smach.StateMachine.add('GO_TO_FRONT_OF_EXIT', 
                                    states.NavigateGeneric(robot, goal_query=navigate_exit_3),
                                    transitions={   'arrived':'GO_TO_EXIT_1', 
                                                    'preempted':'GO_TO_EXIT_1', 
                                                    'unreachable':'GO_TO_EXIT_1', 
                                                    'goal_not_defined':'GO_TO_EXIT_1'})

            navigate_exit_1 = Conjunction(  Compound("=", "Waypoint", Compound("exit", "a")),
                                            Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))
            
            smach.StateMachine.add('GO_TO_EXIT_1', 
                                    states.NavigateGeneric(robot, goal_query=navigate_exit_1),
                                    transitions={   'arrived':'AT_EXIT', 
                                                    'preempted':'GO_TO_EXIT_2', 
                                                    'unreachable':'GO_TO_EXIT_2', 
                                                    'goal_not_defined':'GO_TO_EXIT_2'})

            navigate_exit_2 = Conjunction(  Compound("=", "Waypoint", Compound("exit", "b")),
                                            Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))

            smach.StateMachine.add('GO_TO_EXIT_2', 
                                    states.NavigateGeneric(robot, goal_query=navigate_exit_2),
                                    transitions={   'arrived':'AT_EXIT', 
                                                    'preempted':'GO_TO_EXIT_3', 
                                                    'unreachable':'GO_TO_EXIT_3', 
                                                    'goal_not_defined':'GO_TO_EXIT_3'})

            smach.StateMachine.add('GO_TO_EXIT_3', 
                                    states.NavigateGeneric(robot, goal_query=navigate_exit_3),
                                    transitions={   'arrived':'GO_TO_EXIT_1_LAST_ATTEMPT', 
                                                    'preempted':'GO_TO_EXIT_1_LAST_ATTEMPT', 
                                                    'unreachable':'GO_TO_EXIT_1_LAST_ATTEMPT', 
                                                    'goal_not_defined':'GO_TO_EXIT_1_LAST_ATTEMPT'})

            smach.StateMachine.add('GO_TO_EXIT_1_LAST_ATTEMPT', 
                                    states.NavigateGeneric(robot, goal_query=navigate_exit_1),
                                    transitions={   'arrived':'AT_EXIT', 
                                                    'preempted':'AT_EXIT', 
                                                    'unreachable':'AT_EXIT', 
                                                    'goal_not_defined':'AT_EXIT'})

            smach.StateMachine.add("NOT_AT_EXIT",
                                   states.Say(robot,"I was not able to reach the exit, I am sorry. Goodbye!"),
                                   transitions={'spoken':'Done'})

            smach.StateMachine.add("AT_EXIT",
                                   states.Say(robot,"I finished this challenge, goodbye!"),
                                   transitions={'spoken':'Done'})


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

    ''' If necessary: set initial state '''
    rospy.loginfo("Sys.argv = {0}, Length = {1}".format(sys.argv,len(sys.argv)))
    if  len(sys.argv) > 1:
        if int(sys.argv[1]) == 1:
            initial_state = ["RESET_HEAD_FOR_PICK_AND_PLACE"]
            amigo.reasoner.reset()
        elif int(sys.argv[1]) == 2:
            initial_state = ["READY_FOR_RESET_HEAD_FOR_AVOID_THAT"]
            amigo.reasoner.reset()
        elif int(sys.argv[1]) == 3:
            initial_state = ["RESET_HEAD_FOR_WHAT_DID_YOU_SAY"]
            amigo.reasoner.reset()

    ''' Setup state machine'''
    machine = ChallengeBasicFunctionalities(amigo)
    if  len(sys.argv) > 1:
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