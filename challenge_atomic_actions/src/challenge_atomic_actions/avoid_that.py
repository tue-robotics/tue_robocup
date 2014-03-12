#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_atomic_actions')
import rospy

import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from robot_skills.reasoner  import Conjunction, Compound

class AvoidThat(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))

        robot.reasoner.assertz(Compound("challenge", "basic_functionalities"))

        query_waypoint =  Compound("waypoint", "goal", Compound("pose_2d", "X", "Y", "Phi"))
        
        with self:

            smach.StateMachine.add('NAVIGATE_TO_WAYPOINT',
                                    states.NavigateGeneric(robot, goal_query=query_waypoint),
                                    transitions={   "arrived":"SAY_GOAL_REACHED",
                                                    "unreachable":'SAY_GOAL_UNREACHABLE',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'SAY_GOAL_NOT_DEFINED'})

            smach.StateMachine.add("FAILED_TO_START_CHALLENGE", 
                                    states.Say(robot, [ "Sorry, I can't start the challenge for some reason. Aborting now."]),
                                    transitions={   'spoken':'Aborted'})

            smach.StateMachine.add("SAY_GOAL_UNREACHABLE", 
                                    states.Say(robot, [ "Sorry, the goal is unreachable. Aborting now."]),
                                    transitions={   'spoken':'Aborted'})

            smach.StateMachine.add("SAY_GOAL_NOT_DEFINED", 
                                    states.Say(robot, [ "Sorry, I don't know where to go. Please specify my waypoint. Aborting now."]),
                                    transitions={   'spoken':'Aborted'})

            smach.StateMachine.add("SAY_GOAL_REACHED", 
                                    states.Say(robot, [ "Yeah, I reached my goal"]),
                                    transitions={   'spoken':'Done'})

if __name__ == "__main__":
    rospy.init_node('avoid_that_exec')
    states.util.startup(AvoidThat)
