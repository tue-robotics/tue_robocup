#!/usr/bin/python

import rospy
import smach
import sys
import random
import math

from robot_smach_states.util.designators import *
import robot_smach_states as states
from robot_smach_states.util.startup import startup

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_final')
INITIAL_POSE = challenge_knowledge.initial_pose_sergio

class ExploreWaypoint(smach.StateMachine):
    def __init__(self, robot, waypoint):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        waypoint_designator = EdEntityDesignator(robot, id=waypoint)

        with self:
            smach.StateMachine.add("GOTO_WAYPOINT",
                                    states.NavigateToWaypoint(robot, waypoint_designator),
                                    transitions={   'arrived'                  :'succeeded',
                                                    'unreachable'              :'succeeded',
                                                    'goal_not_defined'         :'succeeded'})

            ''' Look at thing '''

            ''' Take snapshot '''

            ''' Look to side '''

            ''' Ask for entity '''

            ''' Assert entity (or in previous???) '''

############################## main statemachine ######################
def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:
    	smach.StateMachine.add("INITIALIZE",
    							states.StartChallengeRobust(robot, INITIAL_POSE, use_entry_points = False),
                                transitions={   "Done"              :   "EXPLORE1",
                                                "Aborted"           :   "EXPLORE1",
                                                "Failed"            :   "EXPLORE1"})

        smach.StateMachine.add("EXPLORE1",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_1),
                                transitions={   "succeeded"        :   "EXPLORE2",
                                                "failed"           :   "EXPLORE2"})

        smach.StateMachine.add("EXPLORE2",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_2),
                                transitions={   "succeeded"        :   "EXPLORE3",
                                                "failed"           :   "EXPLORE3"})

        smach.StateMachine.add("EXPLORE3",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_3),
                                transitions={   "succeeded"        :   "EXPLORE4",
                                                "failed"           :   "EXPLORE4"})

        smach.StateMachine.add("EXPLORE4",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_4),
                                transitions={   "succeeded"        :   "EXPLORE5",
                                                "failed"           :   "EXPLORE5"})

        smach.StateMachine.add("EXPLORE5",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_5),
                                transitions={   "succeeded"        :   "EXPLORE6",
                                                "failed"           :   "EXPLORE6"})

        smach.StateMachine.add("EXPLORE6",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_6),
                                transitions={   "succeeded"        :   "EXPLORE7",
                                                "failed"           :   "EXPLORE7"})

        smach.StateMachine.add("EXPLORE7",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_7),
                                transitions={   "succeeded"        :   "EXPLORE8",
                                                "failed"           :   "EXPLORE8"})

        smach.StateMachine.add("EXPLORE8",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_8),
                                transitions={   "succeeded"        :   "EXPLORE9",
                                                "failed"           :   "EXPLORE9"})

        smach.StateMachine.add("EXPLORE9",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_9),
                                transitions={   "succeeded"        :   "EXPLORE10",
                                                "failed"           :   "EXPLORE10"})

        smach.StateMachine.add("EXPLORE10",
                                ExploreWaypoint(robot, challenge_knowledge.explore_location_10),
                                transitions={   "succeeded"        :   "END_CHALLENGE",
                                                "failed"           :   "END_CHALLENGE"})

    	smach.StateMachine.add('END_CHALLENGE',
                                   states.Say(robot,"My work here is done, goodbye!"),
                                   transitions={'spoken':'Done'})

    return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('final_exec_sergio')

    startup(setup_statemachine, robot_name='sergio')