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

############################## main statemachine ######################
def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:
    	smach.StateMachine.add("INITIALIZE",
    							states.StartChallengeRobust(robot, INITIAL_POSE, use_entry_points = True),
                                transitions={   "Done"              :   "Done",
                                                "Aborted"           :   "Done",
                                                "Failed"            :   "Done"})

    return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('final_exec_sergio')

    startup(setup_statemachine, robot_name='sergio')