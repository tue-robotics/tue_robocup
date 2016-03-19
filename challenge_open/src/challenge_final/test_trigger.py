#!/usr/bin/python

import rospy
import smach
import sys
import random
import math

from robot_smach_states.util.designators import *
import robot_smach_states as states
from robot_smach_states.util.startup import startup

############################## main statemachine ######################
def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:
        smach.StateMachine.add("TEST",
                                states.WaitForTrigger(robot, ["start"], "/sergio/trigger"),
                                transitions={   "preempted"              :   "Done",
                                                "start"           :   "Done"})

    return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('final_exec_amigo')

    startup(setup_statemachine, robot_name='sergio')
