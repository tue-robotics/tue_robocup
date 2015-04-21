#!/usr/bin/python

"""This challenge is defined in https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/master/Restaurant.tex
"""

import rospy
import smach
import sys
import random
import math

from robot_smach_states.util.designators import *
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states import Grab
from robot_smach_states import Place
from robot_smach_states.util.geometry_helpers import *
from robot_skills.util import msg_constructors as geom
from robot_skills.util import transformations
import geometry_msgs.msg as gm
from robot_skills.util import transformations 
from cb_planner_msgs_srvs.msg import PositionConstraint

def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:
        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'Done',
                                                'abort':'Aborted'})
    return sm


############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('restaurant_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE RESTAURANT] Please provide robot name as argument."
        exit(1)

    startup(setup_statemachine, robot_name=robot_name)
