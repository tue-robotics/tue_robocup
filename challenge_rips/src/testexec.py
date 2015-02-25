#!/usr/bin/python
import roslib;
import rospy
import smach

from robot_skills.amigo import Amigo
from robot_skills.sergio import Sergio
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from geometry_msgs.msg import Point

from speech_interpreter.srv import AskUser

from psi import *
import robot_skills.util.msg_constructors as msgs

import sys

from robot_smach_states.util.designators import Designator, VariableDesignator

# def setup_statemachine(robot):

# 	sm = smach.StateMachine(outcomes=['Done','Aborted'])

# 	with sm:
# 		smach.StateMachine.add('DRIVE', 
# 			                    states.NavigateToPose(robot, x = 1, y = 3, rz = -1.57),
# 			                    transitions={   'arrived':'Done', 
# 			                                    'unreachable':'Aborted', 
# 			                                    'goal_not_defined':'Aborted'})

############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('test_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
        if robot_name == 'sergio':
        	robot = Sergio()
        elif robot_name == 'amigo':
        	robot = Amigo()
        else:
        	robot = Amigo() 

    #testexec = states.NavigateToPose(robot, 3, 1, -1.57, 0.15)
    testexec = states.NavigateToObserve(robot, Designator("dinner_table"))
    testexec.execute()

    #startup(setup_statemachine, robot_name=robot_name)