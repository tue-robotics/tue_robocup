#!/usr/bin/python

"""This challenge is defined in https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/master/Restaurant.tex
In short:

1) The robot follows a guide inside a unknown space while learning the environment. The guide will tell the robot where the tables are.
2) After the robot has been guided back to the kitchen, the robot asks which table he should go to ask an order.
   Robot drives to that location and asks for an order which can be either a drink (coke) or a combo (burger with fries)
   Robot then drives to the kitchen and graps the drink or asks for a tray with the combo.
3) At any time during step 2) when somebody at an other table waves at the robot it should go there and ask for an order
4) Once two orders are specified, the robot should deliver the combo and the drink.
"""

import rospy
import smach
import sys

import robot_smach_states as states
from robot_smach_states.util.startup import startup

import follow_operator_and_store_waypoints

def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['done', 'aborted'])

    with sm:
        smach.StateMachine.add('INITIALIZE', states.Initialize(robot), transitions={   'initialized':'FOLLOW_OPERATOR_AND_STORE_WAYPOINTS', 'abort':'aborted'})
        smach.StateMachine.add('FOLLOW_OPERATOR_AND_STORE_WAYPOINTS', follow_operator_and_store_waypoints.FollowOperatorAndStoreWaypoints(robot), transitions={ 'done':'done', 'aborted':'aborted'})
        smach.StateMachine.add('GOTO', states.Initialize(robot), transitions={   'initialized':'FOLLOW_OPERATOR_AND_STORE_WAYPOINTS', 'abort':'aborted'})

#        smach.StateMachine.add('SAY_INTRO', states.Say(robot, "Hi Guide, Show me your restaurant please"), transitions={ 'spoken' :'GUIDING_PHASE'})



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
