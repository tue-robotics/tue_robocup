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

import robot_smach_states as states
from robot_smach_states.util.startup import startup
from challenge_restaurant import AskOrder, HearWhichTable, StoreWaypoint


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['done'])

    with sm:
        smach.StateMachine.add('WAYPOINT_SAY', states.Say(robot, "Say waypoint"), transitions={ 'spoken' :'WAYPOINT_ASK'})
        smach.StateMachine.add('WAYPOINT_ASK', StoreWaypoint(robot), transitions={ 'done' :'TABLE_SAY', 'continue' : 'TABLE_SAY'})

        smach.StateMachine.add('TABLE_SAY', states.Say(robot, "Say table"), transitions={ 'spoken' :'TABLE_ASK'})
        smach.StateMachine.add('TABLE_ASK', HearWhichTable(robot), transitions={ 'one' :'ORDER_SAY', 'two' :'ORDER_SAY', 'three' :'ORDER_SAY', 'no_result' :'ORDER_SAY',})

        smach.StateMachine.add('ORDER_SAY', states.Say(robot, "Say order"), transitions={ 'spoken' :'ORDER_ASK'})
        smach.StateMachine.add('ORDER_ASK', AskOrder(robot, "one"), transitions={ 'next_order' :'WAYPOINT_SAY', 'orders_done' : 'WAYPOINT_SAY'})

    return sm


############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('restaurant_test_recognition')

    startup(setup_statemachine, challenge_name="restaurant_test_recognition")