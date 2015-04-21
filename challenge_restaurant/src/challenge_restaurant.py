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

class GUIDING_PHASE(smach.State):
    def __init__(self, robot, time_out=rospy.Duration(10)):
        smach.State.__init__(self, outcomes=["answered"])
        self.robot = robot
        self.time_out = time_out

    def execute(self, userdata):
        # add while guiding
        res = amigo.ears.recognize("<choice>",{"choice":["TABLE A","TABLE B","TABLE C"]})
        
        if res:
            if "question" in res.choices:
                rospy.loginfo("Is this talble: '%s'?"%res.result)
                self.robot.speech.speak("Is this talble %s"%res.choice['question'])
        return "answered"

def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:
        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'Done',
                                                'abort':'Aborted'})
        smach.StateMachine.add('SAY_INTRO', states.Say(robot, "Hi Guide, Show me your restaurant please"), transitions={ 'spoken' :'GUIDING_PHASE'})
        
    
    
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
