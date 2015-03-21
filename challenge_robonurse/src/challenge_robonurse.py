#!/usr/bin/python

"""This challenge is defined in https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/master/RoboNurse.tex

In short, the robot goes from it start in the corner of the room to Granny.
Granny, instructs the robot to get some pills for her.
At the shelf, there are a bunch of bottles with pills.
The robot must describe the bottles and let Granny choose a bottle.
The robot must grab the bottle and bring it to Granny.

Then, part 2 start which involves action recognition.
Granny does 1 of 3 things to which the robot must respond.
"""

import rospy
import smach
import sys
import random

from robot_smach_states.util.designators import *
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states import Grab
from robot_smach_states import Place
from robot_skills.util import msg_constructors as geom
from robot_skills.util import transformations

ROOM = "room_livingroom"

class RoboNurse(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        granny = EdEntityDesignator(robot, type='person')
        shelf = EdEntityDesignator(robot, id='shelf') #TODO: determine ID of shelf

        with self:
            smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'INTRO',
                                                'abort':'Aborted'})

            smach.StateMachine.add( "INTRO",
                                    states.Say(robot, ["I will be your RoboNurse today!", "I'm your RoboNurse, I'll be right there"]), 
                                    transitions={"spoken":"GOTO_TO_GRANNY"})

            smach.StateMachine.add( "GOTO_TO_GRANNY",
                                    states.NavigateToSymbolic(robot, {granny:"near", EdEntityDesignator(robot, id=ROOM):"in"}, granny),
                                    transitions={   'arrived'           :'ASK_GRANNY',
                                                    'unreachable'       :'ASK_GRANNY',
                                                    'goal_not_defined'  :'ASK_GRANNY'})

            smach.StateMachine.add( "ASK_GRANNY",
                                    states.Say(robot, ["Hello Granny, Shall I bring you your pills?"]),
                                    transitions={   'spoken'            :'GOTO_SHELF'})

            smach.StateMachine.add( "GOTO_SHELF",
                                    states.NavigateToSymbolic(robot, {shelf:"front", EdEntityDesignator(robot, id=ROOM):"in"}, shelf),
                                    transitions={   'arrived'           :'LOOKAT_SHELF',
                                                    'unreachable'       :'LOOKAT_SHELF',
                                                    'goal_not_defined'  :'LOOKAT_SHELF'})

            smach.StateMachine.add( "LOOKAT_SHELF",
                                    states.Say(robot, "TODO: Look at shelf"),
                                    transitions={   'spoken'            :'DESCRIBE_OBJECTS'})

            smach.StateMachine.add( "DESCRIBE_OBJECTS",
                                    states.Say(robot, "TODO: Tell about the bottles"),
                                    transitions={   'spoken'            :'Done'})


############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('robonurse_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE MANIPULATION] Please provide robot name as argument."
        exit(1)

    startup(RoboNurse, robot_name=robot_name)
