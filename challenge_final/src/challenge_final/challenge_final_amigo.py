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
INITIAL_POSE = challenge_knowledge.initial_pose_amigo

from robot_smach_states.util.designators import Designator

OBJECTS_LIST = []

class AwaitTriggerAndSave(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "failed"])
        self.robot = robot

    def execute(self, userdata):

        order_object = states.WaitForTrigger(self.robot, challenge_knowledge.object_options,"amigo/trigger")

        OBJECTS_LIST.append(str(order_object.execute(None)))

        return "done"

class SayFinal(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["spoken"])
        self.robot = robot

    def execute(self, userdata):

        sentence = "Sergio triggered me! Let's get a "+ OBJECTS_LIST[0] +" for the boss!"

        say_final = states.Say(self.robot, sentence,block=False)

        say_final.execute(None)

        return "spoken"

class GrabFinal(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done","failed"])
        self.robot = robot

    def execute(self, userdata):

        empty_arm_designator = UnoccupiedArmDesignator(self.robot.arms, self.robot.leftArm)
        drink = EdEntityDesignator(self.robot, type=OBJECTS_LIST[0])

        grab_coke = states.Grab(self.robot, drink, empty_arm_designator)

        status = grab_coke.execute(None)

        return status

############################## main statemachine ######################
def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:
        smach.StateMachine.add("WAIT_FOR_TRIGGER_TO_START", 
                                    AwaitTriggerAndSave(robot),
                                    transitions={   'done'              :'INITIALIZE',
                                                    'failed'            :'INITIALIZE'})
        smach.StateMachine.add("INITIALIZE",
                                states.StartChallengeRobust(robot, INITIAL_POSE, use_entry_points = True),
                                transitions={   "Done"              :   "SAY_GET_OBJECT",
                                                "Aborted"           :   "SAY_GET_OBJECT",
                                                "Failed"            :   "SAY_GET_OBJECT"})

        smach.StateMachine.add( 'SAY_GET_OBJECT',
                                SayFinal(robot),
                                transitions={'spoken':'GRAB_OBJECT'})

        smach.StateMachine.add( "GRAB_OBJECT",
                                    GrabFinal(robot),
                                    transitions={   'done'              :'Done',
                                                    'failed'            :'Done'})

    return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('final_exec_amigo')

    startup(setup_statemachine, robot_name='amigo')