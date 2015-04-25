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


############################## main statemachine ######################
def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:
        smach.StateMachine.add("INITIALIZE",
                                states.StartChallengeRobust(robot, INITIAL_POSE, use_entry_points = True),
                                transitions={   "Done"              :   "WAIT_FOR_TRIGGER_TO_START",
                                                "Aborted"           :   "WAIT_FOR_TRIGGER_TO_START",
                                                "Failed"            :   "WAIT_FOR_TRIGGER_TO_START"})
       
        smach.StateMachine.add("WAIT_FOR_TRIGGER_TO_START", 
                                    AwaitTriggerAndSave(robot),
                                    transitions={   'done'              :'SAY_GET_OBJECT',
                                                    'failed'            :'SAY_GET_OBJECT'})

        #object_type_to_grab = 
        smach.StateMachine.add( 'SAY_GET_OBJECT',
                                SayFinal(robot),
                                transitions={'spoken':'Done'})

        # empty_arm_designator = UnoccupiedArmDesignator(robot.arms, robot.leftArm)
        # drink = EdEntityDesignator(robot, type="object_type_to_grab")

        # smach.StateMachine.add( "GRAB_PHONE",
        #                             Grab(robot, phone, empty_arm_designator),
        #                             transitions={   'done'              :'SAY_PHONE_TAKEN',
        #                                             'failed'            :'SAY_PHONE_TAKEN_FAILED'})

    return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('final_exec_amigo')

    startup(setup_statemachine, robot_name='amigo')