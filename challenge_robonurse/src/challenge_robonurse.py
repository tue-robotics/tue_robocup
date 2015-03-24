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

class DetectAction(smach.State):
    def __init__(self, robot, person_to_analyse):
        smach.State.__init__(self, outcomes=["drop_blanket", "fall", "walk_and_sit"])
        self.robot = robot
        self.person_to_analyse = person_to_analyse

    def execute(self, userdata):
        which = int(raw_input("Which action has been performed? : {0}".format({i:v for i, v in enumerate(self.registered_outcomes)})))
        return self.registered_outcomes[which]

class RoboNurse(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        granny = EdEntityDesignator(robot, type='human')
        shelf = EdEntityDesignator(robot, id='shelf') #TODO: determine ID of shelf

        def described_by_granny(entity):
            #TODO: Check whether the entity matches the description given by Granny
            return True

        described_bottle = LockingDesignator(EdEntityDesignator(robot, 
            criteriafuncs=[described_by_granny], debug=False))
        
        empty_arm_designator = UnoccupiedArmDesignator(robot.arms, robot.leftArm)
        arm_with_item_designator = ArmHoldingEntityDesignator(robot.arms, described_bottle)

        with self:
            smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'INTRO',
                                                'abort':'Aborted'})

            smach.StateMachine.add( "INTRO",
                                    states.Say(robot, ["I will be your RoboNurse today!", "I'm your RoboNurse, I'll be right there"]), 
                                    transitions={"spoken":"GOTO_GRANNY"})

            smach.StateMachine.add( "GOTO_GRANNY",
                                    #states.NavigateToPose(robot, 0, 0, 0),
                                    states.NavigateToSymbolic(robot, { granny:"near", EdEntityDesignator(robot, id=ROOM) : "in"}, granny),
                                    transitions={   'arrived'           :'ASK_GRANNY',
                                                    'unreachable'       :'ASK_GRANNY',
                                                    'goal_not_defined'  :'ASK_GRANNY'})

            smach.StateMachine.add( "ASK_GRANNY",
                                    states.Say(robot, ["Hello Granny, Shall I bring you your pills?"]),
                                    transitions={   'spoken'            :'GOTO_SHELF'})

            smach.StateMachine.add( "GOTO_SHELF",
                                    states.NavigateToSymbolic(robot, { shelf:"front", EdEntityDesignator(robot, id=ROOM) : "in"}, shelf),
                                    transitions={   'arrived'           :'LOOKAT_SHELF',
                                                    'unreachable'       :'LOOKAT_SHELF',
                                                    'goal_not_defined'  :'LOOKAT_SHELF'})

            smach.StateMachine.add( "LOOKAT_SHELF",
                                    states.Say(robot, "TODO: Look at shelf"),
                                    transitions={   'spoken'            :'DESCRIBE_OBJECTS'})

            smach.StateMachine.add( "DESCRIBE_OBJECTS",
                                    states.Say(robot, "TODO: Tell about the bottles"),
                                    transitions={   'spoken'            :'ASK_WHICH_BOTTLE'})

            smach.StateMachine.add( "ASK_WHICH_BOTTLE",
                                    states.Say(robot, "TODO: Ask which bottle do you want?"),
                                    transitions={   'spoken'            :'GRAB_BOTTLE'})

            smach.StateMachine.add( "GRAB_BOTTLE",
                                    Grab(robot, described_bottle, empty_arm_designator),
                                    transitions={   'done'              :'GOTO_GRANNY_WITH_BOTTLE',
                                                    'failed'            :'SAY_GRAB_FAILED'})

            smach.StateMachine.add( "SAY_GRAB_FAILED",
                                    states.Say(robot, "I couldn't grab the bottle, sorry"),
                                    transitions={   'spoken'            :'GOTO_GRANNY_WITHOUT_BOTTLE'})

            smach.StateMachine.add( "GOTO_GRANNY_WITHOUT_BOTTLE",
                                    states.NavigateToSymbolic(robot, {granny:"near", EdEntityDesignator(robot, id=ROOM):"in"}, granny),
                                    transitions={   'arrived'           :'DETECT_ACTION',
                                                    'unreachable'       :'DETECT_ACTION',
                                                    'goal_not_defined'  :'DETECT_ACTION'})

            smach.StateMachine.add( "GOTO_GRANNY_WITH_BOTTLE",
                                    states.NavigateToSymbolic(robot, {granny:"near", EdEntityDesignator(robot, id=ROOM):"in"}, granny),
                                    transitions={   'arrived'           :'SAY_HANDOVER_BOTTLE',
                                                    'unreachable'       :'DETECT_ACTION',
                                                    'goal_not_defined'  :'DETECT_ACTION'})

            smach.StateMachine.add( "SAY_HANDOVER_BOTTLE",
                                    states.Say(robot, ["Here are your pills, Granny.", "Granny, here are your pills."]),
                                    transitions={   'spoken'            :'HANDOVER_TO_GRANNY'})

            smach.StateMachine.add('HANDOVER_TO_GRANNY',
                                   states.HandoverToHuman(robot, arm_with_item_designator),
                                   transitions={   'succeeded'          :'DETECT_ACTION',
                                                    'failed'            :'DETECT_ACTION'})

            smach.StateMachine.add('DETECT_ACTION',
                                   DetectAction(robot, granny),
                                   transitions={    "drop_blanket"      :"PICKUP_BLANKET", 
                                                    "fall"              :"BRING_PHONE",
                                                    "walk_and_sit"      :"FOLLOW_TAKE_CANE"})

            smach.StateMachine.add( "PICKUP_BLANKET",
                                    states.Say(robot, ["I will pick up your blanket"]),
                                    transitions={   'spoken'            :'Done'})

            smach.StateMachine.add( "BRING_PHONE",
                                    states.Say(robot, ["I will bring you a phone"]),
                                    transitions={   'spoken'            :'Done'})

            smach.StateMachine.add( "FOLLOW_TAKE_CANE",
                                    states.Say(robot, ["I will hold your cane"]),
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
