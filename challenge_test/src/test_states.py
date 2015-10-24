#! /usr/bin/env python
import roslib;
import rospy
import smach
import subprocess
import inspect
import random
import ed_perception.msg
import robot_skills.util.msg_constructors as msgs
import math
from smach_ros import SimpleActionState
from robot_smach_states.util.designators import *
from robot_smach_states.human_interaction.human_interaction import HearOptionsExtra
from ed.msg import EntityInfo
from dragonfly_speech_recognition.srv import GetSpeechResponse
from robocup_knowledge import load_knowledge
from robot_skills.util import transformations

# ----------------------------------------------------------------------------------------------------

common_knowledge = load_knowledge("common")
challenge_knowledge = load_knowledge("challenge_person_recognition")
OBJECT_TYPES = challenge_knowledge.object_types

# define print shortcuts from common knowledge
printOk, printError, printWarning = common_knowledge.make_prints("[Challenge Test] ")



# ----------------------------------------------------------------------------------------------------

class PointAtOperator(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, robot):
        printOk("PointAtOperator")

        # Get information about the operator and point at the location
        self.robot.rightArm.send_goal(0.5, -0.2, 0.9, 0, 0, 0, 60)

        return 'done'