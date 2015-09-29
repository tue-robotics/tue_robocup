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


''' printing shortcuts '''
prefix = common_knowledge.bcolors.HEADER + "[Challenge Test] " + common_knowledge.bcolors.ENDC

def printOk(sentence):
    print prefix + common_knowledge.bcolors.OKBLUE + sentence + common_knowledge.bcolors.ENDC

def printError(sentence):
    print prefix + common_knowledge.bcolors.FAIL + sentence + common_knowledge.bcolors.ENDC

def printWarning(sentence):
    print prefix + common_knowledge.bcolors.WARNING + sentence + common_knowledge.bcolors.ENDC


