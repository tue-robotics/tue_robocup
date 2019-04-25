#! /usr/bin/env python

################################################
# Creator: Luis Ferreira (luisfferreira@outlook.com)
# Date: October 2015
################################################

import roslib
import rospy
import smach
import subprocess
import inspect
import random
import ed_perception.msg
import math
import robot_skills.util.msg_constructors as msgs

from ed_msgs.msg import EntityInfo
from smach_ros import SimpleActionState
from collections import namedtuple
from dragonfly_speech_recognition.srv import GetSpeechResponse
from robot_smach_states.util.designators import *
from robot_smach_states.human_interaction.human_interaction import HearOptionsExtra
from robot_smach_states import Grab
from robocup_knowledge import load_knowledge
from robot_skills.util import transformations
from robot_skills.arms import Arm


# ----------------------------------------------------------------------------------------------------

common_knowledge = load_knowledge("common")
challenge_knowledge = load_knowledge("challenge_person_recognition")
OBJECT_TYPES = challenge_knowledge.object_types

# define print shortcuts from common knowledge
printOk, printError, printWarning = common_knowledge.make_prints("[Challenge Test] ")


# ----------------------------------------------------------------------------------------------------

# OBSELETE! BUT MIGHT BE USEFULL
# class PointAtOperator(smach.State):
#     def __init__(self, robot):
#         smach.State.__init__(self, outcomes=['done'])
#         self.robot = robot

#     def execute(self, robot):
#         printOk("PointAtOperator")

#         # Get information about the operator and point at the location
#         self.robot.rightArm.send_goal(0.5, -0.2, 0.9, 0, 0, 0, 60)

#         return 'done'


# ----------------------------------------------------------------------------------------------------


class AskPersonName(smach.State):
    """
        Ask the person's name, and try to hear one of the names in common_knowledge
    """
    def __init__(self, robot, personNameDes, defaultName = 'Operator'):
        smach.State.__init__(   self, outcomes=['succeeded', 'failed'])

        self.robot = robot
        self.personNameDes = personNameDes
        self.defaultName = defaultName

    def execute(self, userdata=None):
        printOk("AskPersonName")

        self.robot.speech.speak("What is your name?", block=True)

        spec = Designator("((<prefix> <name>)|<name>)")

        choices = Designator({"name"  : common_knowledge.names,
                              "prefix": ["My name is", "I'm called", "I am"]})

        answer = VariableDesignator(resolve_type=GetSpeechResponse)

        state = HearOptionsExtra(self.robot, spec, choices, answer.writeable)
        outcome = state.execute() # REVIEW(Loy): Luis, this is really not the way to go. Nest state machines using the smach way

        if not outcome == "heard":
            self.personNameDes.write(self.defaultName)

            printWarning("Speech recognition outcome was not successful (outcome: '{0}'). Using default name '{1}'".format(str(outcome), self.personNameDes.resolve()))
            return 'failed'
        else:
            try:
                print answer.resolve()
                name = answer.resolve().choices["name"]
                self.personNameDes.write(name)

                printOk("Result received from speech recognition is '" + name + "'")
            except KeyError, ke:
                printOk("KeyError resolving the name heard: " + str(ke))
                pass

        return 'succeeded'


# ----------------------------------------------------------------------------------------------------


class PickUpRandomObj(smach.State):
    """
        Ask the person's name, and try to hear one of the names in common_knowledge
    """
    def __init__(self, robot, objectsIDsDes):
        smach.State.__init__(   self, outcomes=['succeeded', 'failed', 'no_objects'])

        self.robot = robot
        self.objectsIDsDes = objectsIDsDes

    def execute(self, userdata=None):
        printOk("PickUpRandomObj")

        objsResolved = self.objectsIDsDes.resolve()

        if not (self.objectsIDsDes) or len(objsResolved) == 0:
            self.robot.speech.speak("I don't see any objects that i can pick up!", block=False)
            return 'no_objects'
        else:
            sentence = "I see "

            # describe objects
            for idx, obj in enumerate(objsResolved):

                if (idx < len(objsResolved)-1 or len(objsResolved) == 1):
                    # if its not the last item or there is only one
                    sentence+=(", a ")
                else:
                    # if its the last item, finish with 'and'
                    sentence+=("and a ")

                sentence+=("{} ".format(objsResolved[0].type if objsResolved[0].type else "unknown object"))

            # print sentence

            # Anounce objects found
            self.robot.speech.speak(sentence, block=False)

            selectedObj = random.choice(objsResolved)

            # import ipdb; ipdb.set_trace()

            # anounce which object is going to be picked up
            self.robot.speech.speak("I am going to pick up the {}".format(
                selectedObj.type if selectedObj.type else "unknown object"), block=False)

            armDes = UnoccupiedArmDesignator(self.robot, {'required_arm_name':'right'})
            entityDes = EdEntityDesignator(self.robot, id=selectedObj.id)

            # Grab(robot, self.current_item, self.empty_arm_designator),
            grabState = Grab(self.robot, entityDes, armDes)
            result = grabState.execute()

            if result == 'done':
                return 'succeeded'
            else:
                return 'failed'


# ----------------------------------------------------------------------------------------------------


class RecognizePeople(smach.State):
    """
        Attempt to recognize the peolpe in front of the robot and say their names
    """
    def __init__(self, robot):
        smach.State.__init__(   self, outcomes=['succeeded', 'failed', 'no_people'])

        self.robot = robot

    def execute(self, userdata=None):
        printOk("RecognizePeople")

        rospy.logerr("ed.detect_ persons() method disappeared! This was only calling the face recognition module and we are using a new one now!")
        rospy.logerr("I will return an empty detection list!")
        detections = []

        if not detections:
            return no_people
        else:

            self.robot.speech.speak("I see {0} {1}".format(len(detections),
                    "person" if len(detections) == 1 else "people"), block=False)

            # import ipdb; ipdb.set_trace()

            for det in detections:
                printOk("Name: {} \tAge:{} \tGender:{} Pose:{}".format(det.name, det.age, det.gender, det.pose))

                if det.name:
                    self.robot.speech.speak("Hello {}!".format(det.name), block=True)
                else:
                    self.robot.speech.speak("Hello stranger!".format(det.name), block=True)

        return 'succeeded'


# ----------------------------------------------------------------------------------------------------


class SelectNextContainer(smach.State):
    """
        Select a new test to be executed, either by a certain order, random or on repeat
    """
    def __init__(self, robot, containerResultDes):
        smach.State.__init__(   self, outcomes=['go_to_enter_room',
                                                'go_to_wait_person',
                                                'go_to_pick_up',
                                                'go_to_recognize_people',
                                                'go_to_search_people'])

        self.containerResultDes = containerResultDes
        self.robot = robot
        self.nextContainer = ""
        self.statusLog = {stateName:[0, 0] for stateName in self.get_registered_outcomes()}

    def execute(self, userdata):
        printOk("SelectNextContainer")

        # skip if there is no nextContainer (first run)
        if self.nextContainer:
            # update total number of runs
            self.statusLog[self.nextContainer][1] += 1
            # updatenumber of successfull runs
            if self.containerResultDes.resolve() == 0:
                self.statusLog[self.nextContainer][0] += 1

        print ""
        # print results
        for name, stateInfo in self.statusLog.iteritems():
            printOk("Container '{}': \tsucceeded {}/{}".format(name, stateInfo[0], stateInfo[1]))
        print ""

        # import ipdb; ipdb.set_trace()
        self.nextContainer = random.choice(self.get_registered_outcomes())

        printOk("Jumping to next container: '{}'".format(self.nextContainer))
        return self.nextContainer

