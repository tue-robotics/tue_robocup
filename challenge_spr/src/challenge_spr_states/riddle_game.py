#!/usr/bin/python

import roslib;
import rospy
import smach
import sys

import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states.util.designators import Designator, EdEntityDesignator

from robocup_knowledge import load_knowledge
data = load_knowledge('challenge_speech_recognition')

class HearQuestion(smach.State):
    def __init__(self, robot, time_out=rospy.Duration(15)):
        smach.State.__init__(self, outcomes=["answered"])
        self.robot = robot
        self.time_out = time_out

    def execute(self, userdata):
        self.robot.head.look_at_ground_in_front_of_robot(100)

        res = self.robot.ears.recognize(spec=data.spec, choices=data.choices, time_out=self.time_out)

        if not res:
            self.robot.speech.speak("My ears are not working properly.")

        if res:
            if "question" in res.choices:
                rospy.loginfo("Question was: '%s'?"%res.result)
                self.robot.speech.speak("The answer is %s"%data.choice_answer_mapping[res.choices['question']])
            else:
                self.robot.speech.speak("Sorry, I do not understand your question")

        return "answered"

        # Standalone testing -----------------------------------------------------------------

class TestRiddleGame(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'HEAR_QUESTION',
                                                'abort': 'Aborted'})

            smach.StateMachine.add("HEAR_QUESTION",
                                   HearQuestion(robot),
                                   transitions={'answered': 'HEAR_QUESTION_2'})

            smach.StateMachine.add("HEAR_QUESTION_2",
                                   HearQuestion(robot),
                                   transitions={'answered': 'Done'})

if __name__ == "__main__":
    rospy.init_node('speech_person_recognition_exec')

    startup(TestRiddleGame, challenge_name="challenge_spr")