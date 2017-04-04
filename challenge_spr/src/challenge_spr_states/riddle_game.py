#!/usr/bin/python

import roslib;
import rospy
import smach
import sys

import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states.util.designators import Designator, EdEntityDesignator

from robocup_knowledge import load_knowledge
choice_answer_mapping = load_knowledge('challenge_spr').choice_answer_mapping


def hear(robot, time_out):
    spec = '<question>'
    choices = {'question': [k for k,v in choice_answer_mapping.iteritems()]}
    
    return robot.ears.recognize(spec=spec, choices=choices, time_out=time_out)


def answer(robot, res, crowd_data):
    if res:
        if "question" in res.choices:
            answer = choice_answer_mapping[res.choices['question']]
            
            # override crowd answers
            if answer == 'ANSWER_NO_MALES':
                answer = 'In the crowd are %d males' % crowd_data['males']

            rospy.loginfo("Question was: '%s'?"%res.result)
            robot.speech.speak("The answer is %s"%answer)
            return 'answered'
        else:
            robot.speech.speak("Sorry, I do not understand your question")
    else:
        robot.speech.speak("My ears are not working properly.")    

    return 'not_answered'


class HearQuestion(smach.State):
    def __init__(self, robot, time_out=rospy.Duration(15)):
        smach.State.__init__(self, outcomes=["answered"], input_keys=['crowd_data'])
        self.robot = robot
        self.time_out = time_out

    def execute(self, userdata):
        crowd_data = userdata.crowd_data

        self.robot.head.look_at_standing_person()

        res = hear(self.robot, time_out=self.time_out)

        answer(self.robot, res, crowd_data)

        return "answered"

        # Standalone testing -----------------------------------------------------------------

class TestRiddleGame(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        self.userdata.crowd_data = {
            'males': 2,
            'females': 3
        }

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'HEAR_QUESTION',
                                                'abort': 'Aborted'})

            smach.StateMachine.add("HEAR_QUESTION",
                                   HearQuestion(robot),
                                   transitions={'answered': 'HEAR_QUESTION_2'},
                                   remapping={'crowd_data':'crowd_data'})

            smach.StateMachine.add("HEAR_QUESTION_2",
                                   HearQuestion(robot),
                                   transitions={'answered': 'Done'})

if __name__ == "__main__":
    rospy.init_node('speech_person_recognition_exec')

    startup(TestRiddleGame, challenge_name="challenge_spr")