#! /usr/bin/env python
import rospy

import smach

from robot_smach_states.utility_states import Initialize
from robot_smach_states.human_interaction import Say
from dragonfly_speech_recognition.msg import Choice

# dictionary of questions and answers
# QA_MAP = { "Sjoerd" : "Is awesome",
#            "Rein" : "Is quite ok",
#            "Janno" : "Is very alright" }

class AnswerQuestion(smach.State):

    def __init__(self, robot, question_nr):
        smach.State.__init__(self, outcomes = ['done', 'failed'])
        self.robot = robot
        self.question_nr = question_nr

    def execute(self, userdate=None):

        self.robot.speak("Please ask question " + str(self.question_nr))

        #req = "(" + "|".join(QA_MAP.keys()) + ")"

        spec = "<questions>"
        choices = [Choice("questions",{
                          "Is Sjoerd awesome": "He has his moments",                           
                          "But is Erik a king": "Hell yeah he is!", 
                         }
                  )
          ]

        res = self.robot.ears.recognize(spec=spec, choices=choices)

        if not res:
            self.robot.speak("I could not hear your question.")
            return "failed"

        try:
            q_answer = answers.choices["questions"]
            #q_answer = QA_MAP[res.result]
        except KeyError:
            print "[what_did_you_say] Received question is not in map. THIS SHOULD NEVER HAPPEN!"
            return "failed"

        self.robot.speak("Your question was: " + res.result + ". The answer is: " + q_answer)

        return "done"

class WhatDidYouSay(smach.StateMachine):

    def __init__(self, robot, grasp_arm="left"):
        # ToDo: get rid of hardcode poi lookat
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot

        with self:

            smach.StateMachine.add( 'INIT',
                                    Initialize(robot),
                                    transitions={ "initialized": "ASK_QUESTION_1",
                                                  "abort":       "Aborted"})

            smach.StateMachine.add( 'ASK_QUESTION_1',
                                    AnswerQuestion(robot, 1),
                                    transitions={ 'done' : 'ASK_QUESTION_2'},
                                                  'failed' : 'ASK_QUESTION_1'))

            smach.StateMachine.add( 'ASK_QUESTION_2',
                                    AnswerQuestion(robot, 2),
                                    transitions={ 'done' : 'ASK_QUESTION_3'},
                                                  'failed' : 'ASK_QUESTION_2'))

            smach.StateMachine.add( 'ASK_QUESTION_3',
                                    AnswerQuestion(robot, 3),
                                    transitions={ 'done' : 'Done'},
                                                  'failed' : 'ASK_QUESTION_3'))

if __name__ == "__main__":
    rospy.init_node('what_did_you_say_exec')
    robot_smach_states.util.startup(WhatDidYouSay)
