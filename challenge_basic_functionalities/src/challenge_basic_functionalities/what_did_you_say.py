#! /usr/bin/env python
import rospy

import smach

from robot_smach_states.utility_states import Initialize
from robot_smach_states.human_interaction import Say
from dragonfly_speech_recognition.msg import Choice

# dictionary of questions and answers
QA_MAP = { "what time is it" : "time to buy a watch",
           "what is the capital of germany" : "berlin",
           "what is the heaviest animal in the world" : "Is quite ok",
           "who is the president of america" : "barack obama",
           "what is your name" : "amigo",
           "who is your example" : "erik geerts",
           "what is your motto" : "yolo",
           "which football club is the best" : "feyenoord",
           "who is the best looking person around here" : "erik geerts of course",
           "which town has been bombed" : "schijndel",
           "which person is not able to say yes" : "dirk holtz",
           "what is the capital of brazil" : "brasilia",
           "what is the oldest drug used on earth" : "alcohol",
           "in which year was robocup founded" : "nineteen ninety seven",
           "how many rings has the olympic flag" : "five",
           "what is the worlds most popular green vegetable" : "lettuce",
           "which insect has the best eyesight" : "dragonfly",
           "who lives in a pineapple under the sea" : "spongebob squarepants",
           "what is your teams name" : "tech united eindhoven",
           "what is the answer to the ultimate question about life the universe and everything" : "forty two",
           "what is the capital of poland" : "warsaw",
           "which country grows the most potatoes" : "russia",
           "which country grew the first orange" : "china",
           "how many countries are in europe" : "fifty",
           "which fish can hold objects in its tail" : "sea horse"}

class AnswerQuestion(smach.State):

    def __init__(self, robot, question_nr):
        smach.State.__init__(self, outcomes = ['done', 'failed'])
        self.robot = robot
        self.question_nr = question_nr

    def execute(self, userdate=None):

        print "------- POSSIBLE QUESTIONS -------"
        for x in range(0, len(QA_MAP)):
            print "Question", 1+x,":", QA_MAP.keys()[x],"?"

        self.robot.speech.speak("Please ask question " + str(self.question_nr))

        #req = "(" + "|".join(QA_MAP.keys()) + ")"

        spec = "<questions>"
        choices = { "questions": [ x for x in QA_MAP ] }

        res = self.robot.ears.recognize(spec=spec, choices=choices)

        if not res:
            self.robot.speech.speak("I could not hear your question.")
            return "failed"

        try:
            q_answer = QA_MAP[res.choices["questions"]]
            #q_answer = QA_MAP[res.result]
        except KeyError:
            print "[what_did_you_say] Received question is not in map. THIS SHOULD NEVER HAPPEN!"
            return "failed"

        self.robot.speech.speak("Your question was: " + res.choices["questions"] + ". The answer is: " + q_answer)

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
                                    transitions={ 'done' : 'ASK_QUESTION_2',
                                                  'failed' : 'ASK_QUESTION_1'})

            smach.StateMachine.add( 'ASK_QUESTION_2',
                                    AnswerQuestion(robot, 2),
                                    transitions={ 'done' : 'ASK_QUESTION_3',
                                                  'failed' : 'ASK_QUESTION_2'})

            smach.StateMachine.add( 'ASK_QUESTION_3',
                                    AnswerQuestion(robot, 3),
                                    transitions={ 'done' : 'Done',
                                                  'failed' : 'ASK_QUESTION_3'})

if __name__ == "__main__":
    rospy.init_node('what_did_you_say_exec')
    robot_smach_states.util.startup(WhatDidYouSay)
