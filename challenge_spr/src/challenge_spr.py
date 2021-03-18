#! /usr/bin/env python

import rospy
import smach
import math

from robot_smach_states.human_interaction.answer_questions import HearAndAnswerQuestions
from robot_smach_states.human_interaction import Say, WaitForPersonInFront
from robot_smach_states.navigation import ForceDrive
from robot_smach_states.utility import Initialize, WaitTime
from robot_smach_states.util.startup import startup
import robot_smach_states.util.designators as ds

from challenge_spr_states import detect
from challenge_spr_states import bluff_game

from robocup_knowledge import load_knowledge
knowledge = load_knowledge('challenge_spr')
common_knowledge = load_knowledge('common')


class ChallengeSpeechPersonRecognition(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   Initialize(robot),
                                   transitions={'initialized': 'ANNOUNCEMENT',
                                                'abort': 'Aborted'})

            smach.StateMachine.add("ANNOUNCEMENT",
                                   Say(robot, "I want to play a riddle game"),
                                   transitions={"spoken": "WAIT"})

            smach.StateMachine.add("WAIT",
                                   WaitTime(robot),
                                   transitions={"waited": "TURN",
                                                "preempted": "TURN"})

            smach.StateMachine.add("TURN",
                                   ForceDrive(robot, vx=0.0, vy=0.0, vth=1.0, duration=math.pi),
                                   transitions={"turned": "DETECT_CROWD"})

            smach.StateMachine.add("DETECT_CROWD",
                                   detect.DetectCrowd(robot),
                                   transitions={"succeeded": "REQUEST_OPERATOR",
                                                "failed": "REQUEST_OPERATOR"},
                                    remapping={'crowd_data': 'crowd_data'})

            smach.StateMachine.add("REQUEST_OPERATOR",
                                   Say(robot, "Who wants to play riddles with me?"),
                                   transitions={"spoken": "WAIT_FOR_OPERATOR"})

            smach.StateMachine.add("WAIT_FOR_OPERATOR",
                                   WaitForPersonInFront(robot, attempts=3, sleep_interval=1),
                                   transitions={'success': 'RIDDLE_GAME',
                                                'failed': 'RIDDLE_GAME'})

            # Riddle Game

            smach.StateMachine.add('RIDDLE_GAME',
                                   HearAndAnswerQuestions(
                                       robot,
                                       grammar=knowledge.grammar,
                                       knowledge=common_knowledge,
                                       num_questions=5,
                                   ),
                                   transitions={'done':'TRANSITION'})

            # Transition:

            smach.StateMachine.add("TRANSITION",
                                   Say(robot, "Now lets play the blind mans bluff game"),
                                   transitions={"spoken": "BLUFF_GAME"})

            # Bluff Games:

            smach.StateMachine.add('BLUFF_GAME',
                                   bluff_game.HearTurnAndAnswerQuestions(robot, num_questions=5, num_operators=5),
                                   transitions={'done': 'END_CHALLENGE'})
            # End

            smach.StateMachine.add('END_CHALLENGE',
                                   Say(robot, "My work here is done, goodbye!"),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "person_recognition")


if __name__ == "__main__":
    rospy.init_node('speech_person_recognition_exec')

    startup(ChallengeSpeechPersonRecognition, challenge_name="challenge_spr")
