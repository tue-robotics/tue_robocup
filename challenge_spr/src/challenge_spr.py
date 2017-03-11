#! /usr/bin/env python

import rospy
import smach
import time
import os
import datetime
import math

from robot_smach_states import Initialize, Say, WaitForPersonInFront, Turn, WaitTime
from robot_smach_states.util.startup import startup
import robot_smach_states.util.designators as ds

from challenge_spr.detect import DetectCrowd
from challenge_spr import bluff_game
from challenge_spr import riddle_game

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
                                   Turn(robot, math.pi),
                                   transitions={"turned": "DETECT_CROWD"})

            smach.StateMachine.add("DETECT_CROWD",
                                   DetectCrowd(robot),
                                   transitions={"succeeded": "REQUEST_OPERATOR",
                                                "failed": "REQUEST_OPERATOR"})

            smach.StateMachine.add("REQUEST_OPERATOR",
                                   Say(robot, "Who wants to play riddles with me?"),
                                   transitions={"spoken": "WAIT_FOR_OPERATOR"})

            smach.StateMachine.add("WAIT_FOR_OPERATOR",
                                   WaitForPersonInFront(robot, attempts=1, sleep_interval=1),
                                   transitions={'success': 'RIDDLE_GAME_1',
                                                'failed': 'RIDDLE_GAME_1'})

            # Riddle Game

            smach.StateMachine.add('RIDDLE_GAME_1', riddle_game.HearQuestion(robot), transitions={ 'answered':'BLUFF_GAME_1'})

            # Bluff Game

            smach.StateMachine.add('BLUFF_GAME_1', bluff_game.HearQuestion(robot), transitions={ 'answered' :'TURN_1_ANSWERED', 'not_answered': 'TURN_1_MISSED'})
            smach.StateMachine.add('TURN_1_ANSWERED', bluff_game.Turn(robot), transitions={ 'turned' :'END_CHALLENGE'})
            smach.StateMachine.add('TURN_1_MISSED', bluff_game.Turn(robot), transitions={ 'turned' :'BLUFF_GAME_1_2'})
            smach.StateMachine.add('BLUFF_GAME_1_2', bluff_game.HearQuestion(robot), transitions={ 'answered' :'END_CHALLENGE', 'not_answered': 'END_CHALLENGE'})          

            # End

            smach.StateMachine.add('END_CHALLENGE',
                                   Say(robot, "My work here is done, goodbye!"),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "person_recognition")

if __name__ == "__main__":
    rospy.init_node('speech_person_recognition_exec')

    startup(ChallengeSpeechPersonRecognition, challenge_name="challenge_spr")
