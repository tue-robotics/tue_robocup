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

from challenge_spr_states import detect
from challenge_spr_states import bluff_game
from challenge_spr_states import riddle_game

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
                                   detect.DetectCrowd(robot),
                                   transitions={"succeeded": "REQUEST_OPERATOR",
                                                "failed": "REQUEST_OPERATOR"},
                                    remapping={'crowd_data': 'crowd_data'})

            smach.StateMachine.add("REQUEST_OPERATOR",
                                   Say(robot, "Who wants to play riddles with me?"),
                                   transitions={"spoken": "WAIT_FOR_OPERATOR"})

            smach.StateMachine.add("WAIT_FOR_OPERATOR",
                                   WaitForPersonInFront(robot, attempts=3, sleep_interval=1),
                                   transitions={'success': 'RIDDLE_GAME_1',
                                                'failed': 'RIDDLE_GAME_1'})

            # Riddle Game

            smach.StateMachine.add('RIDDLE_GAME_1', 
                                   riddle_game.HearQuestion(robot), 
                                   transitions={ 'answered':'RIDDLE_GAME_2',
                                                 'not_answered':'RIDDLE_GAME_1_MISSED'})

            smach.StateMachine.add("RIDDLE_GAME_1_MISSED",
                                   Say(robot, "Please ask next question!"),
                                   transitions={"spoken": "RIDDLE_GAME_2"})

            smach.StateMachine.add('RIDDLE_GAME_2', 
                                   riddle_game.HearQuestion(robot), 
                                   transitions={ 'answered':'RIDDLE_GAME_3',
                                                 'not_answered':'RIDDLE_GAME_2_MISSED'})

            smach.StateMachine.add("RIDDLE_GAME_2_MISSED",
                                   Say(robot, "Please ask next question!"),
                                   transitions={"spoken": "RIDDLE_GAME_3"})

            smach.StateMachine.add('RIDDLE_GAME_3', 
                                   riddle_game.HearQuestion(robot), 
                                   transitions={ 'answered':'RIDDLE_GAME_4',
                                                 'not_answered':'RIDDLE_GAME_3_MISSED'})

            smach.StateMachine.add("RIDDLE_GAME_3_MISSED",
                                   Say(robot, "Please ask next question!"),
                                   transitions={"spoken": "RIDDLE_GAME_4"})
 
            smach.StateMachine.add('RIDDLE_GAME_4', 
                                   riddle_game.HearQuestion(robot), 
                                   transitions={ 'answered':'RIDDLE_GAME_5',
                                                 'not_answered':'RIDDLE_GAME_4_MISSED'})

            smach.StateMachine.add("RIDDLE_GAME_4_MISSED",
                                   Say(robot, "Please ask next question!"),
                                   transitions={"spoken": "RIDDLE_GAME_5"})

            smach.StateMachine.add('RIDDLE_GAME_5', 
                                   riddle_game.HearQuestion(robot), 
                                   transitions={ 'answered':'TRANSITION',
                                                 'not_answered':'TRANSITION'})

            # Transition:

            smach.StateMachine.add("TRANSITION",
                                   Say(robot, "Now lets play the blind mans bluff game"),
                                   transitions={"spoken": "BLUFF_GAME_1"})

            # Bluff Games:

            # 1         
            smach.StateMachine.add('BLUFF_GAME_1', 
                                   bluff_game.HearQuestion(robot), 
                                   transitions={'answered': 'BLUFF_GAME_2', 
                                                'not_answered': 'BLUFF_GAME_1_ASK_REPEAT'})
            
            smach.StateMachine.add("BLUFF_GAME_1_ASK_REPEAT",
                                   Say(robot, "Could you please repeat your question?"),
                                   transitions={"spoken": "BLUFF_GAME_1_REPEAT"})

            smach.StateMachine.add('BLUFF_GAME_1_REPEAT', 
                                   bluff_game.HearQuestionRepeat(robot), 
                                   transitions={'answered' :'BLUFF_GAME_2', 
                                                'not_answered': 'BLUFF_GAME_2'})
 
            # 2
            smach.StateMachine.add('BLUFF_GAME_2', 
                                   bluff_game.HearQuestion(robot), 
                                   transitions={'answered': 'BLUFF_GAME_3', 
                                                'not_answered': 'BLUFF_GAME_2_ASK_REPEAT'})
            
            smach.StateMachine.add("BLUFF_GAME_2_ASK_REPEAT",
                                   Say(robot, "Could you please repeat your question?"),
                                   transitions={"spoken": "BLUFF_GAME_2_REPEAT"})

            smach.StateMachine.add('BLUFF_GAME_2_REPEAT', 
                                   bluff_game.HearQuestionRepeat(robot), 
                                   transitions={'answered' :'BLUFF_GAME_3', 
                                                'not_answered': 'BLUFF_GAME_3'})
 
            # 3
            smach.StateMachine.add('BLUFF_GAME_3', 
                                   bluff_game.HearQuestion(robot), 
                                   transitions={'answered': 'BLUFF_GAME_4', 
                                                'not_answered': 'BLUFF_GAME_3_ASK_REPEAT'})
            
            smach.StateMachine.add("BLUFF_GAME_3_ASK_REPEAT",
                                   Say(robot, "Could you please repeat your question?"),
                                   transitions={"spoken": "BLUFF_GAME_3_REPEAT"})

            smach.StateMachine.add('BLUFF_GAME_3_REPEAT', 
                                   bluff_game.HearQuestionRepeat(robot), 
                                   transitions={'answered' :'BLUFF_GAME_4',
                                                'not_answered': 'BLUFF_GAME_4'})

            # 4
            smach.StateMachine.add('BLUFF_GAME_4', 
                                   bluff_game.HearQuestion(robot), 
                                   transitions={'answered': 'BLUFF_GAME_5', 
                                                'not_answered': 'BLUFF_GAME_4_ASK_REPEAT'})
            
            smach.StateMachine.add("BLUFF_GAME_4_ASK_REPEAT",
                                   Say(robot, "Could you please repeat your question?"),
                                   transitions={"spoken": "BLUFF_GAME_4_REPEAT"})

            smach.StateMachine.add('BLUFF_GAME_4_REPEAT', 
                                   bluff_game.HearQuestionRepeat(robot), 
                                   transitions={'answered' :'BLUFF_GAME_5', 
                                                'not_answered': 'BLUFF_GAME_5'})
 
            # 5
            smach.StateMachine.add('BLUFF_GAME_5', 
                                   bluff_game.HearQuestion(robot), 
                                   transitions={'answered': 'BLUFF_GAME_6', 
                                                'not_answered': 'BLUFF_GAME_5_ASK_REPEAT'})
            
            smach.StateMachine.add("BLUFF_GAME_5_ASK_REPEAT",
                                   Say(robot, "Could you please repeat your question?"),
                                   transitions={"spoken": "BLUFF_GAME_5_REPEAT"})

            smach.StateMachine.add('BLUFF_GAME_5_REPEAT', 
                                   bluff_game.HearQuestionRepeat(robot), 
                                   transitions={'answered' :'BLUFF_GAME_6', 
                                                'not_answered': 'BLUFF_GAME_6'})
  
            # 6
            smach.StateMachine.add('BLUFF_GAME_6', 
                                   bluff_game.HearQuestion(robot), 
                                   transitions={'answered': 'END_CHALLENGE', 
                                                'not_answered': 'BLUFF_GAME_6_ASK_REPEAT'})
            
            smach.StateMachine.add("BLUFF_GAME_6_ASK_REPEAT",
                                   Say(robot, "Could you please repeat your question?"),
                                   transitions={"spoken": "BLUFF_GAME_6_REPEAT"})

            smach.StateMachine.add('BLUFF_GAME_6_REPEAT', 
                                   bluff_game.HearQuestionRepeat(robot), 
                                   transitions={'answered' :'END_CHALLENGE', 
                                                'not_answered': 'END_CHALLENGE'})                                   
            

            # End

            smach.StateMachine.add('END_CHALLENGE',
                                   Say(robot, "My work here is done, goodbye!"),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "person_recognition")

if __name__ == "__main__":
    rospy.init_node('speech_person_recognition_exec')

    startup(ChallengeSpeechPersonRecognition, challenge_name="challenge_spr")
