#!/usr/bin/python

import threading
import rospy
import smach
import sys
import math
import time

from robot_smach_states.util.startup import startup

from robot_smach_states import Initialize

from riddle_game import hear, answer

##############################################################################
#
# Default parameters:
#
##############################################################################

DEFAULT_HEAR_TIME = 20.0        # seconds
SOURCE_LOCALISATION_TIME = 6    # seconds
INITIAL_TURNING_DELAY = 1.5     # seconds

##############################################################################
#
# Main class:
#
##############################################################################

class HearTurnAndAnswerQuestions(smach.State):
    '''
    Robot hears a question, turn and then answers (bluff game in SPR challenge).
    If the question is not answered, robot asks the operator to repeat the question and tries again, without turning.

    Variables:
        num_questions: number of questions to be heard and answered
        num_operators: number of operators to play the game
        hear_time: amount of time to hear a single question

    Outputs:
        done: answered all questions
    '''
    def __init__(self, robot, num_questions=1, num_operators=5, hear_time=DEFAULT_HEAR_TIME):
        smach.State.__init__(self, outcomes=["done"], input_keys=['crowd_data'])
        self.robot = robot
        self.num_questions = num_questions
        self.num_operators = num_operators
        self.hear_time = hear_time

    def execute(self, userdata):
        crowd_data = userdata.crowd_data

        self.robot.head.look_at_standing_person()

        for _ in xrange(self.num_questions):

            t = threading.Thread(target=turn_to_closest_entity, args=(self.robot,self.num_operators))
            t.start()

            res = hear(self.robot, hear_time=self.hear_time)

            t.join()

            if not answer(self.robot, res, crowd_data):
                self.robot.speech.speak("Could you please repeat your question?")

                res = hear(self.robot, hear_time=self.hear_time)

                if not answer(self.robot, res, crowd_data):
                    self.robot.speech.speak("Please ask next question!")

        return "done"

##############################################################################
#
# Functions:
#
##############################################################################

def turn_to_closest_entity(robot, num_operators):

    rospy.sleep(INITIAL_TURNING_DELAY)

    start = rospy.Time.now()
    yaw = None

    # Try to find a source for a couple of seconds
    while not yaw and (rospy.Time.now() - start).to_sec() < SOURCE_LOCALISATION_TIME:
        yaw = robot.ssl.get_last_yaw(.1)
        rospy.sleep(0.05)

    # If we did not find a yaw, just default
    if yaw is None:
        yaw = math.pi * 2 / num_operators

    if yaw > math.pi:
        yaw -= 2 * math.pi
    if yaw < -math.pi:
        yaw += 2 * math.pi

    rospy.loginfo("I should turn %.2f rad now", yaw)

    vyaw = 1.0
    robot.base.force_drive(0, 0, (yaw / abs(yaw)) * vyaw, abs(yaw) / vyaw)

##############################################################################
#
# Standalone testing:
#
##############################################################################

class TestBluffGame(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        self.userdata.crowd_data = {
            "males": 1,
            "men": 2,
            "females": 3,
            "women": 4,
            "children": 5,
            "boys": 6,
            "girls": 7,
            "adults": 8,
            "elders": 9,
            "crowd_size": 10
        }

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   Initialize(robot),
                                   transitions={'initialized': 'BLUFF_GAME',
                                                'abort': 'Aborted'})

            smach.StateMachine.add('BLUFF_GAME',
                                   HearTurnAndAnswerQuestions(robot, num_questions=3),
                                   transitions={'done': 'Done'},
                                   remapping={'crowd_data':'crowd_data'})

if __name__ == "__main__":
    rospy.init_node('speech_person_recognition_exec')

    startup(TestBluffGame, challenge_name="challenge_spr")
