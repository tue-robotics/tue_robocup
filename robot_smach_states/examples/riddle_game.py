#!/usr/bin/python

# ROS
import rospy
import smach

# TU/e Robotics
import robot_smach_states as states
from robot_smach_states.human_interaction.answer_questions import HearAndAnswerQuestions
from robot_smach_states.util.startup import startup
from robocup_knowledge import load_knowledge
knowledge = load_knowledge('challenge_spr')
common_knowledge = load_knowledge('common')


class TestRiddleGame(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        self.userdata.crowd_data = {
            "males": 3,
            "men": 2,
            "females": 5,
            "women": 3,
            "children": 3,
            "boys": 1,
            "girls": 2,
            "adults": 5,
            "elders": 1,
            "crowd_size": 8
        }

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'RIDDLE_GAME',
                                                'abort': 'Aborted'})

            smach.StateMachine.add("RIDDLE_GAME",
                                   HearAndAnswerQuestions(
                                       robot,
                                       grammar=knowledge.grammar,
                                       knowledge=common_knowledge,
                                       num_questions=3,
                                   ),
                                   transitions={'done': 'Done'},
                                   remapping={'crowd_data':'crowd_data'})


if __name__ == "__main__":
    rospy.init_node('speech_person_recognition_exec')

    startup(TestRiddleGame, challenge_name="challenge_spr")
