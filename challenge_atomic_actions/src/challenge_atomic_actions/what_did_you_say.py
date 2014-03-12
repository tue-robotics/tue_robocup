#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_atomic_actions')
import rospy
#import robot_parts.speech
from std_msgs.msg import String
import geometry_msgs
import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states
import robot_skills.util.msg_constructors as msgs

from robot_smach_states.util.startup import startup

from speech_interpreter.srv import AskUser # for speech_to_text only

from psi import *

###########################
# Created by: Erik Geerts #
###########################

class AskQuestions(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self.robot = robot
        self.ask_user_service_questions = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata=None):

        self.robot.head.look_up()

        rospy.loginfo("----Possible questions for now: -----------------")
        rospy.loginfo("--- What is the capital of Germany? -------------")
        rospy.loginfo("--- What is the heaviest animal in the world?----")
        rospy.loginfo("--- Who is the president of America?-------------")
        rospy.loginfo("--- Who is your example?-------------------------")
        rospy.loginfo("--- When do the olympics start?------------------")
        rospy.loginfo("--- Which football club is the best?-------------")
        rospy.loginfo("--- Who is the best looking person around here?--")
        rospy.loginfo("--- Which person is not able to say yes?---------")
        rospy.loginfo("--- Which town has been bombed?------------------")
        rospy.loginfo("--- What is your motto?--------------------------")


        # Here you can define how many times you want to try to listen and want the maximum duration is to listen to operator.
        self.response = self.ask_user_service_questions("questions", 10 , rospy.Duration(60))

        if self.response.keys[0] == "answer":
            return "succeeded"
        else:
            return "failed"


class WhatDidYouSay(smach.StateMachine):

    def __init__(self, robot=None):
        smach.StateMachine.__init__(self, outcomes=["Done"])
        self.robot = robot

        rospy.loginfo("-------------------- WHAT DIT YOU SAY ---------------------")
        rospy.loginfo("-----------------------------------------------------------")
        rospy.loginfo("------------ MAKE SURE YOU ADDED THE SENTENCES ------------")
        rospy.loginfo("-------------- speech_interpreter.pl in map ---------------")
        rospy.loginfo("----------- tue_reasoner/tue_knowledge/prolog -------------")
        rospy.loginfo("-------- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -----------")
        rospy.loginfo("-----------------------------------------------------------")
            
        with self:

            smach.StateMachine.add("SAY_FIRST_QUESTION",
                                    states.Say(robot,"What is your first question?"),
                                    transitions={'spoken':'ASK_FIRST_QUESTION'})

            smach.StateMachine.add("ASK_FIRST_QUESTION",
                                    AskQuestions(robot),
                                    transitions={'succeeded':'SAY_SECOND_QUESTION',
                                                 'failed':'ASK_FIRST_QUESTION'})

            smach.StateMachine.add("SAY_SECOND_QUESTION",
                                    states.Say(robot,"What is your second question?"),
                                    transitions={'spoken':'ASK_SECOND_QUESTION'})

            smach.StateMachine.add("ASK_SECOND_QUESTION",
                                    AskQuestions(robot),
                                    transitions={'succeeded':'SAY_THIRD_QUESTION',
                                                 'failed':'ASK_SECOND_QUESTION'})

            smach.StateMachine.add("SAY_THIRD_QUESTION",
                                    states.Say(robot,"What is your third question?"),
                                    transitions={'spoken':'ASK_THIRD_QUESTION'})

            smach.StateMachine.add("ASK_THIRD_QUESTION",
                                    AskQuestions(robot),
                                    transitions={'succeeded':'Done',
                                                 'failed':'ASK_THIRD_QUESTION'})

if __name__ == "__main__":
    rospy.init_node('what_did_you_say_exec')
    startup(WhatDidYouSay)