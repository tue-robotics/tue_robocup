#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_open_2014')
import rospy, sys

import smach

from robot_skills.reasoner  import Conjunction, Compound

from math import cos, sin
from geometry_msgs.msg import *

from robot_skills.amigo import Amigo
import robot_smach_states as states

from speech_interpreter.srv import AskUser

from navigation_interface import Navigation

class AskOpenChallenge(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.ask_user_service = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata=None):

        self.robot.head.look_up()
        
        try:
            self.response = self.ask_user_service("challenge_open_2014", 4 , rospy.Duration(60))  # This means that within 4 tries and within 60 seconds an answer is received. 
            
            for x in range(0,len(self.response.keys)):
                if self.response.keys[x] == "answer":
                    response_answer = self.response.values[x]

            if response_answer == "no_answer" or response_answer == "wrong_answer" or response_answer == "":
                self.robot.speech.speak("I was not able to understand you but I'll drive to table.")
                target = "table"
            else:
                target = response_answer

        except Exception, e:
            target = "table"
            self.robot.speech.speak("There is something wrong with my ears, I will go to %s"%target)

        self.robot.base2.pc.constraint = 'x^2 + y^2 < 1.2^2'
        self.robot.base2.pc.frame      = target

        self.robot.base2.oc.look_at    = Point()
        self.robot.base2.oc.frame      = target

        return "done"


#######################################################################################################################################################################################################

class OpenChallenge2014(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:

            smach.StateMachine.add("ASK_OPENCHALLENGE",
                                AskOpenChallenge(robot),
                                transitions={'done'             :   'NAVIGATE_TO_TARGET'})

            smach.StateMachine.add("NAVIGATE_TO_TARGET",
                                Navigation.NavigateWithConstraints(robot),
                                transitions={'arrived'          :   'ASK_OPENCHALLENGE',
                                             'unreachable'      :   'ASK_OPENCHALLENGE',
                                             'goal_not_defined' :   'ASK_OPENCHALLENGE'})

if __name__ == "__main__":
    rospy.init_node('open_challenge_2014')
    states.util.startup(OpenChallenge2014)
