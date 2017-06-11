#!/usr/bin/python
import roslib;
import rospy
import smach
import sys
import random
import math
import time

from robot_skills.util import transformations as tf
from robot_skills.util import transformations, msg_constructors

import robot_smach_states as states
from robot_smach_states.util.designators import Designator, EdEntityDesignator

from robocup_knowledge import load_knowledge
data = load_knowledge('challenge_speech_recognition')

class HearQuestion(smach.State):
    def __init__(self, robot, time_out=rospy.Duration(15)):
        smach.State.__init__(self, outcomes=["answered", "not_answered"])
        self.robot = robot
        self.time_out = time_out

    def execute(self, userdata=None):
        self.robot.head.look_at_ground_in_front_of_robot(100)

        res = self.robot.ears.recognize(spec=data.spec, choices=data.choices, time_out=self.time_out)

        if not res:
            self.robot.speech.speak("My ears are not working properly.")
        if res:
            if "question" in res.choices:
                rospy.loginfo("Question was: '%s'?"%res.result)
                self.robot.speech.speak("The answer is %s"%data.choice_answer_mapping[res.choices['question']])

                return "answered"
            else:
                self.robot.speech.speak("Sorry, I do not understand your question")

        return "not_answered"


class Turn(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["turned"])
        self.robot = robot

    def _turn_to_closest_entity(self):
        # Reset the world model just to be sure
        self.robot.ed.reset()

        operator = None
        while not operator:
            operator = self.robot.ed.get_closest_entity(self, radius=1.9,
                                                        center_point=self.robot.base.get_location().extractVectorStamped())
            print operator
            if not operator:
                vth = 0.5
                th = 3.1415 / 10
                print "Turning %f radians with force drive" % th
                self.robot.base.force_drive(0, 0, vth, th / vth)

        self.robot.base.force_drive(0, 0, 0, 0.5)

        # Turn towards the operator
        current = self.robot.base.get_location()
        robot_th = current.frame.M.GetRPY()[2]  # Get the Yaw, rotation around Z
        desired_th = math.atan2(operator._pose.p.y() - current.frame.p.y(),
                                operator._pose.p.x() - current.frame.p.x())

        # Calculate params
        th = desired_th - robot_th
        if th > 3.1415:
            th -= 2 * 3.1415
        if th < -3.1415:
            th += 2 * 3.1415
        vth = 0.5

        # TUrn
        self.robot.base.force_drive(0, 0, (th / abs(th)) * vth, abs(th) / vth)

    def execute(self, userdata=None):

        print "Last talker id: " + self.robot.hmi.last_talker_id

        # Calculate params
        if "dragonfly_speech_recognition" not in self.robot.hmi.last_talker_id:
            # TUrn
            vth = 0.5
            th = 3.1415
            self.robot.base.force_drive(0, 0, (th / abs(th)) * vth, abs(th) / vth)

        self._turn_to_closest_entity()

        self.robot.speech.speak(random.choice(["There you are!",
                                               "Hi there!",
                                               "I think the sound is coming from this direction"]))

        time.sleep(1.0)

        return "turned"


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:

        # Start challenge via StartChallengeRobust
        smach.StateMachine.add( "START_CHALLENGE_ROBUST",
                                states.Initialize(robot),
                                transitions={   'initialized'      :   "SAY_1",
                                                "abort"            :   "Aborted"})

        smach.StateMachine.add('SAY_1', states.Say(robot, "Please ask me question one"), transitions={ 'spoken' :'QUESTION_1'})
        smach.StateMachine.add('QUESTION_1', HearQuestion(robot), transitions={ 'answered' :'SAY_2', 'not_answered': 'SAY_2'})
        smach.StateMachine.add('TURN_1', Turn(robot), transitions={ 'turned' :'SAY_1A'})
        smach.StateMachine.add('SAY_1A', states.Say(robot, "Please repeat your question"), transitions={ 'spoken' :'QUESTION_1A'})
        smach.StateMachine.add('QUESTION_1A', HearQuestion(robot), transitions={ 'answered' :'SAY_2', 'not_answered': 'SAY_2'})

        smach.StateMachine.add('SAY_2', states.Say(robot, "Please ask me question two"), transitions={ 'spoken' :'QUESTION_2'})
        smach.StateMachine.add('QUESTION_2', HearQuestion(robot), transitions={ 'answered' :'SAY_3', 'not_answered': 'SAY_3'})
        smach.StateMachine.add('TURN_2', Turn(robot), transitions={ 'turned' :'SAY_2A'})
        smach.StateMachine.add('SAY_2A', states.Say(robot, "Please repeat your question"), transitions={ 'spoken' :'QUESTION_2A'})
        smach.StateMachine.add('QUESTION_2A', HearQuestion(robot), transitions={ 'answered' :'SAY_3', 'not_answered': 'SAY_3'})

        smach.StateMachine.add('SAY_3', states.Say(robot, "Please ask me question three"), transitions={ 'spoken' :'QUESTION_3'})
        smach.StateMachine.add('QUESTION_3', HearQuestion(robot), transitions={ 'answered' :'SAY_4', 'not_answered': 'SAY_4'})
        smach.StateMachine.add('TURN_3', Turn(robot), transitions={ 'turned' :'SAY_3A'})
        smach.StateMachine.add('SAY_3A', states.Say(robot, "Please repeat your question"), transitions={ 'spoken' :'QUESTION_3A'})
        smach.StateMachine.add('QUESTION_3A', HearQuestion(robot), transitions={ 'answered' :'SAY_4', 'not_answered': 'SAY_4'})

        smach.StateMachine.add('SAY_4', states.Say(robot, "Please ask me question four"), transitions={ 'spoken' :'QUESTION_4'})
        smach.StateMachine.add('QUESTION_4', HearQuestion(robot), transitions={ 'answered' :'SAY_5', 'not_answered': 'SAY_5'})
        smach.StateMachine.add('TURN_4', Turn(robot), transitions={ 'turned' :'SAY_4A'})
        smach.StateMachine.add('SAY_4A', states.Say(robot, "Please repeat your question"), transitions={ 'spoken' :'QUESTION_4A'})
        smach.StateMachine.add('QUESTION_4A', HearQuestion(robot), transitions={ 'answered' :'SAY_5', 'not_answered': 'SAY_5'})

        smach.StateMachine.add('SAY_5', states.Say(robot, "Please ask me question five"), transitions={ 'spoken' :'QUESTION_5'})
        smach.StateMachine.add('QUESTION_5', HearQuestion(robot), transitions={ 'answered' :'AT_END', 'not_answered': 'AT_END'})
        smach.StateMachine.add('TURN_5', Turn(robot), transitions={ 'turned' :'SAY_5A'})
        smach.StateMachine.add('SAY_5A', states.Say(robot, "Please repeat your question"), transitions={ 'spoken' :'QUESTION_5A'})
        smach.StateMachine.add('QUESTION_5A', HearQuestion(robot), transitions={ 'answered' :'AT_END', 'not_answered': 'AT_END'})

        smach.StateMachine.add('AT_END', states.Say(robot, "That was all folks!"), transitions={ 'spoken' :'Done'})
    return sm


############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('challenge_speech_recognition_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE SPEECH RECOGNITION] Please provide robot name as argument."
        exit(1)

    states.util.startup(setup_statemachine, robot_name=robot_name)
