#!/usr/bin/python
import roslib;
import rospy
import smach
import sys

import robot_smach_states as states
from robot_smach_states.util.designators import Designator, EdEntityDesignator

from robocup_knowledge import load_knowledge
data = load_knowledge('challenge_speech_recognition')

class HearQuestion(smach.State):
    def __init__(self, robot, time_out=rospy.Duration(15)):
        smach.State.__init__(self, outcomes=["answered"])
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
            else:
                self.robot.speech.speak("Sorry, I do not understand your question")

        return "answered"


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:

        # Start challenge via StartChallengeRobust
        smach.StateMachine.add( "START_CHALLENGE_ROBUST",
                                states.Initialize(robot),
                                transitions={   'initialized'      :   "SAY_1",
                                                "abort"            :   "Aborted"})

        smach.StateMachine.add('SAY_1', states.Say(robot, "Please ask me question one"), transitions={ 'spoken' :'QUESTION_1'})
        smach.StateMachine.add('QUESTION_1', HearQuestion(robot), transitions={ 'answered' :'SAY_2'})
        smach.StateMachine.add('SAY_2', states.Say(robot, "Please ask me question two"), transitions={ 'spoken' :'QUESTION_2'})
        smach.StateMachine.add('QUESTION_2', HearQuestion(robot), transitions={ 'answered' :'SAY_3'})
        smach.StateMachine.add('SAY_3', states.Say(robot, "Please ask me question three"), transitions={ 'spoken' :'QUESTION_3'})
        smach.StateMachine.add('QUESTION_3', HearQuestion(robot), transitions={ 'answered' :'SAY_4'})
        smach.StateMachine.add('SAY_4', states.Say(robot, "Please ask me question four"), transitions={ 'spoken' :'QUESTION_4'})
        smach.StateMachine.add('QUESTION_4', HearQuestion(robot), transitions={ 'answered' :'SAY_5'})
        smach.StateMachine.add('SAY_5', states.Say(robot, "Please ask me question five"), transitions={ 'spoken' :'QUESTION_5'})
        smach.StateMachine.add('QUESTION_5', HearQuestion(robot), transitions={ 'answered' :'AT_END'})

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
