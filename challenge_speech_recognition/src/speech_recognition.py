#!/usr/bin/python
import roslib;
import rospy
import smach
import sys
import random

import robot_smach_states as states
from robot_smach_states.util.designators import Designator, EdEntityDesignator, EntityByIdDesignator, analyse_designators
from geometry_msgs.msg import PointStamped

import direct_speech_recognition
import indirect_speech_recognition

def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:

        # Start challenge via StartChallengeRobust
        smach.StateMachine.add( "START_CHALLENGE_ROBUST",
                                states.Initialize(robot),
                                transitions={   'initialized'      :   "SAY_INTRO",
                                                "abort"            :   "Aborted"})

        smach.StateMachine.add('SAY_INTRO', states.Say(robot, "Please come a little bit closer to me, talk loudly and, very important, wait for the ping!"), transitions={ 'spoken' :'SAY_1'})

        smach.StateMachine.add('SAY_1', states.Say(robot, "Please ask me question one"), transitions={ 'spoken' :'QUESTION_1'})
        smach.StateMachine.add('QUESTION_1', direct_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'SAY_2'})
        smach.StateMachine.add('SAY_2', states.Say(robot, "Please ask me question two"), transitions={ 'spoken' :'QUESTION_2'})
        smach.StateMachine.add('QUESTION_2', direct_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'SAY_3'})
        smach.StateMachine.add('SAY_3', states.Say(robot, "Please ask me question three"), transitions={ 'spoken' :'QUESTION_3'})
        smach.StateMachine.add('QUESTION_3', direct_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'SAY_4'})
        smach.StateMachine.add('SAY_4', states.Say(robot, "Please ask me question four"), transitions={ 'spoken' :'QUESTION_4'})
        smach.StateMachine.add('QUESTION_4', direct_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'SAY_5'})
        smach.StateMachine.add('SAY_5', states.Say(robot, "Please ask me question five"), transitions={ 'spoken' :'QUESTION_5'})
        smach.StateMachine.add('QUESTION_5', direct_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'LOOK_UP'})

        @smach.cb_interface(outcomes=['done'])
        def look_up(userdata):
            robot.head.look_at_standing_person()

            return 'done'

        smach.StateMachine.add('LOOK_UP', smach.CBState(look_up), transitions={'done': 'TRANSITION'})

        smach.StateMachine.add('TRANSITION', states.Say(robot, "Let's proceed to round two! yeah!"), transitions={ 'spoken' :'2SAY_1'})

        smach.StateMachine.add('2SAY_1', states.Say(robot, "Please ask me question one"), transitions={ 'spoken' :'2QUESTION_1'})
        smach.StateMachine.add('2QUESTION_1', indirect_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'2SAY_2', 'not_answered': '2SAY_2'})
        smach.StateMachine.add('2TURN_1', indirect_speech_recognition.Turn(robot), transitions={ 'turned' :'2SAY_1A'})
        smach.StateMachine.add('2SAY_1A', states.Say(robot, "Please repeat your question"), transitions={ 'spoken' :'2QUESTION_1A'})
        smach.StateMachine.add('2QUESTION_1A', indirect_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'2SAY_2', 'not_answered': '2SAY_2'})

        smach.StateMachine.add('2SAY_2', states.Say(robot, "Please ask me question two"), transitions={ 'spoken' :'2QUESTION_2'})
        smach.StateMachine.add('2QUESTION_2', indirect_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'2SAY_3', 'not_answered': '2SAY_3'})
        smach.StateMachine.add('2TURN_2', indirect_speech_recognition.Turn(robot), transitions={ 'turned' :'2SAY_2A'})
        smach.StateMachine.add('2SAY_2A', states.Say(robot, "Please repeat your question"), transitions={ 'spoken' :'2QUESTION_2A'})
        smach.StateMachine.add('2QUESTION_2A', indirect_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'2SAY_3', 'not_answered': '2SAY_3'})

        smach.StateMachine.add('2SAY_3', states.Say(robot, "Please ask me question three"), transitions={ 'spoken' :'2QUESTION_3'})
        smach.StateMachine.add('2QUESTION_3', indirect_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'2SAY_4', 'not_answered': '2SAY_4'})
        smach.StateMachine.add('2TURN_3', indirect_speech_recognition.Turn(robot), transitions={ 'turned' :'2SAY_3A'})
        smach.StateMachine.add('2SAY_3A', states.Say(robot, "Please repeat your question"), transitions={ 'spoken' :'2QUESTION_3A'})
        smach.StateMachine.add('2QUESTION_3A', indirect_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'2SAY_4', 'not_answered': '2SAY_4'})

        smach.StateMachine.add('2SAY_4', states.Say(robot, "Please ask me question four"), transitions={ 'spoken' :'2QUESTION_4'})
        smach.StateMachine.add('2QUESTION_4', indirect_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'2SAY_5', 'not_answered': '2SAY_5'})
        smach.StateMachine.add('2TURN_4', indirect_speech_recognition.Turn(robot), transitions={ 'turned' :'2SAY_4A'})
        smach.StateMachine.add('2SAY_4A', states.Say(robot, "Please repeat your question"), transitions={ 'spoken' :'2QUESTION_4A'})
        smach.StateMachine.add('2QUESTION_4A', indirect_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'2SAY_5', 'not_answered': '2SAY_5'})

        smach.StateMachine.add('2SAY_5', states.Say(robot, "Please ask me question five"), transitions={ 'spoken' :'2QUESTION_5'})
        smach.StateMachine.add('2QUESTION_5', indirect_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'AT_END', 'not_answered': 'AT_END'})
        smach.StateMachine.add('2TURN_5', indirect_speech_recognition.Turn(robot), transitions={ 'turned' :'2SAY_5A'})
        smach.StateMachine.add('2SAY_5A', states.Say(robot, "Please repeat your question"), transitions={ 'spoken' :'2QUESTION_5A'})
        smach.StateMachine.add('2QUESTION_5A', indirect_speech_recognition.HearQuestion(robot), transitions={ 'answered' :'AT_END', 'not_answered': 'AT_END'})

        smach.StateMachine.add('AT_END', states.Say(robot, "That was all folks!"), transitions={ 'spoken' :'Done'})

    analyse_designators(sm, "speech_recognition")
    return sm


############################## initializing program ##############################
if __name__ == '__main__':
    rospy.init_node('challenge_speech_recognition_exec')

    states.util.startup(setup_statemachine, challenge_name="speech_recognition")
