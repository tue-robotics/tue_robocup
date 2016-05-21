#! /usr/bin/env python

import rospy
import smach
import robot_smach_states as states
import time

from robot_smach_states.util.startup import startup

import robot_smach_states.util.designators as ds

from person_recognition_states import LearnOperatorFace, Detect


class ChallengePersonRecognition(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        #  -----------------------------------------------------------------

        with self:
            smach.StateMachine.add( 'INITIALIZE',
                                    states.Initialize(robot),
                                    transitions={'initialized': 'LEARN_OPERATOR_FACE',
                                                 'abort': 'Aborted'})

            smach.StateMachine.add( 'LEARN_OPERATOR_FACE',
                                    LearnOperatorFace(robot),
                                    transitions={'succeeded': 'WAIT_FOR_OPERATOR_TO_JOIN',
                                                 'failed': 'WAIT_FOR_OPERATOR_TO_JOIN'})

            @smach.cb_interface(outcomes=['done'])
            def wait_a_sec(userdata):
                robot.speech.speak("I will wait for 10 seconds for you to join the crowd", block=False)
                time.sleep(10)
                return 'done'

            smach.StateMachine.add('WAIT_FOR_OPERATOR_TO_JOIN',
                                   smach.CBState(wait_a_sec),
                                   transitions={'done': 'FORCE_DRIVE'})

            @smach.cb_interface(outcomes=['done'])
            def force_drive(userdata):
                vth = 0.5
                th = 3.1415
                robot.head.cancel_goal()
                robot.base.force_drive(0, 0, vth, th / vth)
                return 'done'

            smach.StateMachine.add('FORCE_DRIVE',
                                   smach.CBState(force_drive),
                                   transitions={'done': 'DETECT'})

            smach.StateMachine.add( 'DETECT',
                                    Detect(robot),
                                    transitions={'succeeded': 'END_CHALLENGE',
                                                 'failed': 'DETECT'})

            smach.StateMachine.add('END_CHALLENGE',
                                   states.Say(robot, "My work here is done, goodbye!"),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "person_recognition")

if __name__ == "__main__":
    rospy.init_node('person_recognition_exec')

    startup(ChallengePersonRecognition, challenge_name="person_recognition")
