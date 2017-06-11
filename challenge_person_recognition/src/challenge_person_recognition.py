#! /usr/bin/env python

import rospy
import smach
import robot_smach_states as states
import time
import os
import datetime

from robot_smach_states.util.startup import startup

import robot_smach_states.util.designators as ds

from person_recognition_states import LearnOperatorFace, Detect


class ChallengePersonRecognition(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        #  -----------------------------------------------------------------

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'INSTRUCT_WAIT_FOR_DOOR',
                                                'abort': 'Aborted'})

            smach.StateMachine.add("INSTRUCT_WAIT_FOR_DOOR",
                                   states.Say(robot, ["Hi there, I will now wait until you remove the cup",
                                                      "I'm waiting for you to remove the cup"], block=False),
                                   transitions={"spoken": "WAIT_FOR_DOOR"})

            smach.StateMachine.add("WAIT_FOR_DOOR",
                                   states.WaitForDoorOpen(robot, timeout=10),
                                   transitions={"closed": "DOOR_CLOSED",
                                                "open": "AWAIT_START"})

            smach.StateMachine.add("DOOR_CLOSED",
                                   states.Say(robot, ["I am waiting for you to remove the cup",
                                                      "I'd start, if you remove the cup from my laser"]),
                                   transitions={"spoken": "WAIT_FOR_DOOR"})

            smach.StateMachine.add("AWAIT_START",
                                   states.AskContinue(robot),
                                   transitions={'continue': 'LEARN_OPERATOR_FACE',
                                                'no_response': 'AWAIT_START'})

            smach.StateMachine.add( 'LEARN_OPERATOR_FACE',
                                    LearnOperatorFace(robot),
                                    transitions={'succeeded': 'WAIT_FOR_OPERATOR_TO_JOIN',
                                                 'failed': 'LEARN_OPERATOR_FACE'})

            @smach.cb_interface(outcomes=['done'])
            def wait_a_sec(userdata=None):
                robot.speech.speak("I will wait for 10 seconds for you to join the crowd", block=True)
                start = rospy.Time.now()
                stop = rospy.Duration(10) + start

                last_spoken = start
                while rospy.Time.now() < stop:
                    if (rospy.Time.now() - last_spoken).to_sec() > 1.0:
                        robot.speech.speak("%d" % (stop - rospy.Time.now()).to_sec())
                        last_spoken = rospy.Time.now()
                return 'done'

            smach.StateMachine.add('WAIT_FOR_OPERATOR_TO_JOIN',
                                   smach.CBState(wait_a_sec),
                                   transitions={'done': 'FORCE_DRIVE'})

            @smach.cb_interface(outcomes=['done'])
            def force_drive(userdata=None):
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
    cmd = 'convert `ls -t /tmp/faces/*_annotated.jpeg | head -1` /home/amigo/usb/tech_united_%s.pdf' % datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")

    rospy.loginfo("Executing bash command: %s" % cmd)
    os.system(cmd)

    rospy.loginfo("Listing files on USB:")
    os.system("ls -lah /home/amigo/usb")

    rospy.loginfo("If this went wrong for a reason, please execute the command on amigo1 to create a pdf and copy to USB manually")

    cmd = 'convert `ls -t /tmp/faces/*_annotated.jpeg | head -1` /home/amigo/tech_united_%s.pdf' % datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")

    rospy.loginfo("Executing bash command: %s" % cmd)
    os.system(cmd)
