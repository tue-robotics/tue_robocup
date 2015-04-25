#!/usr/bin/python

import rospy
import smach
import sys
import random
import math

from robot_smach_states.util.designators import *
import robot_smach_states as states
from robot_smach_states.util.startup import startup

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_final')
INITIAL_POSE = challenge_knowledge.initial_pose_amigo

class Ask_what_do_i_see(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "failed"])
        self.robot = robot

    def execute(self, userdata):

        self.robot.speech.speak("What do I see here?")

        res = self.robot.ears.recognize(spec=challenge_knowledge.mesh_spec, choices=challenge_knowledge.mesh_choices, time_out = rospy.Duration(20))
        if not res:
            self.robot.speech.speak("My ears are not working properly, can i get a restart?.")
            return "failed"
        try:
            if res.result:
                self.robot.speech.speak("Okay")
                name_object = res.choices['object']
                print "name_object = ", name_object
                # JANNO, hier kun je mee doen wat je er mee wilt.
                return "done"
            else:
                self.robot.speech.speak("Sorry, could you please repeat?")
                return "failed"
        except KeyError:
            print "KEYERROR FINAL, should not happen!"
            return "failed"


############################## main statemachine ######################
def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:
        smach.StateMachine.add("INITIALIZE",
                                states.StartChallengeRobust(robot, INITIAL_POSE, use_entry_points = True),
                                transitions={   "Done"              :   "WAIT_FOR_TRIGGER_TO_START",
                                                "Aborted"           :   "WAIT_FOR_TRIGGER_TO_START",
                                                "Failed"            :   "WAIT_FOR_TRIGGER_TO_START"})

        smach.StateMachine.add("WAIT_FOR_TRIGGER_TO_START", 
                                    states.WaitForTrigger(robot, ['amigo_trigger'],"/amigo/trigger"),
                                    transitions={   'amigo_trigger':'SAY_YES',
                                                    'preempted'  :'SAY_YES'})

        smach.StateMachine.add( 'SAY_YES',
                                states.Say(robot, ["I have arrived at the desired location."], block=True),
                                transitions={'spoken':'ASK_WHICH_PILLS'})     

        smach.StateMachine.add("ASK_WHICH_PILLS",
                                    Ask_what_do_i_see(robot),
                                    transitions={'done':'Done',
                                                'failed':'Done'})

    return sm

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('final_exec_amigo')

    startup(setup_statemachine, robot_name='amigo')