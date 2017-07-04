#! /usr/bin/env python

import rospy
import smach
import time
import os
import datetime
import math

from robot_smach_states import Initialize, Say
from robot_smach_states.util.startup import startup
import robot_smach_states.util.designators as ds

# Still empty
#from challenge_set_a_table_states import clean_table
from challenge_set_a_table_states import fetch_command
#from challenge_set_a_table_states import set_table


class ChallengeSetATable(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:
            #Part I: Set a table
            smach.StateMachine.add('INITIALIZE',
                                   Initialize(robot),
                                   transitions={'initialized': 'FETCH_COMMAND_I',
                                                'abort': 'Aborted'})            
            smach.StateMachine.add('FETCH_COMMAND_I', # Hear "set the table"
                                   fetch(robot),
                                   transitions={'heard': 'ASK_FOR_MEAL'})

            smach.StateMachine.add('ASK_FOR_MEAL',
                                   Say(robot, "What should I serve, master?"),
                                   transitions={'spoken': 'SET_THE_TABLE'})
#save the order
            smach.StateMachine.add('SET_THE_TABLE', # Set the table (bring the objects to the table)
                                   Initialize(robot),
                                   transitions={'initialized': 'SERVE_MEAL',
                                                'abort': 'Aborted'})
            smach.StateMachine.add('SERVE_MEAL', # Serve the meal (pour milk into the bowl)
                                   Initialize(robot),
                                   transitions={'initialized': 'CORRECT_OBJECT_POSITIONS',
                                                'abort': 'Aborted'})
            smach.StateMachine.add('CORRECT_OBJECT_POSITIONS', # Inspect table and correct the moved objects
                                   Initialize(robot),
                                   transitions={'initialized': 'FETCH_COMMAND_II',
                                                'abort': 'Aborted'})

            #Part II: Clean the table
            smach.StateMachine.add('FETCH_COMMAND_II', # Hear "clear up the table"
                                   fetch(robot),
                                   transitions={'heard': 'CLEAR_UP'})

            smach.StateMachine.add('CLEAR_UP', # Clear up the table (bring the objects to their default location)
                                   Initialize(robot),
                                   transitions={'initialized': 'CLEAN_THE_TABLE',
                                                'abort': 'Aborted'})
            smach.StateMachine.add('CLEAN_THE_TABLE', # Inspect for spots and spills and clean them
                                   Initialize(robot),
                                   transitions={'initialized': 'END_CHALLENGE',
                                                'abort': 'Aborted'})
            
            # End
            smach.StateMachine.add('END_CHALLENGE',
                                   Say(robot, "I am finally free!"),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "set_a_table")

if __name__ == "__main__":
    rospy.init_node('set_a_table_exec')

    startup(ChallengeSetATable, challenge_name="challenge_set_a_table")
