#! /usr/bin/env python

# ------------------------------------------------------------------------------------------------------------------------
# By Sam Aleksandrov, 2017
# ------------------------------------------------------------------------------------------------------------------------

import rospy
import smach
import time
import os
import datetime
import math

from robot_smach_states import Initialize, Say, StartChallengeRobust
from robot_smach_states.util.startup import startup
import robot_smach_states.util.designators as ds

# Still empty
#from challenge_set_a_table_states import set_table
#from challenge_set_a_table_states import clean_table
from challenge_set_a_table_states import fetch_command



class ChallengeSetATable(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:
            #Part I: Set a table
            smach.StateMachine.add('INITIALIZE',
                                   Initialize(robot),
                                   transitions={'initialized': 'ENTER_ROOM',
                                                'abort': 'Aborted'})    

            smach.StateMachine.add('ENTER_ROOM', # Enter the room
                                   StartChallengeRobust(robot, knowledge.initial_pose),
                                   transitions={'Done': 'ANNOUNCEMENT',
                                                'Aborted': 'Aborted',
                                                'Failed': 'Aborted'})

            smach.StateMachine.add('ANNOUNCEMENT',
                                   Say(robot, "Let's see if my master has a task for me! Moving to the meeting point.", 
                                    block=True),
                                   transitions={'spoken': 'NAVIGATE_TO_WAYPOINT_I'})
                                   
            smach.StateMachine.add('NAVIGATE_TO_WAYPOINT_I', 
                                   NavigateToWaypoint(robot, EntityByIdDesignator(robot=robot, id=knowledge.starting_pose), 
                                    radius=0.3),
                                   transitions={'done': 'FETCH_COMMAND_I',
                                                'abort': 'Aborted'})  

            smach.StateMachine.add('FETCH_COMMAND_I', # Hear "set the table"
                                   HearFetchCommand(robot, 15.0),
                                   transitions={'heard': 'ASK_FOR_MEAL'})

            smach.StateMachine.add('ASK_FOR_MEAL',
                                   Say(robot, "What should I serve, master?", block=True),
                                   transitions={'spoken': 'SET_THE_TABLE'})
                                   
            smach.StateMachine.add('SET_THE_TABLE', # Take order and Set the table (bring the objects to the table)
                                   Initialize(robot),
                                   transitions={'initialized': 'SERVE_MEAL',
                                                'abort': 'Aborted'},
                                                remapping={'meal': 'meal'})

            smach.StateMachine.add('SERVE_MEAL', # Serve the meal (for example: pour milk into the bowl)
                                   Initialize(robot),
                                   transitions={'initialized': 'CORRECT_OBJECT_POSITIONS',
                                                'abort': 'Aborted'})

            smach.StateMachine.add('CORRECT_OBJECT_POSITIONS', # Inspect table and correct the moved objects
                                   Initialize(robot),
                                   transitions={'initialized': 'ANNOUNCE_TASK_COMPLETION',
                                                'abort': 'Aborted'})

            smach.StateMachine.add('ANNOUNCE_TASK_COMPLETION',
                                   Say(robot, "The table is set! Moving to the meeting point for the next task.", 
                                    block=True),
                                   transitions={'spoken': 'NAVIGATE_TO_WAYPOINT_II'})
                                   
            

            #Part II: Clean the table
            smach.StateMachine.add('NAVIGATE_TO_WAYPOINT_II', 
                                   NavigateToWaypoint(robot, EntityByIdDesignator(robot=robot, id=knowledge.starting_pose), 
                                    radius=0.3),
                                   transitions={'done': 'FETCH_COMMAND_II',
                                                'abort': 'Aborted'})  

            smach.StateMachine.add('FETCH_COMMAND_II', # Hear "clear up the table"
                                   HearFetchCommand(robot, 15.0),
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
