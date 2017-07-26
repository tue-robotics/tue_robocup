#! /usr/bin/env python

# ------------------------------------------------------------------------------------------------------------------------
# By Sam Aleksandrov, 2017
# ------------------------------------------------------------------------------------------------------------------------

# System

# ROS
import rospy
import smach

# TU/e Robotics
from robocup_knowledge import load_knowledge
import robot_smach_states as states
from robot_smach_states.util.startup import startup
import robot_smach_states.util.designators as ds

# Set the table
# from challenge_set_a_table_states import set_table
# from challenge_set_a_table_states import clean_table
from challenge_set_a_table_states.fetch_command import HearFetchCommand, GetBreakfastOrder
from challenge_set_a_table_states.manipulate_machine import ManipulateMachine

# Load all knowledge
knowledge = load_knowledge('challenge_set_a_table')


class ChallengeSetATable(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        with self:
            # Part I: Set a table
            smach.StateMachine.add('ENTER_ROOM',  # Enter the room
                                   states.StartChallengeRobust(robot, knowledge.initial_pose),
                                   transitions={'Done': 'ANNOUNCEMENT',
                                                'Aborted': 'Aborted',
                                                'Failed': 'Aborted'})

            smach.StateMachine.add('ANNOUNCEMENT',
                                   states.Say(robot, "Let's see if my master has a task for me! "
                                                     "Moving to the meeting point.",
                                              block=False),
                                   transitions={'spoken': 'NAVIGATE_TO_WAYPOINT_I'})

            smach.StateMachine.add('NAVIGATE_TO_WAYPOINT_I',
                                   states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot=robot,
                                                                                            id=knowledge.starting_pose),
                                                             radius=0.3),
                                   transitions={'arrived': 'FETCH_COMMAND_I',
                                                'unreachable': 'FETCH_COMMAND_I',
                                                'goal_not_defined': 'FETCH_COMMAND_I'})

            smach.StateMachine.add('FETCH_COMMAND_I',  # Hear "set the table"
                                   HearFetchCommand(robot, 15.0),
                                   transitions={'heard': 'ASK_FOR_MEAL'})

            smach.StateMachine.add('ASK_FOR_MEAL',
                                   states.Say(robot, "What should I serve, master?", block=True),
                                   transitions={'spoken': 'SET_THE_TABLE'})

            smach.StateMachine.add('SET_THE_TABLE',  # Take order and Set the table (bring the objects to the table)
                                   ManipulateMachine(robot=robot,
                                                     grasp_furniture_id=knowledge.cupboard,
                                                     grasp_surface_id=knowledge.cupboard_surface,
                                                     place_furniture_id=knowledge.table,
                                                     place_surface_id=knowledge.table_surface),
                                   transitions={'succeeded': 'ANNOUNCE_TASK_COMPLETION',
                                                'failed': 'NAVIGATE_TO_WAYPOINT_II'})

            # We won't pour anything
            # smach.StateMachine.add('SERVE_MEAL',  # Serve the meal (for example: pour milk into the bowl)
            #                        states.Initialize(robot),
            #                        transitions={'initialized': 'CORRECT_OBJECT_POSITIONS',
            #                                     'abort': 'Aborted'})

            # You don't get points for this?!
            # smach.StateMachine.add('CORRECT_OBJECT_POSITIONS',  # Inspect table and correct the moved objects
            #                        states.Initialize(robot),
            #                        transitions={'initialized': 'ANNOUNCE_TASK_COMPLETION',
            #                                     'abort': 'Aborted'})

            smach.StateMachine.add('ANNOUNCE_TASK_COMPLETION',
                                   states.Say(robot, "The table is set! Moving to the meeting point for the next task.",
                                              block=False),
                                   transitions={'spoken': 'NAVIGATE_TO_WAYPOINT_II'})

            # Part II: Clean the table
            smach.StateMachine.add('NAVIGATE_TO_WAYPOINT_II',
                                   states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot=robot,
                                                                                            id=knowledge.starting_pose),
                                                             radius=0.3),
                                   transitions={'arrived': 'FETCH_COMMAND_II',
                                                'unreachable': 'FETCH_COMMAND_II',
                                                'goal_not_defined': 'FETCH_COMMAND_II'})

            smach.StateMachine.add('FETCH_COMMAND_II',  # Hear "clear up the table"
                                   HearFetchCommand(robot, 15.0),
                                   transitions={'heard': 'CLEAR_UP'})

            smach.StateMachine.add('CLEAR_UP',  # Clear the table
                                   ManipulateMachine(robot=robot,
                                                     grasp_furniture_id=knowledge.table,
                                                     grasp_surface_id=knowledge.table_surface,
                                                     place_furniture_id=knowledge.cupboard,
                                                     place_surface_id=knowledge.cupboard_surface),
                                   transitions={'succeeded': 'END_CHALLENGE',
                                                'failed': 'END_CHALLENGE'})

            # We can't clean the table
            # smach.StateMachine.add('CLEAN_THE_TABLE',  # Inspect for spots and spills and clean them
            #                        states.Initialize(robot),
            #                        transitions={'initialized': 'END_CHALLENGE',
            #                                     'abort': 'Aborted'})

            # End
            smach.StateMachine.add('END_CHALLENGE',
                                   states.Say(robot, "I am done here"),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "set_a_table")

if __name__ == "__main__":
    rospy.init_node('set_a_table_exec')

    startup(ChallengeSetATable, challenge_name="challenge_set_a_table")
