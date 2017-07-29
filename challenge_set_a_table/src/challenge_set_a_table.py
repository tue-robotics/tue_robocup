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
from challenge_set_a_table_states.fetch_command import HearFetchCommand, GetBreakfastOrder
from challenge_set_a_table_states.manipulate_machine import ManipulateMachine, DefaultGrabDesignator

# Load all knowledge
knowledge = load_knowledge('challenge_set_a_table')


class ChallengeSetATable(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        # Create designators
        grasp_furniture_designator = ds.EntityByIdDesignator(robot, id=knowledge.cupboard)
        grasp_designator = DefaultGrabDesignator(robot=robot, surface_designator=grasp_furniture_designator,
                                                  area_description=knowledge.cupboard_surface)

        place_furniture_designator = ds.EntityByIdDesignator(robot, id=knowledge.table)
        place_designator = ds.EmptySpotDesignator(robot=robot,
                                                  place_location_designator=place_furniture_designator,
                                                  area=knowledge.table_surface)

        grasp_designator1 = ds.EdEntityDesignator(robot, type="temp")
        grasp_designator2 = ds.EdEntityDesignator(robot, type="temp")
        grasp_designator3 = ds.EdEntityDesignator(robot, type="temp")

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
                                              block=True),
                                   transitions={'spoken': 'NAVIGATE_TO_WAYPOINT_I'})

            smach.StateMachine.add('NAVIGATE_TO_WAYPOINT_I',
                                   states.NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot=robot,
                                                                                            id=knowledge.starting_pose),
                                                             radius=0.3),
                                   transitions={'arrived': 'FETCH_COMMAND_I',
                                                'unreachable': 'FETCH_COMMAND_I',
                                                'goal_not_defined': 'FETCH_COMMAND_I'})

            smach.StateMachine.add('FETCH_COMMAND_I',  # Hear "set the table"
                                   HearFetchCommand(robot, 15.0, "set"),
                                   transitions={'done': 'ASK_FOR_MEAL'})

            smach.StateMachine.add('ASK_FOR_MEAL',
                                   states.Say(robot, "What should I serve, master?", block=True),
                                   transitions={'spoken': 'GET_ORDER'})

            smach.StateMachine.add('GET_ORDER',
                                   GetBreakfastOrder(robot, knowledge.options,
                                                     grasp_designator1,
                                                     grasp_designator2,
                                                     grasp_designator3,
                                                     timeout=15.0),
                                   transitions={'done': 'SET_THE_TABLE'})

            smach.StateMachine.add('SET_THE_TABLE',  # Take order and Set the table (bring the objects to the table)
                                   ManipulateMachine(robot=robot,
                                                     grasp_designator1=grasp_designator1,
                                                     grasp_designator2=grasp_designator2,
                                                     grasp_designator3=grasp_designator3),
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
                                   HearFetchCommand(robot, 15.0, "clear"),
                                   transitions={'done': 'CLEAR_UP'})

            smach.StateMachine.add('CLEAR_UP',  # Clear the table
                                   ManipulateMachine(robot=robot,
                                                     grasp_furniture_designator, grasp_designator,
                                                     place_furniture_designator, place_designator
                                                     ),
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
