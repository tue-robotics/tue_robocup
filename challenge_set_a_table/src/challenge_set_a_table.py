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
from challenge_set_a_table_states.clear_manipulate_machine import ClearManipulateMachine

# Load all knowledge
knowledge = load_knowledge('challenge_set_a_table')


class ChallengeSetATable(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        # Create designators
        grasp_designator1 = ds.EdEntityDesignator(robot, type="temp")
        grasp_designator2 = ds.EdEntityDesignator(robot, type="temp")
        grasp_designator3 = ds.EdEntityDesignator(robot, type="temp")

        start_pose = robot.base.get_location()
        start_x = start_pose.frame.p.x()
        start_y = start_pose.frame.p.y()
        start_rz = start_pose.frame.M.GetRPY()[2]

        with self:
            # Part I: Set a table
            smach.StateMachine.add('ENTER_ROOM',  # Enter the room
                                   states.Initialize(robot),
                                   transitions={'initialized': 'ANNOUNCEMENT',
                                                'abort': 'Aborted'})

            smach.StateMachine.add('ANNOUNCEMENT',
                                   states.Say(robot, "Let's see if my master has a task for me! ", block=True),
                                   transitions={'spoken': 'FETCH_COMMAND_I'})

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
                                                     grasp_designator3=grasp_designator3,
                                                     grasp_furniture_id1=knowledge.cupboard,
                                                     grasp_furniture_id3=knowledge.cupboard,
                                                     place_furniture_id=knowledge.table),
                                   transitions={'succeeded': 'ANNOUNCE_TASK_COMPLETION',
                                                'failed': 'RETURN_TO_START_2'})

            smach.StateMachine.add('ANNOUNCE_TASK_COMPLETION',
                                   states.Say(robot, "The table is set! Moving to the meeting point for the next task.",
                                              block=False),
                                   transitions={'spoken': 'RETURN_TO_START_2'})

            # Part II: Clean the table
            smach.StateMachine.add('RETURN_TO_START_2',
                                   states.NavigateToPose(robot=robot, x=start_x, y=start_y, rz=start_rz, radius=0.3),
                                   transitions={'arrived': 'FETCH_COMMAND_II',
                                                'unreachable': 'FETCH_COMMAND_II',
                                                'goal_not_defined': 'FETCH_COMMAND_II'})

            smach.StateMachine.add('FETCH_COMMAND_II',  # Hear "clear up the table"
                                   HearFetchCommand(robot, 15.0, "clear"),
                                   transitions={'done': 'CLEAR_UP'})

            smach.StateMachine.add('CLEAR_UP',  # Clear the table
                                   ClearManipulateMachine(robot=robot, grasp_furniture_id=knowledge.table,
                                                          place_furniture_id1=knowledge.cupboard,
                                                          place_furniture_id3=knowledge.cupboard),
                                   transitions={'succeeded': 'END_CHALLENGE',
                                                'failed': 'END_CHALLENGE'})

            # End
            smach.StateMachine.add('END_CHALLENGE',
                                   states.Say(robot, "I am done here"),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "set_a_table")

if __name__ == "__main__":
    rospy.init_node('set_a_table_exec')

    startup(ChallengeSetATable, challenge_name="challenge_set_a_table")
