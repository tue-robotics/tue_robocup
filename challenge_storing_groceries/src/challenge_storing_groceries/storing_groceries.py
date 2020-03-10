#!/usr/bin/python

# ROS
import rospy
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_smach_states.util.geometry_helpers import *

from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.utility import CheckBool
from robocup_knowledge import load_knowledge
from robot_skills.classification_result import ClassificationResult

# Challenge storing groceries
from inspect_shelves import InspectAreas
from manipulate_machine import StoreItems
from open_door import OpenDoorMachine

challenge_knowledge = load_knowledge('challenge_storing_groceries')


def setup_statemachine(robot):
    state_machine = smach.StateMachine(outcomes=['Done', 'Failed', 'Aborted'])

    skip_door = rospy.get_param("~skip_door", True)
    shelfDes = ds.EntityByIdDesignator(robot, id=challenge_knowledge.shelf)
    tableDes = ds.EntityByIdDesignator(robot, id=challenge_knowledge.table)
    objectsDes = ds.VariableDesignator(resolve_type=[ClassificationResult])

    with state_machine:

        smach.StateMachine.add('START_CHALLENGE_ROBUST',
                               StartChallengeRobust(robot, challenge_knowledge.starting_point),
                               transitions={'Done': 'SKIP_DOOR',
                                            'Aborted': 'Aborted',
                                            'Failed': 'Failed'})

        smach.StateMachine.add("SKIP_DOOR",
                               CheckBool(skip_door),
                               transitions={'true': "NAV_TO_START",
                                            'false': "OPEN_DOOR"})

        # open the door of the cabinet
        smach.StateMachine.add("OPEN_DOOR",
                               OpenDoorMachine(robot, shelfDes),
                               transitions={'succeeded': 'NAV_TO_START',
                                            'failed': 'SAY_UNABLE_TO_OPEN_DOOR'})

        smach.StateMachine.add('SAY_UNABLE_TO_OPEN_DOOR',
                               states.human_interaction.Say(robot, "I am unable to open the shelf door, "
                                                                   "can you please open it for me?"),
                               transitions={'spoken': 'NAV_TO_START'})

        # Inspect shelf
        smach.StateMachine.add("NAV_TO_START",
                               states.navigation.NavigateToSymbolic(robot,
                                                                    {shelfDes: "in_front_of"},
                                                                    shelfDes),
                               transitions={'arrived': 'INSPECT_SHELVES',
                                            'unreachable': 'INSPECT_SHELVES',
                                            'goal_not_defined': 'INSPECT_SHELVES'})

        smach.StateMachine.add("INSPECT_SHELVES",
                               InspectAreas(robot, shelfDes, objectsDes, knowledge=challenge_knowledge, navigation_area='in_front_of'),
                               transitions={'done': 'RESET_ARM',
                                            'failed': 'Failed'})

        @smach.cb_interface(outcomes=["done"])
        def reset_arm(userdata=None):
            """ Set the robots arm to reset pose"""
            arm = robot.get_arm(required_goals=['reset'])
            arm.send_joint_goal('reset')
            return "done"
        smach.StateMachine.add("RESET_ARM",
                               smach.CBState(reset_arm),
                               transitions={'done': 'STORE_GROCERIES'})

        # store items
        smach.StateMachine.add("STORE_GROCERIES",
                               StoreItems(robot, tableDes, shelfDes, objectsDes, challenge_knowledge),
                               transitions={'succeeded': 'AT_END',
                                            'preempted': 'Aborted',
                                            'failed': 'Failed'}
                               )

        smach.StateMachine.add('AT_END',
                               states.human_interaction.Say(robot, "Goodbye"),
                               transitions={'spoken': 'Done'})

        ds.analyse_designators(state_machine, "manipulation")

    return state_machine
