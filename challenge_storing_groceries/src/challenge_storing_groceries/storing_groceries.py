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

# Challenge storing groceries
from inspect_shelves import InspectShelves
from manipulate_machine import ManipulateMachine
from open_door import OpenDoorMachine

challenge_knowledge = load_knowledge('challenge_storing_groceries')


class StoringGroceries(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])

        skip_door = rospy.get_param("~skip_door", True)
        shelfDes = ds.EntityByIdDesignator(robot, id=challenge_knowledge.shelf)
        tableDes = ds.EntityByIdDesignator(robot, id=challenge_knowledge.table)

        with self:
            smach.StateMachine.add('START_CHALLENGE_ROBUST',
                                   StartChallengeRobust(robot, challenge_knowledge.starting_point),
                                   transitions={'Done': 'SKIP_DOOR',
                                                'Aborted': 'Aborted',
                                                'Failed': 'Failed'})

            smach.StateMachine.add("SKIP_DOOR?",
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
                                   InspectShelves(robot, shelfDes),
                                   transitions={'succeeded': 'WRITE_PDF_SHELVES',
                                                'nothing_found': 'WRITE_PDF_SHELVES',
                                                'failed': 'WRITE_PDF_SHELVES'})

            # store items
            # Begin setup iterator
            single_item = ManipulateMachine(robot)
            # The exhausted argument should be set to the prefered state machine outcome
            range_iterator = smach.Iterator(outcomes=['succeeded', 'failed'],  # Outcomes of the iterator state
                                            input_keys=[], output_keys=[],
                                            it=lambda: range(5),
                                            it_label='index',
                                            exhausted_outcome='succeeded')

            with range_iterator:

                smach.Iterator.set_contained_state('SINGLE_ITEM',
                                                   single_item,
                                                   loop_outcomes=['succeeded', 'failed'])

            smach.StateMachine.add('RANGE_ITERATOR', range_iterator,
                                   {'succeeded': 'AT_END',
                                    'failed': 'Failed'})
            # End setup iterator

            smach.StateMachine.add('AT_END',
                                   states.human_interaction.Say(robot, "Goodbye"),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "manipulation")
