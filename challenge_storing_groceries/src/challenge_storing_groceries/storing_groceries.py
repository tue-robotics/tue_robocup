#!/usr/bin/python

"""This challenge is defined in https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/master/Manipulation.tex

In short, the robot starts at 1-1.5m from a bookcase and must wait until started by an operator (by voice or a start button)

This bookcase has a couple of shelves on which some items are placed.
**The middle shelve starts empty**, this is where the objects need to be placed.

The robot must take objects form the shelves and place them on the middle shelve and indicate the class of each grasped object.

After the robot is started by voice or a button,
    the ManipRecogSingleItem state machine is repeated at least 5 times (for 5 objects).
Afterwards, a PDF report has to be made:
'After the test is completed or the time has run out,
    the robot may upload a single PDF report file including the list of recognized objects with a picture showing:
    - the object,
    - the object name,
    - the bounding box of the object.'
"""

import robot_smach_states as states
import robot_smach_states.util.designators as ds
import rospy
import smach
from robot_skills.util.entity import Entity
from robot_smach_states.util.geometry_helpers import *

# import pdf
from config import *
from inspect_shelves import InspectShelves
from manipulate_single_item import ManipRecogSingleItem


class StoringGroceries(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])
        start_waypoint = ds.EntityByIdDesignator(robot, id="manipulation_init_pose", name="start_waypoint")
        placed_items = []

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'INIT_WM',
                                                'abort': 'Aborted'})

            smach.StateMachine.add("AWAIT_START",
                                   states.AskContinue(robot),
                                   transitions={'continue': "NAV_TO_START",
                                                'no_response': 'AWAIT_START'})

            cabinet = ds.EntityByIdDesignator(robot, id=CABINET)
            room = ds.EntityByIdDesignator(robot, id=ROOM)

            smach.StateMachine.add("NAV_TO_START",
                                   states.NavigateToSymbolic(robot,
                                                             {cabinet: "in_front_of"},
                                                             cabinet),
                                   transitions={'arrived': 'INSPECT_SHELVES',
                                                'unreachable': 'INSPECT_SHELVES',
                                                'goal_not_defined': 'INSPECT_SHELVES'})

            smach.StateMachine.add("INSPECT_SHELVES",
                                   InspectShelves(robot),
                                   transitions={'succeeded': 'EXPORT_PDF',
                                                'nothing_found': 'EXPORT_PDF',
                                                'failed': 'EXPORT_PDF'})

            @smach.cb_interface(outcomes=["exported"])
            def export_to_pdf(userdata):
                global DETECTED_OBJECTS_WITH_PROBS

                entities = [e[0] for e in DETECTED_OBJECTS_WITH_PROBS]

                # Export images (Only best MAX_NUM_ENTITIES_IN_PDF)
                # pdf.entities_to_pdf(robot.ed, entities[:MAX_NUM_ENTITIES_IN_PDF],
                # "tech_united_manipulation_challenge")

                return "exported"
            smach.StateMachine.add('EXPORT_PDF',
                                   smach.CBState(export_to_pdf),
                                   transitions={'exported': 'RANGE_ITERATOR'})

            # Begin setup iterator
            # The exhausted argument should be set to the prefered state machine outcome
            range_iterator = smach.Iterator(outcomes=['succeeded', 'failed'],  # Outcomes of the iterator state
                                            input_keys=[], output_keys=[],
                                            it=lambda: range(5),
                                            it_label='index',
                                            exhausted_outcome='succeeded')

            with range_iterator:
                single_item = ManipRecogSingleItem(robot, ds.VariableDesignator(placed_items, [Entity],
                                                                                name="placed_items"))

                smach.Iterator.set_contained_state('SINGLE_ITEM',
                                                   single_item,
                                                   loop_outcomes=['succeeded', 'failed'])

            smach.StateMachine.add('RANGE_ITERATOR', range_iterator,
                                   {'succeeded': 'AT_END',
                                    'failed': 'Aborted'})
            # End setup iterator

            smach.StateMachine.add('AT_END',
                                   states.Say(robot, "Goodbye"),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "manipulation")

