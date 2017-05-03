#!/usr/bin/python

# ROS
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_smach_states.util.geometry_helpers import *

# Challenge storing groceries
from config import *
from inspect_shelves import InspectShelves
from manipulate_machine import ManipulateMachine
from pdf import WritePdf


class StoringGroceries(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])
        # start_waypoint = ds.EntityByIdDesignator(robot, id="manipulation_init_pose", name="start_waypoint")

        pdf_writer = WritePdf(robot=robot)

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'MOVE_TABLE',
                                                'abort': 'Aborted'})

            @smach.cb_interface(outcomes=["done"])
            def move_table(userdata):
                """ 'Locks' a locking designator """
                # This determines that self.current_item cannot not resolve to a new value until it is unlocked again.
                robot.ed.update_entity(id=TABLE, frame_stamped=TABLE_POSE)

                return "done"

            smach.StateMachine.add("MOVE_TABLE",
                                   smach.CBState(move_table),
                                   transitions={'done': 'AWAIT_START'})

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
                                   transitions={'succeeded': 'WRITE_PDF_SHELVES',
                                                'nothing_found': 'WRITE_PDF_SHELVES',
                                                'failed': 'WRITE_PDF_SHELVES'})

            smach.StateMachine.add("WRITE_PDF_SHELVES", pdf_writer, transitions={"done": "RANGE_ITERATOR"})

            # Begin setup iterator
            # The exhausted argument should be set to the prefered state machine outcome
            range_iterator = smach.Iterator(outcomes=['succeeded', 'failed'],  # Outcomes of the iterator state
                                            input_keys=[], output_keys=[],
                                            it=lambda: range(5),
                                            it_label='index',
                                            exhausted_outcome='succeeded')

            with range_iterator:
                single_item = ManipulateMachine(robot, pdf_writer=pdf_writer)  # ToDo: add more pdf stuff

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

