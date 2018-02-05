#!/usr/bin/python

# ROS
import PyKDL as kdl
import smach

# TU/e Robotics
from robot_skills.util.kdl_conversions import FrameStamped
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_smach_states.util.geometry_helpers import *

# Challenge storing groceries
from config import *
from inspect_shelves import InspectShelves
from manipulate_machine import ManipulateMachine
from pdf import WritePdf
from inspection_result_designator import InspectionResultDesignator

class StoringGroceries(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])
        # start_waypoint = ds.EntityByIdDesignator(robot, id="manipulation_init_pose", name="start_waypoint")

        initial_inspection_ds = InspectionResultDesignator() 
        final_inspection_ds = InspectionResultDesignator()

        #Filtering what was already written to pdf happens in pdf_writer
        pdf_writer = WritePdf(robot=robot,initial_inspection_ds=initial_inspection_ds, final_inspection_ds=final_inspection_ds)

        with self:
            single_item = ManipulateMachine(robot, pdf_writer=pdf_writer)

            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'SAY_UNABLE_TO_OPEN_DOOR',
                                                'abort': 'Aborted'})

            smach.StateMachine.add('SAY_UNABLE_TO_OPEN_DOOR',
                                   states.Say(robot, "I am unable to open the shelf door, "
                                                     "can you please open it for me?"),
                                   transitions={'spoken': 'AWAIT_START'})

            smach.StateMachine.add("AWAIT_START",
                                   states.AskContinue(robot),
                                   transitions={'continue': "MOVE_TABLE",
                                                'no_response': 'AWAIT_START'})

            cabinet = ds.EntityByIdDesignator(robot, id=CABINET)
            room = ds.EntityByIdDesignator(robot, id=ROOM)
            

            @smach.cb_interface(outcomes=["done"])
            def move_table(userdata=None, manipulate_machine=None):
                """ Moves the entities for this challenge to the correct poses"""
                # Determine where to perform the challenge
                robot_pose = robot.base.get_location()
                ENTITY_POSES.sort(key=lambda tup: (tup[0].frame.p - robot_pose.frame.p).Norm())
                cabinet_id = ENTITY_POSES[0][2]
                table_id = ENTITY_POSES[0][3]

                # Update the world model
                robot.ed.update_entity(id="balcony_shelf",
                                       frame_stamped=FrameStamped(kdl.Frame(kdl.Rotation(), kdl.Vector(0.0, 3.0, 0.0)),
                                                                  frame_id="map"))
                robot.ed.update_entity(id=cabinet_id, frame_stamped=ENTITY_POSES[0][0])
                robot.ed.update_entity(id=table_id, frame_stamped=ENTITY_POSES[0][1])

                # Update designators
                cabinet.id_ = ENTITY_POSES[0][2]
                room.id_ = ENTITY_POSES[0][4]

                # Update manipulate machine
                manipulate_machine.place_entity_designator.id_ = cabinet_id
                manipulate_machine.place_designator._area = ENTITY_POSES[0][5]
                manipulate_machine.place_designator.place_location_designator.id = cabinet_id
                manipulate_machine.table_designator.id_ = table_id
                manipulate_machine.cabinet.id_ = ENTITY_POSES[0][2]

                return "done"

            smach.StateMachine.add("MOVE_TABLE",
                                   smach.CBState(move_table, cb_args=[single_item]),
                                   transitions={'done': 'NAV_TO_START'})

            smach.StateMachine.add("NAV_TO_START",
                                   states.NavigateToSymbolic(robot,
                                                             {cabinet: "in_front_of"},
                                                             cabinet),
                                   transitions={'arrived': 'INSPECT_SHELVES_BEFORE',
                                                'unreachable': 'INSPECT_SHELVES_BEFORE',
                                                'goal_not_defined': 'INSPECT_SHELVES_BEFORE'})

            smach.StateMachine.add("INSPECT_SHELVES_BEFORE",
                                   InspectShelves(robot, cabinet,initial_inspection_ds),
                                   transitions={'succeeded': 'WRITE_PDF_SHELVES_BEFORE',
                                                'nothing_found': 'WRITE_PDF_SHELVES_BEFORE',
                                                'failed': 'WRITE_PDF_SHELVES_BEFORE'})

            smach.StateMachine.add("WRITE_PDF_SHELVES_BEFORE", pdf_writer, transitions={"done": "RANGE_ITERATOR"})

            # Begin setup iterator
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
                                   {'succeeded': 'NAV_TO_END_INSPECTION',
                                    'failed': 'Aborted'})

            smach.StateMachine.add("NAV_TO_END_INSPECTION",
                                   states.NavigateToSymbolic(robot,
                                                             {cabinet: "in_front_of"},
                                                             cabinet),
                                   transitions={'arrived': 'INSPECT_SHELVES_AFTER',
                                                'unreachable': 'INSPECT_SHELVES_AFTER',
                                                'goal_not_defined': 'INSPECT_SHELVES_AFTER'})

            smach.StateMachine.add("INSPECT_SHELVES_AFTER",
                                   InspectShelves(robot, cabinet,final_inspection_ds),
                                   transitions={'succeeded': 'WRITE_PDF_SHELVES_AFTER',
                                                'nothing_found': 'WRITE_PDF_SHELVES_AFTER',
                                                'failed': 'WRITE_PDF_SHELVES_AFTER'})

            smach.StateMachine.add("WRITE_PDF_SHELVES_AFTER", pdf_writer, transitions={"done": "AT_END"})

            # End setup iterator

            smach.StateMachine.add('AT_END',
                                   states.Say(robot, "Goodbye"),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "manipulation")
