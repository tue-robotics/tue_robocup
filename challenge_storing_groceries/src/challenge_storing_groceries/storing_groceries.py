#!/usr/bin/python

# ROS
import rospy
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
from open_door import OpenDoorMachine
from pdf import WritePdf


class StoringGroceries(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])
        # start_waypoint = ds.EntityByIdDesignator(robot, id="manipulation_init_pose", name="start_waypoint")

        pdf_writer = WritePdf(robot=robot)

        skip_door = rospy.get_param("~skip_door", False)

        with self:
            single_item = ManipulateMachine(robot, pdf_writer=pdf_writer)

            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'AWAIT_START',
                                                'abort': 'Aborted'})

            smach.StateMachine.add("AWAIT_START",
                                   states.AskContinue(robot),
                                   transitions={'continue': "MOVE_TABLE",
                                                'no_response': 'AWAIT_START'})

            cabinet = ds.EntityByIdDesignator(robot, id=CABINET)

            open_door = OpenDoorMachine(robot, 'temp', 'in_front_of', 'shelf6') # cabinet_id is overwritten by 'move_table' below

            @smach.cb_interface(outcomes=["done"])
            def move_table(userdata=None, manipulate_machine=None):
                """ Moves the entities for this challenge to the correct poses"""
                # Determine where to perform the challenge
                # Apparently, the indices here come from:
                # (cabinet_pose, table_pose, cabinet_amcl, grasp_surface, room, default_place_area)

                robot_pose = robot.base.get_location()
                WORKSPACES.sort(key=lambda ws: (ws.place_entity_conf.pose_estimate.frame.p - robot_pose.frame.p).Norm())
                closest_workspace = WORKSPACES[0]
                rospy.loginfo("Closest workspace: grasp from '{grasp}' and place on '{place}'".format(grasp=closest_workspace.grasp_entity_conf.entity_id,
                                                                                                      place=closest_workspace.place_entity_conf.entity_id))
                cabinet_id = closest_workspace.place_entity_conf.entity_id
                table_id = closest_workspace.grasp_entity_conf.entity_id

                # Update the world model by fitting the entities to the frame_stamped's given below.
                robot.ed.update_entity(id=cabinet_id, frame_stamped=closest_workspace.place_entity_conf.pose_estimate)
                robot.ed.update_entity(id=table_id, frame_stamped=closest_workspace.grasp_entity_conf.pose_estimate)

                # Update designators
                cabinet.id_ = closest_workspace.place_entity_conf.entity_id

                # Update manipulate machine
                manipulate_machine.table_designator.id_         = closest_workspace.grasp_entity_conf.entity_id

                manipulate_machine.place_entity_designator.id_  = closest_workspace.place_entity_conf.entity_id
                manipulate_machine.place_designator._area       = closest_workspace.place_entity_conf.manipulation_volumes[0]
                manipulate_machine.place_designator.place_location_designator.id = closest_workspace.place_entity_conf.entity_id
                manipulate_machine.cabinet.id_                  = closest_workspace.place_entity_conf.entity_id
                open_door.cabinet.id_                           = closest_workspace.place_entity_conf.entity_id

                return "done"

            smach.StateMachine.add("MOVE_TABLE",
                                   smach.CBState(move_table, cb_args=[single_item]),
                                   transitions={'done': 'OPEN_DOOR' if not skip_door else "SAY_UNABLE_TO_OPEN_DOOR"})

            smach.StateMachine.add("OPEN_DOOR",
                                   open_door,
                                   transitions={'succeeded': 'RANGE_ITERATOR',
                                                'failed': 'SAY_UNABLE_TO_OPEN_DOOR'})

            smach.StateMachine.add('SAY_UNABLE_TO_OPEN_DOOR',
                                   states.Say(robot, "I am unable to open the shelf door, "
                                                     "can you please open it for me?"),
                                   transitions={'spoken': 'RANGE_ITERATOR'})

            # If you want to reinstate cabinet inspection uncomment section below and change transition above
            # smach.StateMachine.add("NAV_TO_START",
            #                        states.NavigateToSymbolic(robot,
            #                                                  {cabinet: "in_front_of"},
            #                                                  cabinet),
            #                        transitions={'arrived': 'INSPECT_SHELVES',
            #                                     'unreachable': 'INSPECT_SHELVES',
            #                                     'goal_not_defined': 'INSPECT_SHELVES'})
            #
            # smach.StateMachine.add("INSPECT_SHELVES",
            #                        InspectShelves(robot, cabinet),
            #                        transitions={'succeeded': 'WRITE_PDF_SHELVES',
            #                                     'nothing_found': 'WRITE_PDF_SHELVES',
            #                                     'failed': 'WRITE_PDF_SHELVES'})
            #
            # smach.StateMachine.add("WRITE_PDF_SHELVES", pdf_writer, transitions={"done": "RANGE_ITERATOR"})

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
                                   {'succeeded': 'AT_END',
                                    'failed': 'Aborted'})
            # End setup iterator

            smach.StateMachine.add('AT_END',
                                   states.Say(robot, "Goodbye"),
                                   transitions={'spoken': 'Done'})

            ds.analyse_designators(self, "manipulation")
