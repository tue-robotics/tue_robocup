#!/usr/bin/python

# ROS
import rospy
import PyKDL as kdl
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills.util.kdl_conversions import FrameStamped

# Challenge storing groceries
from inform_machine import InformMachine


class WhereIsThis(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        with self:
            single_item = InformMachine(robot)

            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'STORE_STARTING_POSE',
                                                'abort': 'Aborted'})

            @smach.cb_interface(outcomes=["succeeded"])
            def store_pose(userdata=None):
                base_loc = robot.base.get_location()
                base_pose = base_loc.frame
                location_id = "starting_point"
                robot.ed.update_entity(id=location_id, frame_stamped=FrameStamped(base_pose, "/map"), type="waypoint")

                return "succeeded"

            smach.StateMachine.add("STORE_STARTING_POSE",
                                   smach.CBState(store_pose),
                                   transitions={'succeeded': 'RANGE_ITERATOR'})

            # Begin setup iterator
            # The exhausted argument should be set to the prefered state machine outcome
            range_iterator = smach.Iterator(outcomes=['succeeded', 'failed'],  # Outcomes of the iterator state
                                            input_keys=[], output_keys=[],
                                            it=lambda: range(10),
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

