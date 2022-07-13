# System
import math

# ROS
import rospy
import smach

# TU/e Robotics
import robot_smach_states.util.designators as ds
from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation import NavigateToSymbolic, NavigateToWaypoint
from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.utility import Initialize, WaitTime
from hmi import HMIResult
from robocup_knowledge import load_knowledge

# Challenge where is this
from .inform_machine import InformMachine

# Load and extract knowledge here so that stuff fails on startup if not defined
knowledge = load_knowledge("challenge_where_is_this")
INFORMATION_POINT_ID = knowledge.information_point_id
INITIAL_POSE_ID = knowledge.initial_pose_id
START_GRAMMAR = knowledge.starting_point_grammar
GRAMMAR = knowledge.location_grammar

START_ROBUST = True  # Set this flag to False if you don"t want to use StartChallengeRobust


class WhereIsThis(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted"])

        hmi_result_des = ds.VariableDesignator(resolve_type=HMIResult)
        information_point_id_designator = ds.AttrDesignator(hmi_result_des, "semantics", resolve_type=str)
        information_point_designator = ds.EdEntityDesignator(robot, uuid_designator=information_point_id_designator)

        rospy.loginfo("Starting WhereIsThis StateMachine")

        with self:

            if START_ROBUST:
                smach.StateMachine.add(
                    "START_CHALLENGE",
                    StartChallengeRobust(robot, INITIAL_POSE_ID),
                    transitions={"Done": "NAV_TO_START", "Aborted": "Aborted", "Failed": "Aborted"},
                )

                smach.StateMachine.add(
                    "NAV_TO_START",
                    NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, uuid=INFORMATION_POINT_ID), radius=0.375),
                    transitions={
                        "arrived": "TURN_AROUND",
                        "unreachable": "WAIT_NAV_BACKUP",
                        "goal_not_defined": "Aborted",
                    },
                )

                smach.StateMachine.add(
                    "WAIT_NAV_BACKUP",
                    WaitTime(robot, 3.0),
                    transitions={"waited": "NAV_TO_START_BACKUP", "preempted": "Aborted"},
                )

                smach.StateMachine.add(
                    "NAV_TO_START_BACKUP",
                    NavigateToSymbolic(
                        robot=robot,
                        entity_designator_area_name_map={information_point_designator: "near"},
                        entity_lookat_designator=information_point_designator,
                    ),
                    transitions={
                        "arrived": "TURN_AROUND",
                        "unreachable": "SAY_CANNOT_REACH_WAYPOINT",  # Current pose backup
                        "goal_not_defined": "Aborted",
                    },
                )  # If this happens: never mind

                @smach.cb_interface(outcomes=["done"])
                def _turn_around(userdata=None):
                    """Turns the robot approximately 180 degrees around"""
                    v_th = 0.5
                    robot.base.force_drive(vx=0.0, vy=0.0, vth=v_th, timeout=math.pi / v_th)
                    return "done"

                smach.StateMachine.add(
                    "TURN_AROUND", smach.CBState(_turn_around), transitions={"done": "STORE_STARTING_POSE"}
                )

                smach.StateMachine.add(
                    "SAY_CANNOT_REACH_WAYPOINT",
                    Say(robot, "I am not able to reach the starting point." "I'll use this as starting point"),
                    transitions={"spoken": "STORE_STARTING_POSE"},
                )
            else:
                smach.StateMachine.add(
                    "INITIALIZE",
                    Initialize(robot),
                    transitions={"initialized": "STORE_STARTING_POSE", "abort": "Aborted"},
                )

            # This is purely for a back-up scenario until the range iterator
            @smach.cb_interface(outcomes=["succeeded"])
            def store_pose(userdata=None):
                base_loc = robot.base.get_location()
                location_id = INFORMATION_POINT_ID
                robot.ed.update_entity(uuid=location_id, frame_stamped=base_loc, etype="waypoint")

                return "succeeded"

            smach.StateMachine.add(
                "STORE_STARTING_POSE", smach.CBState(store_pose), transitions={"succeeded": "RANGE_ITERATOR"}
            )

            # Begin setup iterator
            # The exhausted argument should be set to the preferred state machine outcome
            range_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],  # Outcomes of the iterator state
                input_keys=[],
                output_keys=[],
                it=lambda: range(1000),
                it_label="index",
                exhausted_outcome="succeeded",
            )

            single_item = InformMachine(robot)

            with range_iterator:
                smach.Iterator.set_contained_state("SINGLE_ITEM", single_item, loop_outcomes=["succeeded", "failed"])

            smach.StateMachine.add("RANGE_ITERATOR", range_iterator, {"succeeded": "AT_END", "failed": "Aborted"})
            # End setup iterator

            smach.StateMachine.add("AT_END", Say(robot, "Goodbye"), transitions={"spoken": "Done"})
