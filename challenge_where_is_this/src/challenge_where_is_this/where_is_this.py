# ROS
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robocup_knowledge import load_knowledge
from robot_skills.util.kdl_conversions import FrameStamped

# Challenge where is this
from .inform_machine import InformMachine

# Load and extract knowledge here so that stuff fails on startup if not defined
knowledge = load_knowledge("challenge_where_is_this")
INFORMATION_POINT_ID = knowledge.information_point_id
INITIAL_POSE_ID = knowledge.initial_pose_id

START_ROBUST = False  # Set this flag to False if you don"t want to use StartChallengeRobust


class WhereIsThis(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted"])

        with self:
            single_item = InformMachine(robot)

            if START_ROBUST:
                smach.StateMachine.add("START_CHALLENGE",
                                       states.StartChallengeRobust(robot, INITIAL_POSE_ID),
                                       transitions={"Done": "NAV_TO_START",
                                                    "Aborted": "Aborted",
                                                    "Failed": "Aborted"})

                smach.StateMachine.add("NAV_TO_START",
                                       states.NavigateToWaypoint(
                                           robot,
                                           ds.EdEntityDesignator(robot, id=INFORMATION_POINT_ID)
                                       ),
                                       transitions={"arrived": "RANGE_ITERATOR",
                                                    "unreachable": "WAIT_NAV_BACKUP",
                                                    "goal_not_defined": "Aborted"})  # If this happens: never mind

                smach.StateMachine.add("WAIT_NAV_BACKUP",
                                       states.WaitTime(robot, 3.0),
                                       transitions={"waited": "NAV_TO_START_BACKUP",
                                                    "preempted": "Aborted"})

                smach.StateMachine.add("NAV_TO_START_BACKUP",
                                       states.NavigateToWaypoint(
                                           robot,
                                           ds.EdEntityDesignator(robot, id=INFORMATION_POINT_ID),
                                           radius=0.5,
                                       ),
                                       transitions={"arrived": "RANGE_ITERATOR",
                                                    "unreachable": "SAY_CANNOT_REACH_WAYPOINT",  # Current pose backup
                                                    "goal_not_defined": "Aborted"})  # If this happens: never mind

                smach.StateMachine.add("SAY_CANNOT_REACH_WAYPOINT",
                                       states.Say(robot, "I am not able to reach the {}."
                                                         "I'll use this as starting point if that's okay"),
                                       transitions={"spoken": "STORE_STARTING_POSE"})
            else:
                smach.StateMachine.add("INITIALIZE",
                                       states.Initialize(robot),
                                       transitions={"initialized": "STORE_STARTING_POSE",
                                                    "abort": "Aborted"})

            @smach.cb_interface(outcomes=["succeeded"])
            def store_pose(userdata=None):
                base_loc = robot.base.get_location()
                base_pose = base_loc.frame
                location_id = INFORMATION_POINT_ID
                robot.ed.update_entity(id=location_id,
                                       frame_stamped=FrameStamped(base_pose, "/map"),
                                       type="waypoint")

                return "succeeded"

            smach.StateMachine.add("STORE_STARTING_POSE",
                                   smach.CBState(store_pose),
                                   transitions={"succeeded": "RANGE_ITERATOR"})

            # Begin setup iterator
            # The exhausted argument should be set to the prefered state machine outcome
            range_iterator = smach.Iterator(outcomes=["succeeded", "failed"],  # Outcomes of the iterator state
                                            input_keys=[], output_keys=[],
                                            it=lambda: range(1000),
                                            it_label="index",
                                            exhausted_outcome="succeeded")

            with range_iterator:
                smach.Iterator.set_contained_state("SINGLE_ITEM",
                                                   single_item,
                                                   loop_outcomes=["succeeded", "failed"])

            smach.StateMachine.add("RANGE_ITERATOR", range_iterator,
                                   {"succeeded": "AT_END",
                                    "failed": "Aborted"})
            # End setup iterator

            smach.StateMachine.add("AT_END",
                                   states.Say(robot, "Goodbye"),
                                   transitions={"spoken": "Done"})
