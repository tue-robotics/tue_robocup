#!/usr/bin/env python3

from smach import StateMachine
from ed.entity import Entity
from robocup_knowledge import load_knowledge
from robot_smach_states.utility import Initialize, SetInitialPose
from robot_smach_states.navigation import FollowOperator, NavigateToWaypoint
from robot_smach_states.human_interaction import AskYesNo, Say, GetFurnitureFromOperatorPose
from robot_smach_states.manipulation import Grab, HandoverToHuman
from robot_smach_states.util.designators import analyse_designators
import robot_smach_states.util.designators as ds

challenge_knowledge = load_knowledge("challenge_carry_my_luggage")

STARTING_POINT = challenge_knowledge.starting_point


class CarryMyLuggage(StateMachine):
    def __init__(self, robot):
        """

        :param robot:
        """
        StateMachine.__init__(self, outcomes=['Done', 'Aborted'])
        self.robot = robot
        self.entity_designator = ds.VariableDesignator(resolve_type=Entity)
        self.arm_designator = ds.UnoccupiedArmDesignator(robot).lockable()
        self.waypoint_designator = ds.EntityByIdDesignator(robot, uuid=STARTING_POINT)

        with self:
            StateMachine.add(
                "INITIALIZE",
                Initialize(self.robot),
                transitions={"initialized": "SET_INITIAL_POSE",
                             "abort": "Aborted"}, #todo: change this?
            )

            StateMachine.add(
                "SET_INITIAL_POSE",
                SetInitialPose(self.robot, challenge_knowledge.starting_point),
                transitions={
                    "done": "POINT_BAG",
                    "preempted": "Aborted", #todo: change this?
                    "error": "POINT_BAG",
                },
            )

            StateMachine.add(
                "POINT_BAG",
                Say(self.robot, ["Please point at the bag you want me to carry and await further instruction!"],
                    block=True,
                    look_at_standing_person=True,
                    ),
                transitions={
                    "spoken": "GET_ENTITY_POSE",
                    },
            )

            StateMachine.add(
                'GET_ENTITY_POSE',
                GetFurnitureFromOperatorPose(self.robot, self.entity_designator.writeable),
                transitions={'succeeded': 'GRAB_BAG',
                             'failed': 'GRAB_BAG'} #todo: change this?
            )

            StateMachine.add(
                'GRAB_BAG',
                Grab(self.robot, self.entity_designator, self.arm_designator),
                transitions={"done": "FOLLOW_OPERATOR",
                             "failed": "FOLLOW_OPERATOR"} #todo: change this?
            )

            StateMachine.add(
                "FOLLOW_OPERATOR",
                FollowOperator(self.robot, operator_timeout=30, ask_follow=True, learn_face=True, replan=True),
                transitions={
                    "stopped": "ASK_FOR_TASK",
                    "lost_operator": "ASK_FOR_TASK",
                    "no_operator": "ASK_FOR_TASK",
                },
            )

            StateMachine.add(
                "ASK_FOR_TASK",
                Say(self.robot, ["Are we at the car already?"],
                    block=True,
                    look_at_standing_person=True,
                    ),
                transitions={
                    "spoken": "WAIT_FOR_TASK",
                    },
            )

            StateMachine.add(
                "WAIT_FOR_TASK",
                AskYesNo(self.robot),
                transitions={
                    "yes": "HANDOVER_TO_HUMAN",
                    "no": "FOLLOW_OPERATOR",
                    "no_result": "HANDOVER_TO_HUMAN"
                }
            )

            StateMachine.add(
                "HANDOVER_TO_HUMAN",
                HandoverToHuman(self.robot, self.arm_designator),
                transitions={
                    "succeeded": "NAVIGATE_TO_ARENA",
                    "failed": "NAVIGATE_TO_ARENA", #todo change this?
                },
            )

            StateMachine.add(
                "NAVIGATE_TO_ARENA",
                NavigateToWaypoint(self.robot, self.waypoint_designator),
                transitions={"arrived": "Done",
                             "unreachable": "Done", #todo change this?
                             "goal_not_defined": "Done"} #todo change this?
            )


if __name__ == '__main__':
    from challenge_carry_my_luggage.carry_my_luggage import CarryMyLuggage
    from robot_skills import get_robot
    import sys
    import rospy

    if len(sys.argv) < 2:
        print("Please provide robot name as argument.")
        sys.exit(1)

    rospy.init_node('carry_my_luggage_exec')

    robot_name = sys.argv[1]
    robot = get_robot(robot_name)

    CarryMyLuggage(robot)

