#!/usr/bin/env python3

import smach
from robocup_knowledge import load_knowledge
from robot_smach_states.utility import Initialize, SetInitialPose
from robot_smach_states.navigation import FollowOperator
from robot_smach_states.human_interaction import AskYesNo, Say
from robot_smach_states.util.designators import analyse_designators

challenge_knowledge = load_knowledge("challenge_carry_my_luggage")

STARTING_POINT = challenge_knowledge.starting_point


def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=["Done", "Aborted"])

    with sm:

        sm.add(
            "INITIALIZE",
            Initialize(robot),
            transitions={"initialized": "SET_INITIAL_POSE", "abort": "Aborted"},
        )

        sm.add(
            "SET_INITIAL_POSE",
            SetInitialPose(robot, challenge_knowledge.starting_point),
            transitions={
                "done": "FOLLOW_OPERATOR",
                "preempted": "Aborted",
                "error": "FOLLOW_OPERATOR",
            },
        )

        sm.add(
            "FOLLOW_OPERATOR",
            FollowOperator(robot, operator_timeout=30, ask_follow=True, learn_face=True, replan=True),
            transitions={
                "Done": "ASK_FOR_TASK",
                "Failed": "ASK_FOR_TASK",
                "Aborted": "FOLLOW_OPERATOR",
            },
        )

        sm.add(
            "ASK_FOR_TASK",
            Say(robot, ["Are we at the car already?"],
                block=True,
                look_at_standing_person=True,
            ),
            transitions={
                "spoken": "WAIT_FOR_TASK",
                },
        )

        sm.add(
            "WAIT_FOR_TASK",
            AskYesNo(robot),
            transitions={
                "yes": "REENTER_ARENA",
                "no": "FOLLOW_OPERATOR",
                "no_result": "ASK_FOR_TASK"
            }
        )

    analyse_designators(sm, "carry_my_luggage")
    return sm


def main():
    import rospy
    from robot_smach_states.util.startup import startup

    rospy.init_node("carry_my_luggage_exec")

    startup(setup_statemachine, challenge_name="carry_my_luggage")
