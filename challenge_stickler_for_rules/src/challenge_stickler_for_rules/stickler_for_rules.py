# Copyright (c) 2022, TU/e Robotics, Netherlands
# All rights reserved.

import smach

from robot_smach_states.designator_iterator import IterateDesignator
from robot_smach_states.human_interaction import Say
from robot_smach_states.util.designators import EntityByIdDesignator, VariableDesignator
from robot_smach_states.utility import SetInitialPose
from robot_smach_states.reset import ResetArmsTorsoHead
from .patrol import Patrol
from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge("challenge_stickler_for_the_rules")


class SticklerForRules(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted"])

        waypoints_des = VariableDesignator(challenge_knowledge.waypoint_ids)
        waypoint_des = VariableDesignator(resolve_type=str)

        with self:
            # Intro
            smach.StateMachine.add("RESET",
                                   ResetArmsTorsoHead(robot),
                                   transitions={"done": "SET_INITIAL_POSE"})

            smach.StateMachine.add("SET_INITIAL_POSE",
                                   SetInitialPose(robot, challenge_knowledge.starting_point),
                                   transitions={"done": "SAY_START",
                                                "preempted": "SAY_START",
                                                "error": "SAY_START"})
            smach.StateMachine.add(
                "SAY_START", Say(robot, "Party police! Party police!. I'm coming.", block=False),
                transitions={"spoken": "ITERATE_ROOMS"},
            )

            # Main loop
            smach.StateMachine.add("ITERATE_ROOMS",
                                   IterateDesignator(waypoints_des, waypoint_des.writeable),
                                   transitions={"next": "GO_TO_ROOM",
                                                "stop_iteration": "ITERATE_ROOMS"}
                                   )
            smach.StateMachine.add("GO_TO_ROOM",
                                   Patrol(robot, EntityByIdDesignator(robot, uuid=waypoint_des)),
                                   transitions={"done": "ITERATE_ROOMS",
                                                "failed": "ITERATE_ROOMS"})

            # # Outro
            # smach.StateMachine.add(
            #     "GOODBYE",
            #     Say(robot, "I have had it with you naughty rulebreakers. Goodbye!"),
            #     transitions={"spoken": "Done"},
            # )
