#!/usr/bin/env python

import smach
from robocup_knowledge import load_knowledge
from robot_smach_states.human_interaction import AskContinue, Say
from robot_smach_states.navigation import NavigateToWaypoint
from robot_smach_states.reset import ResetED
from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.util.designators import EntityByIdDesignator, analyse_designators

challenge_knowledge = load_knowledge('challenge_rips')

STARTING_POINT = challenge_knowledge.starting_point
INTERMEDIATE_1 = challenge_knowledge.intermediate_1
INTERMEDIATE_2 = challenge_knowledge.intermediate_2
INTERMEDIATE_3 = challenge_knowledge.intermediate_3
EXIT_1 = challenge_knowledge.exit_1
EXIT_2 = challenge_knowledge.exit_2
EXIT_3 = challenge_knowledge.exit_3


def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm:
        # Start challenge via StartChallengeRobust
        smach.StateMachine.add("START_CHALLENGE_ROBUST",
                               StartChallengeRobust(robot, STARTING_POINT),
                               transitions={"Done": "GO_TO_INTERMEDIATE_WAYPOINT",
                                            "Aborted": "GO_TO_INTERMEDIATE_WAYPOINT",
                                            "Failed": "GO_TO_INTERMEDIATE_WAYPOINT"})
        # There is no transition to Failed in StartChallengeRobust (28 May)

        smach.StateMachine.add('GO_TO_INTERMEDIATE_WAYPOINT',
                               NavigateToWaypoint(robot, EntityByIdDesignator(robot, uuid=INTERMEDIATE_1), radius=0.5),
                               transitions={'arrived': 'ASK_CONTINUE',
                                            'unreachable': 'GO_TO_INTERMEDIATE_WAYPOINT_BACKUP1',
                                            'goal_not_defined': 'GO_TO_INTERMEDIATE_WAYPOINT_BACKUP1'})

        smach.StateMachine.add('GO_TO_INTERMEDIATE_WAYPOINT_BACKUP1',
                               NavigateToWaypoint(robot, EntityByIdDesignator(robot, uuid=INTERMEDIATE_2), radius=0.5),
                               transitions={'arrived': 'ASK_CONTINUE',
                                            'unreachable': 'GO_TO_INTERMEDIATE_WAYPOINT_BACKUP2',
                                            'goal_not_defined': 'GO_TO_INTERMEDIATE_WAYPOINT_BACKUP2'})

        smach.StateMachine.add('GO_TO_INTERMEDIATE_WAYPOINT_BACKUP2',
                               NavigateToWaypoint(robot, EntityByIdDesignator(robot, uuid=INTERMEDIATE_3), radius=0.5),
                               transitions={'arrived': 'ASK_CONTINUE',
                                            'unreachable': 'ASK_CONTINUE',
                                            'goal_not_defined': 'ASK_CONTINUE'})

        smach.StateMachine.add("ASK_CONTINUE",
                               AskContinue(robot, 30),
                               transitions={'continue': 'SAY_CONTINUING',
                                            'no_response': 'SAY_CONTINUING'})

        smach.StateMachine.add('SAY_CONTINUING',
                               Say(robot,
                                   ["I heard continue, so I will move to the exit now. See you guys later!"],
                                   block=False),
                               transitions={'spoken': 'GO_TO_EXIT'})

        # Robot goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT',
                               NavigateToWaypoint(robot, EntityByIdDesignator(robot, uuid=EXIT_1), radius=0.7),
                               transitions={'arrived': 'AT_END',
                                            'unreachable': 'GO_TO_EXIT_2',
                                            'goal_not_defined': 'GO_TO_EXIT_2'})

        smach.StateMachine.add('GO_TO_EXIT_2',
                               NavigateToWaypoint(robot, EntityByIdDesignator(robot, uuid=EXIT_2), radius=0.5),
                               transitions={'arrived': 'AT_END',
                                            'unreachable': 'GO_TO_EXIT_3',
                                            'goal_not_defined': 'GO_TO_EXIT_3'})

        smach.StateMachine.add('GO_TO_EXIT_3',
                               NavigateToWaypoint(robot, EntityByIdDesignator(robot, uuid=EXIT_3), radius=0.5),
                               transitions={'arrived': 'AT_END',
                                            'unreachable': 'RESET_ED_TARGET',
                                            'goal_not_defined': 'AT_END'})

        smach.StateMachine.add('RESET_ED_TARGET',
                               ResetED(robot),
                               transitions={'done': 'GO_TO_EXIT'})

        # Finally, the robot will stop and says 'goodbye' to show that he's done.
        smach.StateMachine.add('AT_END',
                               Say(robot, "Goodbye"),
                               transitions={'spoken': 'Done'})

    analyse_designators(sm, "rips")
    return sm
