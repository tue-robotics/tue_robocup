from smach.state_machine import StateMachine

from challenge_final.call_neighbor import CallNeighbor
from challenge_final.navigate_arbitrarily import NavigateArbitrarily
from challenge_final.navigate_to_and_interact_with_victim import NavigateToAndInteractWithVictim
from challenge_final.outro import Outro
from robot_smach_states.utility import Initialize


class Final(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["done", "preempted"])

        with self:
            StateMachine.add(
                "INITIALIZE",
                Initialize(robot),
                transitions={"initialized": "NAVIGATE_ARBITRARILY", "abort": "preempted"},
            )

            StateMachine.add(
                "NAVIGATE_ARBITRARILY",
                NavigateArbitrarily(robot),
                transitions={"done": "NAVIGATE_TO_AND_INTERACT_WITH_VICTIM", "preempted": "preempted"},
            )

            StateMachine.add(
                "NAVIGATE_TO_AND_INTERACT_WITH_VICTIM",
                NavigateToAndInteractWithVictim(robot),
                transitions={"done": "CALL_NEIGHBOR_VIA_TELEGRAM", "preempted": "preempted"},
            )

            StateMachine.add(
                "CALL_NEIGHBOR_VIA_TELEGRAM",
                CallNeighbor(robot),
                transitions={
                    "done": "CALL_ROBOT_VIA_DOORBELL_NAVIGATE_TO_THE_DOOR_AND_GUIDE_NEIGHBOR_TO_VICTIM",
                    "preempted": "preempted",
                },
            )

            StateMachine.add(
                "CALL_ROBOT_VIA_DOORBELL_NAVIGATE_TO_THE_DOOR_AND_GUIDE_NEIGHBOR_TO_VICTIM",
                CallRobotViaDoorbellNavigateToTheDoorAndGuideNeighborToVictim(robot),
                transitions={"done": "OUTRO", "preempted": "preempted"},
            )

            StateMachine.add("OUTRO", Outro(robot), transitions={"done": "done", "preempted": "preempted"})
