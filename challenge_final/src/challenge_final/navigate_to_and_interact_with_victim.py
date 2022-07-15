import os

import rospy
from smach.state_machine import StateMachine

from robot_skills import get_robot
from robot_smach_states.human_interaction import Say, AskYesNo
from robot_smach_states.navigation import NavigateToWaypoint, NavigateWiggle
from robot_smach_states.util.designators import EntityByIdDesignator


class NavigateToAndInteractWithVictim(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["done", "preempted"])

        entity_des = EntityByIdDesignator(robot, uuid="victim", name="victim_des")

        with self:
            StateMachine.add("NAVIGATE_VICTIM1", NavigateToWaypoint(robot, entity_des, radius=0.3),
                             transitions={"arrived": "SAY",
                                          "unreachable": "WIGGLE",
                                          "goal_not_defined": "WIGGLE"})
            StateMachine.add("WIGGLE", NavigateWiggle(robot, 2, speak=True),
                             transitions={"done": "NAVIGATE_VICTIM2"})
            StateMachine.add("NAVIGATE_VICTIM2", NavigateToWaypoint(robot, entity_des, radius=0.5),
                             transitions={"arrived": "SAY_OKAY",
                                          "unreachable": "SAY_OKAY",
                                          "goal_not_defined": "SAY_OKAY"})
            StateMachine.add("SAY_OKAY", Say(robot, "Are you Okay? Do you need any help?"),
                             transitions={"spoken": "done"})
            StateMachine.add("ASK_YES_NO", AskYesNo(robot),
                             transitions={"yes": "done",
                                          "no": "done",
                                          "no_result": "done"})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot = get_robot("hero")
    sm = NavigateToAndInteractWithVictim(robot)
    sm.execute()
