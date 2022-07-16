import math
import os

import rospy
from smach.state import CBState
from smach.state_machine import StateMachine
from smach.util import cb_interface

from robot_skills import get_robot
from robot_smach_states.human_interaction import Say, AskYesNo
from robot_smach_states.navigation import NavigateToWaypoint, ForceDrive
from robot_smach_states.util.designators import EntityByIdDesignator


class NavigateToAndInteractWithVictim(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["done", "preempted"])

        entity_des = EntityByIdDesignator(robot, uuid="victim", name="victim_des")

        with self:
            StateMachine.add("NAVIGATE_VICTIM",
                             NavigateToWaypoint(robot, entity_des, radius=0.4, look_at_designator=entity_des),
                             transitions={"arrived": "LOOK_DOWN",
                                          "unreachable": "NAVIGATE_VICTIM2",
                                          "goal_not_defined": "preempted"})
            StateMachine.add("NAVIGATE_VICTIM2",
                             NavigateToWaypoint(robot, entity_des, radius=0.5, look_at_designator=entity_des),
                             transitions={"arrived": "LOOK_DOWN",
                                          "unreachable": "TURN_AROUND",
                                          "goal_not_defined": "preempted"})
            StateMachine.add('TURN_AROUND', ForceDrive(robot, 0, 0, 0.5, (2 * math.pi) / 0.5),
                             transitions={'done': 'NAVIGATE_VICTIM2'})

            @cb_interface(outcomes=["done"])
            def _look_down(_):
                robot.head.look_down()
                robot.head.wait_for_motion_done()
                return "done"

            self.add("LOOK_DOWN", CBState(_look_down), transitions={"done": "SAY_WAVE"})

            StateMachine.add("SAY_WAVE", Say(robot, "You waved, indicating that you are hurt, can I help you?"),
                             transitions={"spoken": "ASK_YES_NO_1"})
            StateMachine.add("ASK_YES_NO_1", AskYesNo(robot),
                             transitions={"yes": "SAY_HELP_REQUEST",
                                          "no": "SAY_HELP_REQUEST",
                                          "no_result": "SAY_HELP_REQUEST"})

            StateMachine.add("SAY_HELP_REQUEST", Say(robot, "That is not good, should I send your neighbour for help?"),
                             transitions={"spoken": "ASK_YES_NO_2"})
            StateMachine.add("ASK_YES_NO_2", AskYesNo(robot),
                             transitions={"yes": "SAY_HELP_REQUEST2",
                                          "no": "SAY_HELP_REQUEST2",
                                          "no_result": "SAY_HELP_REQUEST2"})

            StateMachine.add("SAY_HELP_REQUEST2", Say(robot, "Okay I will sent your neighbour a message"),
                             transitions={"spoken": "done"})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot = get_robot("hero")
    sm = NavigateToAndInteractWithVictim(robot)
    sm.execute()
