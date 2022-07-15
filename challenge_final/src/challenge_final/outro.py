import os

import rospy
from smach.state_machine import StateMachine

from robot_skills import get_robot
from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation.navigate_to_waypoint import NavigateToWaypoint
from robot_smach_states.util.designators import EntityByIdDesignator


class Outro(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["done", "preempted"])
        intermediate_waypoint = EntityByIdDesignator(robot, "outro_point")
        exit_waypoint = EntityByIdDesignator(robot, "outro_exit_point")

        with self:
            StateMachine.add("SAY_LEAVING", Say(robot, "Please take good care of Arpit. He is very sensitive"),
                             transitions={"spoken": "GO_TO_INTERMEDIATE"})

            StateMachine.add("GO_TO_INTERMEDIATE", NavigateToWaypoint(robot, intermediate_waypoint),
                             transitions={"arrived": "SAY_HAPPY",  "unreachable": "done", "goal_not_defined": "done"})

            StateMachine.add("SAY_HAPPY", Say(robot, "I am glad I could be of Help"),
                             transitions={"spoken": "GO_TO_EXIT"})

            StateMachine.add("GO_TO_EXIT", NavigateToWaypoint(robot, exit_waypoint),
                             transitions={"arrived": "done", "unreachable": "done", "goal_not_defined": "done"})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    Outro(robot_instance).execute()
