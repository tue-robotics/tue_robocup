import os

import rospy
from smach.state_machine import StateMachine

from robot_skills import get_robot
from robot_smach_states.human_interaction import Say


class CallNeighbor(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["done", "preempted"])

        with self:
            StateMachine.add("SAY", Say(robot, "CallNeighborViaTelegram"), transitions={"spoken": "done"})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    CallNeighbor(robot_instance).execute()
