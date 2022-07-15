import os

import numpy as np
import rospy
from smach.state_machine import StateMachine

from ed.entity import Entity
from robot_skills import get_robot
from robot_smach_states.human_interaction import FindFirstPerson
from robot_smach_states.util.designators import VariableDesignator


class NavigateArbitrarily(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["done", "preempted"])

        customer_designator = VariableDesignator(resolve_type=Entity, name="detected_victim")

        with self:
            StateMachine.add(
                "WAIT_FOR_WAVING_PERSON",
                FindFirstPerson(
                    robot,
                    customer_designator.writeable,
                    properties={"tags": ["LWave", "RWave"]},
                    strict=False,
                    nearest=True,
                    speak=False,
                    look_range=(-np.pi / 8, np.pi / 8),
                    look_steps=4,
                    search_timeout=600,
                ),  # 10 minutes
                transitions={"found": "done", "failed": "preempted"},
            )


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    NavigateArbitrarily(robot_instance).execute()
