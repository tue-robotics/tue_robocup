#! /usr/bin/env python
import rospy

import smach

from robot_smach_states.utility_states import Initialize
from robot_smach_states.human_interaction import Say

class PickAndPlace(smach.StateMachine):

    def __init__(self, robot, grasp_arm="left"):
        # ToDo: get rid of hardcode poi lookat
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot

        with self:

            smach.StateMachine.add( 'INIT',
                                    Initialize(robot),
                                    transitions={"initialized": "PICKUP_OBJECT",
                                                 "abort":       "Aborted"})

            smach.StateMachine.add( 'PICKUP_OBJECT',
                                    Say(robot, ["Sorry, this task is not yet implemented. Ramon, fix this!"]),
                                     transitions={   'spoken':'Failed'})

if __name__ == "__main__":
    rospy.init_node('pick_and_place_exec')
    robot_smach_states.util.startup(PickAndPlace)
