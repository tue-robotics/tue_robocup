#! /usr/bin/env python
import rospy

import smach

from robot_smach_states.utility_states import Initialize
from robot_smach_states.human_interaction import Say

class WhatDidYouSay(smach.StateMachine):

    def __init__(self, robot, grasp_arm="left"):
        # ToDo: get rid of hardcode poi lookat
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot

        with self:

            smach.StateMachine.add( 'INIT',
                                    Initialize(robot),
                                    transitions={"initialized": "ASK_QUESTION",
                                                 "abort":       "Aborted"})

            smach.StateMachine.add( 'ASK_QUESTION',
                                    Say(robot, ["Sorry, this task is not yet implemented. Rein and Erik, fix this!"]),
                                     transitions={   'spoken':'Failed'})

if __name__ == "__main__":
    rospy.init_node('what_did_you_say_exec')
    robot_smach_states.util.startup(WhatDidYouSay)
