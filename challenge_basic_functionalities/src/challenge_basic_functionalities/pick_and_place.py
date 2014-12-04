#! /usr/bin/env python
import rospy

import smach

from robot_smach_states.utility_states import Initialize
from robot_smach_states.human_interaction import Say
from robot_smach_states.manip import Grab

# ----------------------------------------------------------------------------------------------------

class SimpleDesignator:

    __init__(self):
        self.entity_id = None

    def resolve():
        return self.entity_id

# ----------------------------------------------------------------------------------------------------

class LookForObjects(smach.State):
   def __init__(self, robot, designator):
       smach.State.__init__(self,outcomes=['done', 'failed'])
       self.robot = robot
       self.designator = designator

   def execute(self, userdata):
        self.designator.entity_id = "dinner_table"   # TODO
        return 'done'

# ----------------------------------------------------------------------------------------------------

class PickAndPlace(smach.StateMachine):

    def __init__(self, robot, grasp_arm="left"):
        # ToDo: get rid of hardcode poi lookat
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot
        designator = SimpleDesignator()

        with self:

            smach.StateMachine.add( 'INIT',
                                    Initialize(robot),
                                    transitions={"initialized": "LOOK_FOR_OBJECTS",
                                                 "abort":       "Aborted"})

            smach.StateMachine.add( 'LOOK_FOR_OBJECTS',
                                    LookForObjects(robot, self.designator),
                                    transitions={"done":   "PICKUP_OBJECT",
                                                 "failed": "LOOK_FOR_OBJECTS"})

            smach.StateMachine.add( 'PICKUP_OBJECT',
                                    Grab(robot, self.designator, side="left"),
                                    transitions={"done":   "Done",
                                                 "failed": "PICKUP_OBJECT"})

if __name__ == "__main__":
    rospy.init_node('pick_and_place_exec')
    robot_smach_states.util.startup(PickAndPlace)
