#! /usr/bin/env python
import rospy

import smach

from robot_skills.amigo import Amigo
from robot_smach_states import *

from robot_skills.reasoner  import Conjunction, Compound, Disjunction, Constant
from robot_smach_states.util.startup import startup
import robot_skills.util.msg_constructors as msgs
from robot_smach_states.designators.designator import Designator, VariableDesignator, EdEntityByQueryDesignator

from pein_srvs.srv import SetObjects
from ed.srv import SimpleQuery, SimpleQueryRequest

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

            # smach.StateMachine.add( 'PICKUP_OBJECT',
            #                         Say(robot, ["Sorry, this task is not yet fully implemented. Ramon or Loy, fix me please!"]),
            #                          transitions={   'spoken':'PICKUP_OBJECT'})

            smach.StateMachine.add('PICKUP_OBJECT',
                                    PickUp(robot=robot, 
                                                arm=robot.rightArm, 
                                                grab_entity_designator=EdEntityByQueryDesignator(SimpleQueryRequest(type="coke"))),
                                                transitions={   'succeeded'   : 'SAY_DROPOFF',
                                                                'failed'      : 'HANDOVER_FROM_HUMAN'})

            smach.StateMachine.add( 'HANDOVER_FROM_HUMAN', 
                                    Say(robot, [    "I am terribly sorry, but I cannot pick up the object.", 
                                                    "My apologies, but i cannot pick up the object."]),
                                     transitions={   'spoken':'SAY_DROPOFF'})

            smach.StateMachine.add( 'SAY_DROPOFF', 
                                    Say(robot, [    "I am terribly sorry, but I cannot place the object, please take it from me.", 
                                                    "My apologies, but i cannot place the object."]),
                                     transitions={   'spoken':'RESET'})

            smach.StateMachine.add('RESET',
                                    Initialize(robot),
                                    transitions={"initialized": "Done",
                                                 "abort":       "Aborted"})

if __name__ == "__main__":
    rospy.init_node('pick_and_place_exec')
    robot_smach_states.util.startup(PickAndPlace)
