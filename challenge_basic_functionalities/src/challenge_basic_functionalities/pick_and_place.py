#! /usr/bin/env python
import rospy

import smach

from robot_skills.amigo import Amigo
from robot_smach_states import *

from robot_skills.reasoner  import Conjunction, Compound, Disjunction, Constant
from robot_smach_states.util.startup import startup
import robot_skills.util.msg_constructors as msgs
import robot_skills.util.transformations as transformations
from robot_smach_states.designators.designator import Designator, VariableDesignator, EdEntityByQueryDesignator

from pein_srvs.srv import SetObjects
from ed.srv import SimpleQuery, SimpleQueryRequest

from robot_smach_states.utility_states import Initialize
from robot_smach_states.human_interaction import Say

from robot_smach_states.manip import Grab

# ----------------------------------------------------------------------------------------------------

class SimpleDesignator:
    #TODO: Replace with VariableDesignator
    def __init__(self):
        self.entity_id = None

    def resolve(self):
        return self.entity_id

# ----------------------------------------------------------------------------------------------------

class LookForObjects(smach.State):
    def __init__(self, robot, designator):
        smach.State.__init__(self,outcomes=['done', 'failed'])
        self.robot = robot
        self.designator = designator

    def execute(self, userdata):
        roi = transformations.tf_transform(msgs.Point(0.7, 0, 0.9), "/map", "/base_link", self.robot.tf_listener)
        entities = self.robot.ed.get_entities(type="", point=roi) #TODO Sjoerd: point should be a PointStamped here.

        filtered_entities = filter(lambda ent: (ent.z_max - ent.z_min) < 0.20, entities) #Only objects smaller than 20cm
        if filtered_entities:
            self.designator.entity_id = filtered_entities[0].id
            rospy.loginfo("Selected object: {0}".format(self.designator.entity_id))
        elif entities:
            self.designator.entity_id = entities[0].id
            rospy.logwarn("May not be able to pick up selected object: {0}".format(self.designator.entity_id))
        else:
            rospy.logwarn("Could not select an object")
            return 'failed'

        return 'done'

# ----------------------------------------------------------------------------------------------------

class PickAndPlace(smach.StateMachine):

    def __init__(self, robot, grasp_arm="left"):
        # ToDo: get rid of hardcode poi lookat
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot
        self.designator = SimpleDesignator()

        # self.entity_designator = EdEntityByQueryDesignator(SimpleQueryRequest(type=""))

        with self:

            smach.StateMachine.add( 'INIT',
                                    Initialize(robot),
                                    transitions={"initialized": "LOOK_FOR_OBJECTS",
                                                 "abort":       "Aborted"})

            #TODO: Insert NavigateToExplore here.

            smach.StateMachine.add( 'LOOK_FOR_OBJECTS',
                                    LookForObjects(robot, self.designator),
                                    transitions={"done":   "PICKUP_OBJECT",
                                                 "failed": "LOOK_FOR_OBJECTS"})

            smach.StateMachine.add('PICKUP_OBJECT',
                                    Grab( robot=robot,
                                          arm=robot.rightArm,
                                          designator=self.designator),
                                          transitions={ 'done'   : 'SAY_DROPOFF',
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
