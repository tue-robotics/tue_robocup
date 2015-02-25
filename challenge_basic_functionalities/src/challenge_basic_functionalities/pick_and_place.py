#! /usr/bin/env python
import rospy

import smach

from robot_skills.amigo import Amigo
from robot_smach_states import *

from robot_skills.reasoner  import Conjunction, Compound, Disjunction, Constant
from robot_smach_states.util.startup import startup
import robot_skills.util.msg_constructors as msgs
import robot_skills.util.transformations as transformations
from robot_smach_states.util.designators import Designator, VariableDesignator, EdEntityByQueryDesignator

from pein_srvs.srv import SetObjects
from ed.srv import SimpleQuery, SimpleQueryRequest

from robot_smach_states.utility_states import Initialize
from robot_smach_states.human_interaction import Say

from robot_smach_states import Grab

import inspect

# ----------------------------------------------------------------------------------------------------

class LookForObjects(smach.State):
    """Sets a VariableDesignator to an Entity that matches some criteria."""
    def __init__(self, robot, designator):
        smach.State.__init__(self,outcomes=['done', 'failed'])
        self.robot = robot
        self.designator = designator

    def execute(self, userdata):
        roi = transformations.tf_transform(msgs.Point(0.7, 0, 0.9), self.robot.robot_name+"/base_link", "/map", self.robot.tf_listener)
            
        # import ipdb; ipdb.set_trace()
        entities = self.robot.ed.get_entities(type="", center_point=roi, radius=0.5) #TODO Sjoerd: point should be a PointStamped here.
        rospy.loginfo("Found {0} objects".format(len(entities)))

        max_size = lambda ent: (ent.z_max - ent.z_min) < 0.20 #Only objects smaller than 20cm
        min_size = lambda ent: (ent.z_max - ent.z_min) > 0.05 #Only objects larger than 5cm
        min_height = lambda ent: (ent.z_min) > 0.70 #Only objects higher than 70cm
        max_height = lambda ent: (ent.z_min) < 1.20 #Only objects lower than 1.2m
        filters = [max_size, min_size, min_height, max_height]

        for filter_index, filterfunc in enumerate(filters):
            entities = filter(filterfunc, entities) 
            filter_code = str(inspect.getsource(filterfunc).strip())
            rospy.loginfo("{0} objects remaining after filterfunc #{1}: {2}".format(len(entities), filter_index, filter_code))

        if entities:
            self.designator.current = entities[0]
            rospy.loginfo("Selected object: {0}".format(self.designator.current.id))
        elif entities:
            self.designator.current = entities[0]
            rospy.logwarn("May not be able to pick up selected object: {0}".format(self.designator.current.id))
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
        self.designator = VariableDesignator()

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
                                                 "failed": "HANDOVER_FROM_HUMAN"})

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
