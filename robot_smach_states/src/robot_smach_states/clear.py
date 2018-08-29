#! /usr/bin/env python

# ROS
import rospy
import smach
import tf2_ros

# TU/e Robotics
from robot_skills.util.entity import Entity
from robot_skills.classification_result import ClassificationResult

import robot_smach_states as states
from robot_smach_states.util.designators import check_type
from robot_smach_states.util.designators import VariableDesignator, EdEntityDesignator

class isitclear(smach.State):
    """
    Check if there are entities on the object in the world model
    """

    def __init__(self,
                 robot,
                 objectIDsDes):
        smach.State.__init__(self, outcomes=['clear', 'not_clear'])
        self._robot = robot
        self._object_designator = objectIDsDes

    def execute(self, userdata=None):
        ids_to_select_from = [e.id for e in self._candidate_entities_designator.resolve()]
        rospy.logdebug('list of entities to select from: {}'.format(ids_to_select_from))

        return 'clear'


class Clear(smach.StateMachine):
    def __init__(self, robot, source_location, source_navArea, target_location, target_navArea, target_placeArea="on_top_of", source_searchArea="on_top_of"):
        """
        Let the given robot move to a location and remove all entities from that table one at a time
        :param robot: Robot to use
        :param source_location: Location which will be cleared
        :param target_location: Location where the objects will be placed
        :return:
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        #check_type(source_location, Entity)
        #check_type(target_location, Entity)

        segmented_entities_designator = VariableDesignator([], resolve_type=[ClassificationResult])

        with self:
            smach.StateMachine.add('INSPECT_SOURCE_ENTITY',
                                   states.world_model.Inspect(robot=robot,
                                                              entityDes=source_location,
                                                              objectIDsDes=segmented_entities_designator,
                                                              searchArea=source_searchArea,
                                                              navigation_area=source_navArea,
                                   transitions={'done': 'CHECK_IF_ENTITY_FOUND',
                                                'failed': 'failed'})
                                   )

            smach.StateMachine.add('DETERMINE_IF_CLEAR',
                                   isitclear(robot=robot,
                                             objectIDsDes=segmented_entities_designator),
                                   transitions={'clear': 'done',
                                                'not_clear': 'failed'})
