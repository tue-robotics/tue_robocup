# System
import os

# ROS
import rospy
import smach

# TU/e
from robot_smach_states.navigation import NavigateToSymbolic, NavigateToObserve
from robot_smach_states.util.designators import Designator, VariableDesignator, check_type
from robot_smach_states.world_model import SegmentObjects
from robot_smach_states.designator_iterator import IterateDesignator

from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import VectorStamped
from robot_skills.classification_result import ClassificationResult


class InspectAreaDesignator(Designator):
    """
    Resolve to the inspect areas of a piece of furniture

    :param entityDes: entity designator
    :param knowledge: robocup knowledge object
    """
    def __init__(self, entityDes, knowledge, name=None):
        super(InspectAreaDesignator, self).__init__(resolve_type=[str], name=name)
        self.entityDes = entityDes
        self.knowledge = knowledge

    def _resolve(self):
        entity = self.entityDes.resolve()
        return self.knowledge.common.get_inspect_areas(entity.id)


class InspectAreas(smach.StateMachine):
    """
    Class to navigate to a(n) (furniture) object and segment the objects on top of it.
    """

    def __init__(self, robot, entityDes, objectIDsDes=None, searchAreas=None, navigation_area=None,
                 knowledge=None, unknown_threshold=0.0, filter_threshold=0.0):
        """
        Constructor

        :param robot: robot object
        :param entityDes: EdEntityDesignator indicating the (furniture) object to inspect
        :param objectIDsDes: designator that is used to store the segmented objects
        :param searchAreas: (designator to) array of strings defining where the objects are w.r.t. the entity,
            if none is provided we will use the knowledge to decide them
        :param navigation_area: string identifying the inspection area. If provided, NavigateToSymbolic is used.
            If left empty, NavigateToObserve is used.
        :param unknown_threshold: Entities whose classification score is lower than this float are not marked with a type
        :param filter_threshold: Entities whose classification score is lower than this float are ignored
            (i.e. are not added to the segmented_entity_ids_designator)
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        check_type(entityDes, Entity)

        if not objectIDsDes:
            objectIDsDes = VariableDesignator([], resolve_type=[ClassificationResult])

        if searchAreas:
            check_type(searchAreas, [str])
        else:
            if not knowledge:
                rospy.logerr("Please provide a list of searchAreas or a knowledge object!")
            # get search area's from knowledge
            searchAreas = InspectAreaDesignator(entityDes, knowledge)

        rospy.loginfo("searchAreas: {}".format(searchAreas.resolve()))
        searchArea = VariableDesignator(resolve_type=str)

        with self:
            if navigation_area:
                smach.StateMachine.add('NAVIGATE_TO_INSPECT', NavigateToSymbolic(robot, {entityDes: navigation_area},
                                                                                 entityDes),
                                       transitions={'unreachable': 'failed',
                                                    'goal_not_defined': 'failed',
                                                    'arrived': 'ITERATE_AREA'})
            else:
                smach.StateMachine.add('NAVIGATE_TO_INSPECT', NavigateToObserve(robot, entityDes, radius=1.0),
                                       transitions={'unreachable': 'failed',
                                                    'goal_not_defined': 'failed',
                                                    'arrived': 'ITERATE_AREA'})

            smach.StateMachine.add('ITERATE_AREA',
                                   IterateDesignator(searchAreas, searchArea.writeable),
                                   transitions={'next': 'SEGMENT',
                                                'stop_iteration': 'done'}
                                   )

            smach.StateMachine.add('SEGMENT',
                                   SegmentObjects(robot, objectIDsDes.writeable, entityDes, searchArea,
                                                  unknown_threshold=unknown_threshold,
                                                  filter_threshold=filter_threshold),
                                   transitions={'done': 'ITERATE_AREA'})
