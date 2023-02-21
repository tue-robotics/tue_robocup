#!/usr/bin/python

import rospy

from robot_skills.util.entity import Entity
from robot_smach_states.util.designators import Designator


class SimilarEntityDesignator(Designator):
    def __init__(self, robot, original, entity_list, knowledge, name=None):
        """
        Return an entity from entity_list that bears a resemblance to the original according to the knowledge
        same type > same category

        :param original: EdEntityDesignator
        :param entity_list: Designator resolving to a list of ClassificationResults to compare to the original
        :param knowledge: Robocup knowledge object
        :param name (optional): name of the designator
        """
        super(SimilarEntityDesignator, self).__init__(self, resolve_type=Entity, name=name)
        self.robot = robot
        self.original = original
        self.entity_list = entity_list
        self.knowledge = knowledge

    def _resolve(self):
        original = self.original.resolve()
        entity_list = self.entity_list.resolve()
        match = find_similar_entity(original, entity_list, self.knowledge)
        if match:
            rospy.loginfo("Selected {}, as it is similar to {}".format(match, original))
            return self.robot.ed.get_entity(id=match.id)
        return None


def find_similar_entity(original, entity_list, knowledge):
    """
    Return an entity from entity_list that bears a resemblance to the original according to the knowledge
    same type > same category

    :param original: EdEntity or ClassificationResult
    :param entity_list: list of ClassificationResults to compare to the original
    :param knowledge: Robocup knowledge object
    :return: EdEntity of a similar
    """

    if not entity_list: # List is empty
        return None

    original_type = original.type
    matching_types = [entity for entity in entity_list if entity.type is original_type]
    if matching_types:
        matching_types.sort(lambda obj: obj.probability, reverse=True)
        return matching_types[0]

    original_category = knowledge.common.get_object_category(original_type)
    matching_categories = [entity for entity in entity_list
                           if entity.type in knowledge.common.object_names_of_category(original_category)]
    if matching_categories:
        matching_categories.sort(lambda obj: obj.probability, reverse=True)
        return matching_categories[0]

    return None
