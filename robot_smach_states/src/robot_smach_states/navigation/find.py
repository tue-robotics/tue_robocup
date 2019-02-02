__author__ = 'rokus'

# ROS
import smach
import rospy

# TU/e Robotics
from robocup_knowledge import load_knowledge
from robot_skills.classification_result import ClassificationResult
import robot_smach_states as states
from robot_smach_states.util.designators import VariableDesignator, EdEntityDesignator


def entities_from_description(robot, entity_description, list_of_entity_ids=None):
    '''
    Query entities and return those that satisfy the given description

    @param robot: The robot object
    @param entity_description: A dict that contains a 'type' field
    @param list_of_entity_ids: A list of entity ids to choose from (for example a result of a segment)

    @return: entities
        entities  - list of entities that fulfill the description
                    (each element has type Entity)
    '''
    knowledge = load_knowledge('common')

    list_of_entity_ids = list_of_entity_ids if list_of_entity_ids is not None else []
    assert isinstance(list_of_entity_ids, list), "Input should be a list"
    assert isinstance(entity_description, dict), "Entity description should be a dict"
    assert 'type' in entity_description or 'category' in entity_description

    # Get all entities from the world model
    entities = robot.ed.get_entities()

    # Select entities based on the description
    if entity_description.get('type'):
        entities = [e for e in entities if e.type == entity_description['type']]
    elif entity_description.get('category'):
        entities = [e for e in entities if knowledge.get_object_category(e.type) == entity_description['category']]
    else:
        return []

        # If we have a list of entities to choose from, select based on that list
    if list_of_entity_ids:
        entities = [e for e in entities if e.id in list_of_entity_ids]

    # Sort entities by distance
    robot_location = robot.base.get_location()
    robot_pos = robot_location.frame.p
    entities = sorted(entities, key=lambda entity: entity.distance_to_2d(robot_pos))

    return entities


class CheckIfDescribedEntityAvailable(smach.State):
    """
    Check if a described entity is in the world model
    """
    def __init__(self,
                 robot,
                 description_designator,
                 found_entity_designator,
                 candidate_entities_designator=None):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._robot = robot
        self._description_designator = description_designator
        self._found_entity_designator = found_entity_designator
        self._candidate_entities_designator = candidate_entities_designator

    def execute(self, userdata=None):
        ids_to_select_from = [e.id for e in self._candidate_entities_designator.resolve()]
        rospy.logdebug('list of entities to select from: {}'.format(ids_to_select_from))
        description = self._description_designator.resolve()

        rospy.logdebug('description used for selection: {}'.format(description))

        satisfying_entities = entities_from_description(robot=self._robot,
                                                        entity_description=description,
                                                        list_of_entity_ids=ids_to_select_from)

        rospy.logdebug('entities that satisfy the seleciton criteria: {}'.format(satisfying_entities))

        if satisfying_entities:
            self._found_entity_designator.write(satisfying_entities[0])
            return 'succeeded'
        else:
            return 'failed'


class Find(smach.StateMachine):
    """
    Find an entity based on a description. The description designator should
    have resolve type dict and it should contain at least a 'type' field
    """
    def __init__(self, robot, source_entity_designator, description_designator, area_name_designator,
                 navigation_area_designator, found_entity_designator):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'inspect_failed', 'not_found'])

        segmented_entities_designator = VariableDesignator([], resolve_type=[ClassificationResult])

        with self:
            smach.StateMachine.add( 'INSPECT_SOURCE_ENTITY',
                                    states.world_model.Inspect(robot=robot,
                                                               entityDes=source_entity_designator,
                                                               objectIDsDes=segmented_entities_designator,
                                                               searchArea=area_name_designator,
                                                               navigation_area=navigation_area_designator),
                                    transitions={'done'         : 'CHECK_IF_ENTITY_FOUND',
                                                 'failed'       : 'inspect_failed'})

            smach.StateMachine.add( 'CHECK_IF_ENTITY_FOUND',
                                    CheckIfDescribedEntityAvailable(
                                        robot=robot,
                                        description_designator=description_designator,
                                        found_entity_designator=found_entity_designator.writeable,
                                        candidate_entities_designator=segmented_entities_designator),
                                    transitions={'succeeded'    : 'succeeded',
                                                 'failed'       : 'not_found'})

