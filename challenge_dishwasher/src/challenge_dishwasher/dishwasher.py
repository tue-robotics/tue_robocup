#!/usr/bin/env python
import rospy

from challenge_dishwasher.custom_place import CustomPlace
from challenge_dishwasher.open_dishwasher import OpenDishwasher
from challenge_dishwasher.simple_grab import FindAndGrab

from ed.entity import Entity

from robot_skills.classification_result import ClassificationResult
from robot_smach_states.human_interaction import Say
from robot_smach_states.navigation import NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.util import startup
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator, EntityByIdDesignator, Designator
from robot_smach_states.world_model import Inspect
from smach import StateMachine, State

STARTING_POINT = "initial_pose"

object_type = 'cup'
source_entity = 'dining_table'
dishwasher_id = 'dishwasher'
dishwasher_navigate_area = 'to_the_side_of'

EXIT_1 = "exit_1_rips"
EXIT_2 = "exit_2_rips"
EXIT_3 = "exit_3_rips"


class GetCupId(State):
    """
    Check if a described entity is in the world model
    """

    def __init__(self, robot, found_entity_designator, candidate_entities_designator=None):
        State.__init__(self, outcomes=['succeeded', 'failed'])
        self._robot = robot
        self._found_entity_designator = found_entity_designator
        self._candidate_entities_designator = candidate_entities_designator

    def execute(self, userdata=None):
        ids_to_select_from = [e.uuid for e in self._candidate_entities_designator.resolve()]
        rospy.logdebug('list of entities to select from: {}'.format(ids_to_select_from))

        satisfying_entities = []

        for id in ids_to_select_from:
            e = EntityByIdDesignator(self._robot, id).resolve()
            object_height = e.shape.z_max - e.shape.z_min
            rospy.logerr("Id: {}, height: {}".format(id, object_height))
            if 0.02 < object_height < 0.15:
                satisfying_entities.append(e)

        rospy.logdebug('entities that satisfy the seleciton criteria: {}'.format(satisfying_entities))

        if satisfying_entities:
            rospy.logerr("Cup Id: {}".format(satisfying_entities[0].uuid))
            self._found_entity_designator.write(satisfying_entities[0])
            return 'succeeded'
        else:
            return 'failed'


class CustomFind(StateMachine):
    def __init__(self, robot, source_entity_designator, area_name_designator, navigation_area_designator,
                 found_entity_designator):
        StateMachine.__init__(self, outcomes=['found', 'not_found'])

        segmented_entities_designator = VariableDesignator([], resolve_type=[ClassificationResult])

        with self:
            StateMachine.add('SAY_LOOKING', Say(robot, ["I'm going to look for the cup"], block=False),
                             transitions={'spoken': 'INSPECT_SOURCE_ENTITY'})

            StateMachine.add('INSPECT_SOURCE_ENTITY', Inspect(robot=robot,
                                                              entityDes=source_entity_designator,
                                                              objectIDsDes=segmented_entities_designator,
                                                              searchArea=area_name_designator,
                                                              navigation_area=navigation_area_designator),
                             transitions={'done': 'GET_CUP_ID',
                                          'failed': 'not_found'})
            StateMachine.add('GET_CUP_ID', GetCupId(robot=robot,
                                                    found_entity_designator=found_entity_designator.writeable,
                                                    candidate_entities_designator=segmented_entities_designator),
                             transitions={'succeeded': 'found',
                                          'failed': 'not_found'})


class FindObject(StateMachine):
    def __init__(self, robot, item):
        StateMachine.__init__(self, outcomes=['found', 'not_found'])

        source_entity_designator = EdEntityDesignator(robot, uuid=source_entity)
        # description_designator = VariableDesignator({
        #     'type': 'cup'
        # })
        area_name_designator = VariableDesignator('on_top_of')
        navigation_area_designator1 = VariableDesignator('in_back_of')
        navigation_area_designator2 = VariableDesignator('in_front_of')

        with self:
            StateMachine.add('FIND', CustomFind(robot, source_entity_designator, area_name_designator,
                                                navigation_area_designator1, item),
                             transitions={'found': 'found',
                                          'not_found': 'FIND2'})

            StateMachine.add('FIND2', CustomFind(robot, source_entity_designator, area_name_designator,
                                                 navigation_area_designator2, item),
                             transitions={'found': 'found',
                                          'not_found': 'FIND3'})

            StateMachine.add('FIND3', CustomFind(robot, source_entity_designator, area_name_designator,
                                                 navigation_area_designator1, item),
                             transitions={'found': 'found',
                                          'not_found': 'FIND4'})

            StateMachine.add('FIND4', CustomFind(robot, source_entity_designator, area_name_designator,
                                                 navigation_area_designator2, item),
                             transitions={'found': 'found',
                                          'not_found': 'not_found'})


class NavigateAndOpenDishwasher(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        dishwasher = EdEntityDesignator(robot=robot, uuid=dishwasher_id)

        with self:
            StateMachine.add("NAVIGATE_TO_DISHWASHER",
                             NavigateToSymbolic(robot, {dishwasher: dishwasher_navigate_area}, dishwasher),
                             transitions={'arrived': 'OPEN_DISHWASHER',
                                          'unreachable': 'failed',
                                          'goal_not_defined': 'failed'})

            StateMachine.add("OPEN_DISHWASHER",
                             OpenDishwasher(robot, dishwasher_id),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


class NavigateAndPlaceDishwasher(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        dishwasher = EdEntityDesignator(robot=robot, uuid=dishwasher_id)

        with self:
            StateMachine.add("NAVIGATE_BACK_TO_DISHWASHER",
                             NavigateToSymbolic(robot, {dishwasher: dishwasher_navigate_area}, dishwasher),
                             transitions={'arrived': 'CUSTOM_PLACE',
                                          'unreachable': 'SAY_DISHWASHER_NOT_REACHABLE',
                                          'goal_not_defined': 'SAY_DISHWASHER_NOT_REACHABLE'})

            StateMachine.add('SAY_DISHWASHER_NOT_REACHABLE', Say(robot, [
                "I cannot reach the dishwasher, let me try again in a couple of seconds! Hopefully this will go right this time."],
                                                                 block=True),
                             transitions={'spoken': 'NAVIGATE_BACK_TO_DISHWASHER2'})

            StateMachine.add("NAVIGATE_BACK_TO_DISHWASHER2",
                             NavigateToSymbolic(robot, {dishwasher: dishwasher_navigate_area}, dishwasher),
                             transitions={'arrived': 'CUSTOM_PLACE',
                                          'unreachable': 'failed',
                                          'goal_not_defined': 'failed'})

            StateMachine.add('CUSTOM_PLACE', CustomPlace(robot),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


class GrabRobust(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        source_entity_designator = EdEntityDesignator(robot, uuid=source_entity)
        navigation_area_designator1 = VariableDesignator('in_front_of')
        navigation_area_designator2 = VariableDesignator('in_back_of')

        with self:
            StateMachine.add("NAVIGATE_TO_INSPECT",
                             NavigateToSymbolic(robot, {source_entity_designator: navigation_area_designator1},
                                                source_entity_designator),
                             transitions={'arrived': 'SIMPLE_GRASP',
                                          'unreachable': 'NAVIGATE_TO_INSPECT2',
                                          'goal_not_defined': 'NAVIGATE_TO_INSPECT2'})

            StateMachine.add('SIMPLE_GRASP', FindAndGrab(robot),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'NAVIGATE_TO_INSPECT2'})

            StateMachine.add("NAVIGATE_TO_INSPECT2",
                             NavigateToSymbolic(robot, {source_entity_designator: navigation_area_designator2},
                                                source_entity_designator),
                             transitions={'arrived': 'SIMPLE_GRASP2',
                                          'unreachable': 'failed',
                                          'goal_not_defined': 'failed'})

            StateMachine.add('SIMPLE_GRASP2', FindAndGrab(robot),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


def setup_statemachine(robot):
    item = VariableDesignator(resolve_type=Entity)
    arm = Designator(robot.leftArm)

    sm = StateMachine(outcomes=['done'])

    with sm:
        # Start challenge via StartChallengeRobust
        StateMachine.add('START_CHALLENGE_ROBUST',
                         StartChallengeRobust(robot, STARTING_POINT, use_entry_points=True),
                         transitions={'Done': 'OPEN_DISHWASHER',
                                      'Aborted': 'done',
                                      'Failed': 'OPEN_DISHWASHER'})

        StateMachine.add('OPEN_DISHWASHER', NavigateAndOpenDishwasher(robot),
                         transitions={'succeeded': 'GRAB',
                                      'failed': 'GRAB'})

        # StateMachine.add('FIND_OBJECT', FindObject(robot, item),
        #                  transitions={'found': 'GRAB',
        #                               'not_found': 'SAY_EXIT'})
        #
        # StateMachine.add('GRAB', Grab(robot, item, arm),
        #                  transitions={'done': 'NAVIGATE_BACK_TO_DISHWASHER',
        #                               'failed': 'FIND_OBJECT2'})
        #
        # StateMachine.add('FIND_OBJECT2', FindObject(robot, item),
        #                  transitions={'found': 'GRAB2',
        #                               'not_found': 'SAY_EXIT'})
        #
        # StateMachine.add('GRAB2', Grab(robot, item, arm),
        #                  transitions={'done': 'NAVIGATE_BACK_TO_DISHWASHER',
        #                               'failed': 'SAY_EXIT'})

        StateMachine.add('GRAB', GrabRobust(robot),
                         transitions={'succeeded': 'PLACE_DISHWASHER',
                                      'failed': 'SAY_EXIT'})

        StateMachine.add('PLACE_DISHWASHER', NavigateAndPlaceDishwasher(robot),
                         transitions={'succeeded': 'SAY_EXIT',
                                      'failed': 'SAY_EXIT'})

        StateMachine.add('SAY_EXIT', Say(robot, ["I will move to the exit now. See you guys later!"], block=False),
                         transitions={'spoken': 'GO_TO_EXIT'})

        StateMachine.add('GO_TO_EXIT',
                         NavigateToWaypoint(robot, EntityByIdDesignator(robot, uuid=EXIT_1), radius=0.7),
                         transitions={'arrived': 'done',
                                      'unreachable': 'GO_TO_EXIT_2',
                                      'goal_not_defined': 'GO_TO_EXIT_2'})

        StateMachine.add('GO_TO_EXIT_2',
                         NavigateToWaypoint(robot, EntityByIdDesignator(robot, uuid=EXIT_2), radius=0.5),
                         transitions={'arrived': 'done',
                                      'unreachable': 'GO_TO_EXIT_3',
                                      'goal_not_defined': 'GO_TO_EXIT_3'})

        StateMachine.add('GO_TO_EXIT_3',
                         NavigateToWaypoint(robot, EntityByIdDesignator(robot, uuid=EXIT_3), radius=0.5),
                         transitions={'arrived': 'done',
                                      'unreachable': 'done',
                                      'goal_not_defined': 'done'})
    return sm


if __name__ == '__main__':
    rospy.init_node('dishwasher_exec')
    startup(setup_statemachine, challenge_name="rips")
