#!/usr/bin/env python
import rospy
from robot_smach_states.util import startup
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
        ids_to_select_from = [e.id for e in self._candidate_entities_designator.resolve()]
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
            rospy.logerr("Cup Id: {}".format(satisfying_entities[0].id))
            self._found_entity_designator.write(satisfying_entities[0])
            return 'succeeded'
        else:
            return 'failed'

def setup_statemachine(robot):
    sm = StateMachine(outcomes=['done'])

    with sm:
        # Start challenge via StartChallengeRobust
        StateMachine.add('START_CHALLENGE_ROBUST',
                         StartChallengeRobust(robot, STARTING_POINT, use_entry_points=True),
                         transitions={'Done': 'OPEN_DISHWASHER',
                                      'Aborted': 'done',
                                      'Failed': 'OPEN_DISHWASHER'})

        StateMachine.add('GO_TO_EXIT',
                         NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=EXIT_1), radius=0.7),
                         transitions={'arrived': 'done',
                                      'unreachable': 'GO_TO_EXIT_2',
                                      'goal_not_defined': 'GO_TO_EXIT_2'})

        StateMachine.add('GO_TO_EXIT_2',
                         NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=EXIT_2), radius=0.5),
                         transitions={'arrived': 'done',
                                      'unreachable': 'GO_TO_EXIT_3',
                                      'goal_not_defined': 'GO_TO_EXIT_3'})

        StateMachine.add('GO_TO_EXIT_3',
                         NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=EXIT_3), radius=0.5),
                         transitions={'arrived': 'done',
                                      'unreachable': 'done',
                                      'goal_not_defined': 'done'})
    return sm


if __name__ == '__main__':
    rospy.init_node('find_my_mates_exec')
    startup(setup_statemachine, challenge_name="find_my_mates")
