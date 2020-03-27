from __future__ import absolute_import

# ROS
import rospy

# TU/e Robotics
from cb_planner_msgs_srvs.msg import *
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import kdl_vector_to_point_msg
from .navigation import NavigateTo
from .constraint_functions import combine_constraints, symbolic_constraint, look_at_constraint
from ..util.designators import check_resolve_type


class NavigateToSymbolic(NavigateTo):
    """ Navigation class to navigate to a semantically annotated goal, e.g., in front of the dinner table.
    """
    def __init__(self, robot, entity_designator_area_name_map, entity_lookat_designator):
        """
        Constructor

        :param robot: robot object
        :param entity_designator_area_name_map: dictionary mapping EdEntityDesignators to a string or designator
            resolving to a string, representing the area, e.g., entity_designator_area_name_map[<EdEntity>] = 'in_front_of'.
        :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at. This is used
            to compute the orientation constraint.
        """
        # Check that the entity_designator_area_name_map's keys all resolve to EntityInfo's
        assert(all(entity_desig.resolve_type == Entity for entity_desig in entity_designator_area_name_map.keys()))
        check_resolve_type(entity_lookat_designator, Entity)

        constraint_list = [
            lambda: symbolic_constraint(robot, entity_designator_area_name_map),
            lambda: look_at_constraint(entity_lookat_designator)
        ]
        super(NavigateToSymbolic, self).__init__(robot, lambda: combine_constraints(constraint_list))


class NavigateToRoom(NavigateToSymbolic):
    """
    Navigation class to navigate to the 'in' area of the provided entity, typically a room.
    """
    def __init__(self, robot, entity_designator_room, entity_lookat_designator=None):
        """
        :param robot: robot object
        :type robot: robot
        :param entity_designator_room: Designator to the room
        :type entity_designator_room: Designator resolving to an ED entity
        :param entity_lookat_designator: (Optional) Designator defining the entity the robot should look at. This is
            used to compute the orientation constraint. If not provided, the entity_designator_room is used.
        :type entity_lookat_designator: Designator resolving to an ED entity
        """
        room_area = "in"
        if not entity_lookat_designator:
            entity_lookat_designator = entity_designator_room
        super(NavigateToRoom, self).__init__(robot, {entity_designator_room: room_area}, entity_lookat_designator)
