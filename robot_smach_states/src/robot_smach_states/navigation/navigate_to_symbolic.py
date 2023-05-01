from __future__ import absolute_import

# TU/e Robotics
from ed.entity import Entity
from .navigation import NavigateTo
from .constraint_functions import combine_constraints, symbolic_constraint, look_at_constraint
from ..util.designators import check_resolve_type


class NavigateToSymbolic(NavigateTo):
    def __init__(self, robot, entity_designator_area_name_map, entity_lookat_designator, room: Designator = None, reset_head=True, speak=True, reset_pose=True):
        """
        Navigation class to navigate to a semantically annotated goal, e.g., in front of the dinner table.

        :param robot: robot object
        :param entity_designator_area_name_map: dictionary mapping EdEntityDesignators to a string or designator
            resolving to a string, representing the area, e.g., entity_designator_area_name_map[<EdEntity>] = 'in_front_of'.
        :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at. This is used
            to compute the orientation constraint.
        :param room: (Optional) Designator to the room you want to stay in
        :param reset_head: Whether or not the head should be used for obstacle avoidance during navigation.
        :param speak: Whether or not the robot should speak during navigation
        :param reset_pose: Whether or not the robot is allowed to change its pose for navigation.
        """
        # Check that the entity_designator_area_name_map's keys all resolve to EntityInfo's
        assert(all(entity_desig.resolve_type == Entity for entity_desig in entity_designator_area_name_map.keys()))
        check_resolve_type(entity_lookat_designator, Entity)

        constraint_list = [
            lambda: symbolic_constraint(robot, entity_designator_area_name_map),
            lambda: look_at_constraint(entity_lookat_designator)
        ]

        if room:
            constraint_list.append(lambda: symbolic_constraint(robot, {room: "in"}))

        super(NavigateToSymbolic, self).__init__(robot, lambda: combine_constraints(constraint_list), reset_head=reset_head, speak=speak, reset_pose=reset_pose)


class NavigateToRoom(NavigateToSymbolic):
    """
    Navigation class to navigate to the 'in' area of the provided entity, typically a room.

    :param robot: robot object
    :param entity_designator_room: Designator to the room
    :param entity_lookat_designator: (Optional) Designator defining the entity the robot should look at. This is
        used to compute the orientation constraint. If not provided, the entity_designator_room is used.
    :param reset_head: Whether or not the head should be used for obstacle avoidance during navigation.
    :param speak: Whether or not the robot should speak during navigation
    :param reset_pose: Whether or not the robot is allowed to change its pose for navigation.
    """

    def __init__(self, robot, entity_designator_room, entity_lookat_designator=None, reset_head=True, speak=True, reset_pose=True):
        room_area = "in"
        if not entity_lookat_designator:
            entity_lookat_designator = entity_designator_room
        super(NavigateToRoom, self).__init__(robot, {entity_designator_room: room_area}, entity_lookat_designator, reset_head=reset_head, speak=speak, reset_pose=reset_pose)
