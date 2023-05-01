from __future__ import absolute_import

# TU/e Robotics
from ed.entity import Entity
from .navigation import NavigateTo
from .constraint_functions import combine_constraints, look_at_constraint, radius_constraint, symbolic_constraint
from ..util.designators import check_resolve_type
from ..util.designators.core import Designator


class NavigateToObserve(NavigateTo):
    def __init__(self, robot, entity_designator, radius=0.7, margin=0.075, room: Designator = None, reset_head=True, speak=True, reset_pose=True):
        """
        Navigates to a radius from an ed entity. If a convex hull is present, the distance
        to the convex hull is used. Otherwise, the distance to the pose of the entity is used

        :param robot: (Robot) object
        :param entity_designator: EdEntityDesignator for the object to observe
        :param radius: (float) desired distance to the pose of the entity [m]
        :param margin: (float) allowed margin w.r.t. specified radius on both sides [m]
        :param room: (Optional) Designator to the room you want to stay in
        :param reset_head: Whether or not the head should be used for obstacle avoidance during navigation.
        :param speak: Whether or not the robot should speak during navigation
        :param reset_pose: Whether or not the robot is allowed to change its pose for navigation.
        """
        check_resolve_type(entity_designator, Entity)
        if room:
            check_resolve_type(room, Entity)

        constraint_list = [
            lambda: look_at_constraint(entity_designator),
            lambda: radius_constraint(entity_designator, radius, margin)
        ]

        if room:
            constraint_list.append(lambda: symbolic_constraint(robot, {room: "in"}))

        super(NavigateToObserve, self).__init__(robot, lambda: combine_constraints(constraint_list), reset_head=reset_head, speak=speak, reset_pose=reset_pose)
