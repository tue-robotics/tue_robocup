from __future__ import absolute_import

# TU/e Robotics
from cb_planner_msgs_srvs.msg import *
from robot_skills.util.entity import Entity
from .navigation import NavigateTo
from .constraint_functions import combine_constraints, look_at_constraint, radius_constraint
from ..util.designators import check_resolve_type


class NavigateToObserve(NavigateTo):
    """
    Navigates to a radius from an ed entity. If a convex hull is present, the distance
    to the convex hull is used. Otherwise, the distance to the pose of the entity is used
    :param robot: (Robot) object
    :param entity_designator: EdEntityDesignator for the object to observe
    :param radius: (float) desired distance to the pose of the entity [m]
    :param margin: (float) allowed margin w.r.t. specified radius on both sides [m]
    """
    def __init__(self, robot, entity_designator, radius=0.7, margin=0.075):
        check_resolve_type(entity_designator, Entity)
        constraint_list = [
            lambda: look_at_constraint(entity_designator),
            lambda: radius_constraint(entity_designator, radius, margin)
        ]
        super(NavigateToObserve, self).__init__(robot, lambda: combine_constraints(constraint_list))
