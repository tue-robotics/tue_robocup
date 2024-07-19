from __future__ import absolute_import

# TU/e Robotics
from ed.entity import Entity
from .navigation import NavigateTo
from .constraint_functions import combine_constraints, waypoint_constraint, look_at_constraint
from ..util.designators import check_resolve_type


class NavigateToWaypoint(NavigateTo):
    def __init__(self, robot, waypoint_designator, radius=0.15, look_at_designator=None, reset_head=True, speak=True, reset_pose=True):
        """
        Navigate to a waypoint in the world model

        :param waypoint_designator: designator resolving to the waypoint
        :param radius (default 0.15): allowed distance to the waypoint
        :param look_at_designator: Entity the robot should face from the waypoint. If not specified the robot will adopt
            the orientation of the waypoint
        :param speak: Whether the robot should speak while driving.
        """
        check_resolve_type(waypoint_designator, Entity)

        if look_at_designator:
            check_resolve_type(look_at_designator, Entity)
            constraints = [lambda: waypoint_constraint(waypoint_designator, radius, look=False),
                           lambda: look_at_constraint(look_at_designator)]
        else:
            constraints = [lambda: waypoint_constraint(waypoint_designator, radius, look=True)]

        super(NavigateToWaypoint, self).__init__(robot, lambda: combine_constraints(constraints), reset_head=reset_head, speak=speak, reset_pose=reset_pose)
