from __future__ import absolute_import

# TU/e Robotics
from . import NavigateToDesignator
from ..util.designators.navigation import WayPointConstraintsDesignator


class NavigateToWaypoint(NavigateToDesignator):
    def __init__(self, robot, waypoint_designator, radius = 0.15, look_at_designator=None, speak=True):
        """@param waypoint_designator resolves to a waypoint stored in ED"""
        constraint_designator = WayPointConstraintsDesignator(robot, waypoint_designator, radius, look_at_designator=look_at_designator,)
        super(NavigateToWaypoint, self).__init__(robot, constraint_designator, speak=speak)
