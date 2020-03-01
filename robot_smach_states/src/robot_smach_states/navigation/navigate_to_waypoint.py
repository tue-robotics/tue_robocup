from __future__ import absolute_import

# TU/e Robotics
from . import NavigateToDesignator
from ..util.designators.navigation import CompoundConstraintsDesignator, WayPointConstraintsDesignator, LookAtConstraintsDesignator


class NavigateToWaypoint(NavigateToDesignator):
    def __init__(self, robot, waypoint_designator, radius = 0.15, look_at_designator=None, speak=True):
        """@param waypoint_designator resolves to a waypoint stored in ED"""
        if look_at_designator:
            constraint_designator = CompoundConstraintsDesignator()
            constraint_designator.add(WayPointConstraintsDesignator(waypoint_designator, radius, look=False), 'waypoint')
            constraint_designator.add(LookAtConstraintsDesignator(look_at_designator), 'lookat')
        else:
            constraint_designator = WayPointConstraintsDesignator(waypoint_designator, radius, look=True)
        super(NavigateToWaypoint, self).__init__(robot, constraint_designator, speak=speak)
