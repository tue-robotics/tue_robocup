from __future__ import absolute_import

# TU/e Robotics
from . import NavigateToDesignator
from ..util.designators.navigation import PoseConstraintsDesignator


class NavigateToPose(NavigateToDesignator):
    def __init__(self, robot,  x, y, rz, radius=0.15, frame_id="/map", speak=True):
        """@param waypoint_designator resolves to a waypoint stored in ED"""
        constraint_designator = PoseConstraintsDesignator(x, y, rz, radius, frame_id)
        super(NavigateToPose, self).__init__(robot, constraint_designator, speak=speak)
