from __future__ import absolute_import

# ROS
from geometry_msgs.msg import *
import rospy

# TU/e Robotics
from cb_planner_msgs_srvs.msg import *
from robot_skills.util.entity import Entity
from .navigation import NavigateTo
from .constraint_functions import combine_constraints, waypoint_constraint, look_at_constraint
from ..util.designators import check_resolve_type


class NavigateToWaypoint(NavigateTo):
    def __init__(self, robot, waypoint_designator, radius = 0.15, look_at_designator=None, speak=True):
        """@param waypoint_designator resolves to a waypoint stored in ED"""
        check_resolve_type(waypoint_designator, Entity)

        if look_at_designator:
            check_resolve_type(look_at_designator, Entity)
            constraints = [lambda: waypoint_constraint(waypoint_designator, radius, look=False),
                           lambda: look_at_designator(look_at_designator)]
        else:
            constraints = [lambda: waypoint_constraint(waypoint_designator, radius, look=True)]

        super(NavigateToWaypoint, self).__init__(robot, lambda: combine_constraints(constraints), speak=speak)
