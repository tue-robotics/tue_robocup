from __future__ import absolute_import

# ROS
import rospy
from geometry_msgs.msg import *

# TU/e Robotics
from robot_skills.util.entity import Entity
from .navigation import NavigationConstraintsDesignator, PoseConstraint
from ..checks import check_resolve_type
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint


class WayPointConstraintsDesignator(NavigationConstraintsDesignator):
    """ Navigates to a radius from a waypoint
    :param waypoint_designator: designator resolving to the waypoint
    :param radius: allowed distance to the waypoint
    :param look: whether or not to take the orientation of the waypoint (default True)
    :param name: name of the designator
    """
    def __init__(self, waypoint_designator, radius=0.15, look=True, name=None):
        super(WayPointConstraintsDesignator, self).__init__(name=name)

        check_resolve_type(waypoint_designator, Entity)  # Check that the waypoint_designator resolves to an Entity

        self.waypoint_designator = waypoint_designator
        self.radius = radius
        self.look = look

    def _resolve(self):
        e = self.waypoint_designator.resolve()

        if not e:
            rospy.logerr(
                "WayPointConstraintDesignator: No entity could be resolved from designator '%s'" % self.waypoint_designator)
            return None

        rospy.logdebug("Navigating to waypoint '{}'".format(e.id))

        try:
            x = e.pose.frame.p.x()
            y = e.pose.frame.p.y()
            rz, _, _ = e.pose.frame.M.GetEulerZYX()
        except Exception as e:
            rospy.logerr(e)
            return None

        pc = PositionConstraint(constraint="(x-%f)^2+(y-%f)^2 < %f^2" % (x, y, self.radius), frame="/map")
        oc = None
        if self.look:
            oc = OrientationConstraint(look_at=Point(x + 10, y, 0.0), angle_offset=rz, frame="/map")
        constraint = PoseConstraint(pc, oc)
        return constraint
