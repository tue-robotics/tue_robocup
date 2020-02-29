from __future__ import absolute_import

# System
import inspect
import pprint

# ROS
import rospy
from geometry_msgs.msg import *

# TU/e Robotics
from robot_skills.util.entity import Entity
from ..core import Designator
from ..checks import check_resolve_type
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint


class WayPointConstaintsDesignator(Designator):
    def __init__(self, robot, waypoint_designator, radius=0.15, look_at_designator=None, name=None):
        super(WayPointConstaintsDesignator, self).__init__(resolve_type=tuple, name=name)
        self.robot = robot

        check_resolve_type(waypoint_designator, Entity)  # Check that the waypoint_designator resolves to an Entity
        if look_at_designator:
            check_resolve_type(look_at_designator, Entity)  # Check that the look_at_designator resolves to an Entity

        self.waypoint_designator = waypoint_designator
        self.radius = radius
        self.look_at_designator = look_at_designator

    def _resolve(self):
        e = self.waypoint_designator.resolve()

        if not e:
            rospy.logerr(
                "NavigateToWaypoint::generateConstraint: No entity could be resolved from designator '%s'" % self.waypoint_designator)
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
        if self.look_at_designator:
            look_at_e = self.look_at_designator.resolve()
            if look_at_e:
                oc = OrientationConstraint(frame=look_at_e.id)
            else:
                rospy.logerr(
                    "NavigateToWaypoint::generateConstraint: No entity could be resolved from designator '%s'" % self.look_at_designator)

        if not oc:
            oc = OrientationConstraint(look_at=Point(x + 10, y, 0.0), angle_offset=rz, frame="/map")

        return pc, oc
