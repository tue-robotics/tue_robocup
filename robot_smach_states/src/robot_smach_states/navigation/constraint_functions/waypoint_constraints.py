from __future__ import absolute_import

# ROS
import rospy
from geometry_msgs.msg import Point

# TU/e Robotics
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint


def waypoint_constraint(waypoint_designator, radius, look=True):
    """
    Navigates to a radius from a waypoint

    :param waypoint_designator: designator resolving to the waypoint
    :param radius: allowed distance to the waypoint
    :param look: whether or not to take the orientation of the waypoint (default True)
    :return: navigation constraints, if the entity does not resolve, None is returned.
    :type: tuple(PositionConstraint, OrientationConstraint)
    """
    e = waypoint_designator.resolve()

    if not e:
        rospy.logerr(
            "waypoint_constraint function: No entity could be resolved from designator '%s'" % waypoint_designator)
        return None

    rospy.logdebug("Navigating to waypoint '{}'".format(e.id))

    try:
        x = e.pose.frame.p.x()
        y = e.pose.frame.p.y()
        rz, _, _ = e.pose.frame.M.GetEulerZYX()
    except Exception as e:
        rospy.logerr(e)
        return None

    pc = PositionConstraint(constraint="(x-%f)^2+(y-%f)^2 < %f^2" % (x, y, radius), frame="/map")
    oc = None
    if look:
        oc = OrientationConstraint(look_at=Point(x + 10, y, 0.0), angle_offset=rz, frame="/map")
    return pc, oc
