from __future__ import absolute_import

import math

# ROS
import rospy

# TU/e Robotics
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint


def radius_constraint(entity, radius, margin):
    """
    Navigates to a radius from an ed entity. If a convex hull is present, the distance
    to the convex hull is used. Otherwise, the distance to the pose of the entity is used

    :param entity: EdEntityDesignator for the object to observe
    :param radius: (float) desired distance to the pose of the entity [m]
    :param margin: (float) allowed margin w.r.t. specified radius on both sides [m]
    :return: tuple(PositionConstraint, OrientationConstraint). If the entity does not resolve, None is returned.
    """
    e = entity.resolve()

    if not e:
        rospy.logerr("No entity from {}".format(entity))
        return None

    try:
        ch = e.shape.convex_hull
    except NotImplementedError:
        # In case of an entity without convex hull, we might not want to use this
        ch = []

    x = e.pose.frame.p.x()
    y = e.pose.frame.p.y()
    assert e.pose.frame_id.strip("/") == "map", "radius constraint function assumes entities are defined w.r.t. map frame"

    if len(ch) > 0:  # If a convex hull is present, use this to create the position constraint
        pci = ""  # Create an empty position constraint

        for i in xrange(len(ch)):  # Iterate over the convex hull
            j = (i + 1) % len(ch)
            dx = ch[j].x() - ch[i].x()
            dy = ch[j].y() - ch[i].y()

            length = math.hypot(dx, dy)

            xs = x + ch[i].x() + (dy / length) * radius
            ys = y + ch[i].y() - (dx / length) * radius

            if i != 0:
                pci = pci + ' and '

            pci = pci + "-(x-%f)*%f+(y-%f)*%f > 0.0 " % (xs, dy, ys, dx)

    else:  # If not, simply use the x and y position
        outer_radius = max(0., radius + margin)
        inner_radius = max(0., radius - margin)

        ro = "(x-%f)^2+(y-%f)^2 < %f^2" % (x, y, outer_radius)
        ri = "(x-%f)^2+(y-%f)^2 > %f^2" % (x, y, inner_radius)
        pci = ri + " and " + ro

    pc = PositionConstraint(constraint=pci, frame="/map")  # Create the position constraint from the string
    oc = None
    return pc, oc
