from __future__ import absolute_import

# System
import math

# ROS
import rospy
from geometry_msgs.msg import Point

# TU/e Robotics
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint


def arms_reach_constraint(pose_designator, arm_designator, look=True):
    """
    Position so that the arm can reach the position/entity
    :param pose_designator: designator that resolves to a FrameStamped
    :param arm_designator: which arm to use for manipulation
    :param look: whether or not the orientation must be constrained as well
    :return: tuple(PositionConstraint, OrientationConstraint)
    """
    arm = arm_designator.resolve()
    if not arm:
        rospy.logerr("Could not resolve arm")
        return None

    radius = math.hypot(arm.base_offset.x(), arm.base_offset.y())

    pose = pose_designator.resolve()
    if not pose:
        rospy.logerr("No such place_pose")
        return None

    try:
        x = pose.frame.p.x()
        y = pose.frame.p.y()
    except KeyError as ke:
        rospy.logerr("Could not determine pose: ".format(ke))
        return None

    # Outer radius
    ro = "(x-%f)^2+(y-%f)^2 < %f^2" % (x, y, radius + 0.075)
    ri = "(x-%f)^2+(y-%f)^2 > %f^2" % (x, y, radius - 0.075)
    pc = PositionConstraint(constraint=ri + " and " + ro, frame="/map")

    oc = None
    if look:
        angle_offset = -math.atan2(arm.base_offset.y(), arm.base_offset.x())
        oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="/map", angle_offset=angle_offset)

    return pc, oc
