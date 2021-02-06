from __future__ import absolute_import

# System
import math

# ROS
import rospy
from geometry_msgs.msg import Point

# TU/e Robotics
from robot_skills.arms import PublicArm
from ...util.designators import check_type, Designator
from cb_base_navigation_msgs.msg import OrientationConstraint, PositionConstraint


def arms_reach_constraint(pose_designator, arm, look=True):
    """
    Position so that the arm can reach the position/entity

    :param pose_designator: designator that resolves to a FrameStamped of the point to be reached
    :param arm: PublicArmDesignator or arm to use for manipulation
    :param look: bool, whether or not the orientation must be constrained as well
    :return: navigation constraints, if a designator does not resolve None is returned
    :rtype: tuple(PositionConstraint, OrientationConstraint)
    """

    check_type(arm, PublicArm)  # Check that the arm is either a PublicArm or a designator to one

    if isinstance(arm, Designator):
        arm = arm.resolve()

    radius = math.hypot(arm.base_offset.x(), arm.base_offset.y())

    pose = pose_designator.resolve()
    if not pose:
        rospy.logerr("No such place_pose, Designator {} did not resolve".format(pose_designator))
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
