from __future__ import absolute_import

# System
import math

# ROS
import rospy
from geometry_msgs.msg import Point

# TU/e Robotics
from robot_skills.arm.arms import PublicArm
from cb_base_navigation_msgs.msg import OrientationConstraint, PositionConstraint

# Robot smach states
from ...util.designators import check_type, ArmDesignator, AttrDesignator, Designator


def determine_offsets(pose_designator, arm_designator):
    # type: (pose_designator, arm_designator) -> Tuple[FrameStamped, float, float]
    arm = arm_designator.resolve()
    if not arm:
        rospy.logerr("Could not resolve arm")
        return None

    angle_offset = -math.atan2(arm.base_offset.y(), arm.base_offset.x())
    radius = math.hypot(arm.base_offset.x(), arm.base_offset.y())

    pose = pose_designator.resolve()
    if not pose:
        rospy.logerr("No such place_pose, Designator {} did not resolve".format(pose_designator))
        return None

    pose = pose_designator.resolve()
    if not pose:
        raise RuntimeError("Could not resolve pose to determine offsets")

    return pose, radius, angle_offset


def arms_reach_constraint(pose_designator, arm_designator, look=True):
    """
    Position so that the arm can reach the position/entity

    :param pose_designator: designator that resolves to a FrameStamped of the point to be reached
    :param arm_designator: PublicArmDesignator or arm to use for manipulation
    :param look: bool, whether or not the orientation must be constrained as well
    :return: navigation constraints, if a designator does not resolve None is returned
    :rtype: tuple(PositionConstraint, OrientationConstraint)
    """
    # ToDo: what do we do with the 'look' parameter?
    try:
        pose, radius, angle_offset = determine_offsets(pose_designator, arm_designator)
    except RuntimeError as e:
        rospy.logerr(e.message)
        return None

    x = pose.frame.p.x()
    y = pose.frame.p.y()

    # Outer radius
    ro = "(x-%f)^2+(y-%f)^2 < %f^2" % (x, y, radius + 0.075)
    ri = "(x-%f)^2+(y-%f)^2 > %f^2" % (x, y, radius - 0.075)
    pc = PositionConstraint(constraint=ri + " and " + ro, frame="/map")
    oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="/map", angle_offset=angle_offset)

    return pc, oc
