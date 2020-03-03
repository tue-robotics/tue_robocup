from __future__ import absolute_import

# System
import math

# ROS
import rospy
from geometry_msgs.msg import *

# TU/e Robotics
from .navigation import NavigationConstraintsDesignator
from .look_at_constraints import LookAtConstraintsDesignator
from cb_planner_msgs_srvs.msg import PoseConstraint, OrientationConstraint, PositionConstraint
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import FrameStamped
from robot_skills.arms import PublicArm
from .. import check_resolve_type
from ..arm import ArmDesignator


class ArmsreachConstraintsDesignator(NavigationConstraintsDesignator):
    """Position so that the arm can reach the position/entity
    :param designator: designator that resolves to a either a geometry_msgs.msg.PoseStamped or an Ed Entity with a pose
    :param arm_designator: which arm to use for manipulation
    :param look: whether or not the orientation must be constrained as well
    :param name: Optional name of the designator
    """
    def __init__(self, robot, designator, arm_designator=None, look=True, name=None):
        super(ArmsreachConstraintsDesignator, self).__init__(name=name)

        self.robot = robot
        self.look = look
        self.entity_designator = None
        self.pose_designator = None

        check_resolve_type(designator, Entity, FrameStamped)
        if designator.resolve_type is Entity:
            self.entity_designator = designator
        else:
            self.pose_designator = designator

        if arm_designator:
            check_resolve_type(arm_designator, PublicArm)  # Check that the arm_designator resolves to an Arm
            self.arm_designator = arm_designator
        else:
            rospy.logwarn('ArmsreachConstraintsDesignator: Please specify the arm, will choose at random')
            self.arm_designator = ArmDesignator(self.robot, {})

    def _resolve(self):
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return None

        radius = math.hypot(arm.base_offset.x(), arm.base_offset.y())

        if self.entity_designator:
            entity = self.entity_designator.resolve()

            if not entity:
                rospy.logerr("No such entity")
                return None

            try:
                pose = entity.pose
            except KeyError as ke:
                rospy.logerr("Could not determine pose: ".format(ke))
                return None
        else: # we have a pose designator
            pose = self.pose_designator.resolve()
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
        if self.look:
            angle_offset = -math.atan2(arm.base_offset.y(), arm.base_offset.x())
            oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="/map", angle_offset=angle_offset)

        constraint = PoseConstraint(pc=pc, oc=oc)
        return constraint
