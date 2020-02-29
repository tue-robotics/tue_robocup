from __future__ import absolute_import

# System
import math

# ROS
import rospy
from geometry_msgs.msg import *

# TU/e Robotics
from .. import Designator
from .navigation import NavigationConstraintsDesignator
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint

from robot_skills.util.kdl_conversions import FrameStamped
from .. import check_resolve_type


class PlaceConstraintsDesignator(NavigationConstraintsDesignator):
    def __init__(self, robot, place_pose_designator, arm_designator=None, name=None):
        """Position so that the arm can reach the place point
        :param place_pose_designator: designator that resolves to a geometry_msgs.msg.PoseStamped
        :param arm: which arm to eventually place with?
        :param name: Optional name of the designator
        """
        super(PlaceConstraintsDesignator, self).__init__(name=name)

        self.robot = robot
        # Check that place_pose_designator actually returns a PoseStamped
        check_resolve_type(place_pose_designator, FrameStamped)
        self.place_pose_designator = place_pose_designator

        self.arm_designator = arm_designator
        if not arm_designator:
            rospy.logerr('NavigateToPlace: side should be determined by arm_designator.'
                         'Please specify left or right, will default to left')
            self.arm_designator = Designator(robot.leftArm)

    def _resolve(self):
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return None

        angle_offset = -math.atan2(arm.base_offset.y(), arm.base_offset.x())
        radius = math.hypot(arm.base_offset.x(), arm.base_offset.y())

        place_fs = self.place_pose_designator.resolve()

        if not place_fs:
            rospy.logerr("No such place_pose")
            return None

        rospy.loginfo("Navigating to place at {0}".format(place_fs).replace('\n', ' '))

        try:
            x = place_fs.frame.p.x()
            y = place_fs.frame.p.y()
        except KeyError as ke:
            rospy.logerr("Could not determine pose: ".format(ke))
            return None

        # Outer radius
        radius -= 0.1
        ro = "(x-%f)^2+(y-%f)^2 < %f^2" % (x, y, radius + 0.075)
        ri = "(x-%f)^2+(y-%f)^2 > %f^2" % (x, y, radius - 0.075)
        pc = PositionConstraint(constraint=ri + " and " + ro, frame=place_fs.frame_id)
        oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame=place_fs.frame_id, angle_offset=angle_offset)

        return pc, oc

