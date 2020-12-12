from __future__ import absolute_import

# ROS
import rospy

# TU/e Robotics
from cb_base_navigation_msgs.srv import *
from cb_base_navigation_msgs.msg import *
from robot_skills.util.kdl_conversions import FrameStamped
from .navigation import NavigateTo
from .constraint_functions import arms_reach_constraint
from ..util.designators import Designator, check_resolve_type


class NavigateToPlace(NavigateTo):
    """
    Navigate so that the arm can reach the place point

    :param robot: robot object
    :param place_pose_designator designator that resolves to a FrameStamped
    :param arm_designator: which arm to eventually place with
    """
    def __init__(self, robot, place_pose_designator, arm_designator=None):
        check_resolve_type(place_pose_designator, FrameStamped)

        if not arm_designator:
            rospy.logerr('NavigateToPlace: side should be determined by arm_designator.'
                         'Please specify left or right, will default to left')
            arm_designator = Designator(robot.leftArm)

        super(NavigateToPlace, self).__init__(robot, lambda userdata: arms_reach_constraint(place_pose_designator,
                                                                                            arm_designator, look=True))
