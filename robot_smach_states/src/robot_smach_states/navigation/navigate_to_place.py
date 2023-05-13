from __future__ import absolute_import

# TU/e Robotics
from pykdl_ros import FrameStamped
from .navigation import NavigateTo
from .constraint_functions import arms_reach_constraint
from ..util.designators import  check_resolve_type


class NavigateToPlace(NavigateTo):
    def __init__(self, robot, place_pose_designator, arm_designator, reset_head=True, speak=True, reset_pose=True):
        """
        Navigate so that the arm can reach the place point

        :param robot: robot object
        :param place_pose_designator designator that resolves to a FrameStamped
        :param arm_designator: which arm to eventually place with
        :param reset_head: Whether or not the head should be used for obstacle avoidance during navigation.
        :param speak: Whether or not the robot should speak during navigation
        :param reset_pose: Whether or not the robot is allowed to change its pose for navigation.
        """
        check_resolve_type(place_pose_designator, FrameStamped)

        super(NavigateToPlace, self).__init__(robot,
                                              lambda: arms_reach_constraint(place_pose_designator,
                                                                            arm_designator,
                                                                            look=True),
                                              reset_head=reset_head,
                                              speak=speak,
                                              reset_pose=reset_pose)
