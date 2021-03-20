from __future__ import absolute_import

# TU/e Robotics
from robot_skills.util.kdl_conversions import FrameStamped
from .navigation import NavigateTo
from .constraint_functions import arms_reach_constraint
from ..util.designators import  check_resolve_type


class NavigateToPlace(NavigateTo):
    def __init__(self, robot, place_pose_designator, arm_designator):
        """
        Navigate so that the arm can reach the place point

        :param robot: robot object
        :param place_pose_designator designator that resolves to a FrameStamped
        :param arm_designator: which arm to eventually place with
        """
        check_resolve_type(place_pose_designator, FrameStamped)

        super(NavigateToPlace, self).__init__(robot, lambda: arms_reach_constraint(place_pose_designator,
                                                                                   arm_designator,
                                                                                   look=True))
