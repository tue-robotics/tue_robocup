from __future__ import absolute_import

# TU/e Robotics
from . import NavigateToDesignator
from ..util.designators.navigation import ArmsreachConstraintsDesignator


# ----------------------------------------------------------------------------------------------------
class NavigateToPlace(NavigateToDesignator):
    """Navigate so that the arm can reach the place point
    :param robot: robot object
    :param place_pose_designator designator that resolves to a geometry_msgs.msg.PoseStamped
    :param arm_designator: which arm to eventually place with?
    """
    def __init__(self, robot, place_pose_designator, arm_designator=None):
        constraint_designator = ArmsreachConstraintsDesignator(robot, place_pose_designator, arm_designator, look=True)
        super(NavigateToPlace, self).__init__(robot, constraint_designator)
