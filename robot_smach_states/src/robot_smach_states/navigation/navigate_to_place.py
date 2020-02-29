from __future__ import absolute_import

# TU/e Robotics
from . import NavigateToDesignator
from ..util.designators.navigation import PlaceConstraintsDesignator


# ----------------------------------------------------------------------------------------------------
class NavigateToPlace(NavigateToDesignator):
    def __init__(self, robot, place_pose_designator, arm_designator=None):
        """Navigate so that the arm can reach the place point
        :param place_pose_designator designator that resolves to a geometry_msgs.msg.PoseStamped
        :param arm which arm to eventually place with?
        """
        constraint_designator = PlaceConstraintsDesignator(robot, place_pose_designator, arm_designator)
        super(NavigateToPlace, self).__init__(robot, constraint_designator)
