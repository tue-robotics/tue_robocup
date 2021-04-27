from __future__ import absolute_import

# TU/e Robotics
from cb_base_navigation_msgs.msg import *
from .navigation import NavigateTo
from .constraint_functions import pose_constraints


class NavigateToPose(NavigateTo):
    def __init__(self, robot, x, y, rz, radius=0.15, frame_id="map"):
        """
        Navigates to a pose specified by the user.

        :param robot: (Robot) object
        :param x, y: coordinates of the goal pose
        :param rz (optional): orientation to assume. if not specified no orientation constraint is used.
        :param radius (default 0.15): allowed distance to the pose
        :param frame_id (default "/map"): frame in which the pose is expressed
        """
        super(NavigateToPose, self).__init__(robot, lambda: pose_constraints(x, y, rz, radius, frame_id))
