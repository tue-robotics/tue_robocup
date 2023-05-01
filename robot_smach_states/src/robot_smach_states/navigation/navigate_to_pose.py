from __future__ import absolute_import

# TU/e Robotics
from cb_base_navigation_msgs.msg import *
from .navigation import NavigateTo
from .constraint_functions import pose_constraints


class NavigateToPose(NavigateTo):
    def __init__(self, robot, x, y, rz, radius=0.15, frame_id="map", reset_head=True, speak=True, reset_pose=True):
        """
        Navigates to a pose specified by the user.

        :param robot: (Robot) object
        :param x, y: coordinates of the goal pose
        :param rz: orientation to assume
        :param radius (default 0.15): allowed distance to the pose
        :param frame_id (default "/map"): frame in which the pose is expressed
        :param reset_head: Whether or not the head should be used for obstacle avoidance during navigation.
        :param speak: Whether or not the robot should speak during navigation
        :param reset_pose: Whether or not the robot is allowed to change its pose for navigation.
        """
        super(NavigateToPose, self).__init__(robot, lambda: pose_constraints(x, y, rz, radius, frame_id), reset_head=reset_head, speak=speak, reset_pose=reset_pose)
