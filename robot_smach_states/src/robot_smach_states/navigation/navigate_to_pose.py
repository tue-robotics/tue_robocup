from __future__ import absolute_import

# ROS
from geometry_msgs.msg import *

# TU/e Robotics
from cb_planner_msgs_srvs.msg import *
from .navigation import NavigateTo
from .constraint_functions import pose_constraints


# ----------------------------------------------------------------------------------------------------

class NavigateToPose(NavigateTo):
    def __init__(self, robot, x, y, rz, radius=0.15, frame_id="/map"):
        super(NavigateToPose, self).__init__(robot, lambda: pose_constraints(x, y, rz, radius, frame_id))
