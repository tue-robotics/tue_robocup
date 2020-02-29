from __future__ import absolute_import

# System
import inspect
import pprint
import abc

# ROS
import rospy
from geometry_msgs.msg import *

# TU/e Robotics
from robot_skills.util.entity import Entity
from ..core import Designator
from ..checks import check_resolve_type
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint


class NavigationConstraintsDesignator(Designator):
    __metaclass__ = abc.ABCMeta

    def __init__(self, name=None):
        super(NavigationConstraintsDesignator, self).__init__(resolve_type=tuple, name=name)

    @abc.abstractmethod
    def _resolve(self):
        pass

