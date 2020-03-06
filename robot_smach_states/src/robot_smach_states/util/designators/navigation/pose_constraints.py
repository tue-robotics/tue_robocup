from __future__ import absolute_import

# ROS
from geometry_msgs.msg import *

# TU/e Robotics
from .navigation import NavigationConstraintsDesignator, PoseConstraint
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint


class PoseConstraintsDesignator(NavigationConstraintsDesignator):
    """ Navigates to a radius from a pose
    :param x, y: x and y coordinates
    :param rz: orientation to assume. None=no orientation constraint (default None)
    :param radius: allowed distance to the pose
    :param frame_id: id in which the pose is expressed
    :param name: name of the designator
    """
    def __init__(self, x, y, rz=None, radius=0.15, frame_id="/map", name=None):
        super(PoseConstraintsDesignator, self).__init__(name=name)
        self.x = x
        self.y = y
        self.rz = rz
        self.radius = radius
        self._frame_id = frame_id

    def _resolve(self):
        pc = PositionConstraint(constraint="(x-%f)^2+(y-%f)^2 < %f^2" % (self.x, self.y, self.radius),
                                frame=self._frame_id)
        oc = None
        if self.rz:
            oc = OrientationConstraint(look_at=Point(self.x + 1, self.y, 0.0), angle_offset=self.rz, frame=self._frame_id)

        constraint = PoseConstraint(pc, oc)
        return constraint
