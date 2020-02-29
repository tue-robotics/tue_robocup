from __future__ import absolute_import

# ROS
from geometry_msgs.msg import *

# TU/e Robotics
from .navigation import NavigationConstraintsDesignator
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint


class PoseConstraintsDesignator(NavigationConstraintsDesignator):
    def __init__(self, x, y, rz, radius=0.15, frame_id="/map", name=None):
        super(PoseConstraintsDesignator, self).__init__(name=name)
        self.x = x
        self.y = y
        self.rz = rz
        self.radius = radius
        self._frame_id = frame_id

    def _resolve(self):
        pc = PositionConstraint(constraint="(x-%f)^2+(y-%f)^2 < %f^2" % (self.x, self.y, self.radius),
                                frame=self._frame_id)
        oc = OrientationConstraint(look_at=Point(self.x + 1, self.y, 0.0), angle_offset=self.rz, frame=self._frame_id)

        return pc, oc
