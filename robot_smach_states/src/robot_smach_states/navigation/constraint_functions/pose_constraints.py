from __future__ import absolute_import

# ROS
from geometry_msgs.msg import Point

# TU/e Robotics
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint


def pose_constraints(x, y, rz=None, radius=0.15, frame_id="/map"):
    """
    Generate navigation constraints for a radius from a pose
    :param x, y: x and y coordinates
    :param rz: orientation to assume. None=no orientation constraint (default None)
    :param radius: allowed distance to the pose
    :param frame_id: frame in which the pose is expressed
    :return: tuple(PositionConstraint, OrientationConstraint)
    """
    pc = PositionConstraint(constraint="(x-%f)^2+(y-%f)^2 < %f^2" % (x, y, radius),
                            frame=frame_id)
    oc = None
    if rz:
        oc = OrientationConstraint(look_at=Point(x + 1, y, 0.0), angle_offset=rz, frame=frame_id)
    return pc, oc
