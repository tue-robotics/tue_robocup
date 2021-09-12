from __future__ import absolute_import

# ROS
import rospy

from geometry_msgs.msg import PointStamped

import tf2_ros
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs
# noinspection PyUnresolvedReferences
import tf2_pykdl_ros

# TU/e Robotics
from cb_base_navigation_msgs.msg import OrientationConstraint, PositionConstraint
from pykdl_ros import VectorStamped


def look_at_constraint(entity_lookat_designator, offset=0.0):
    """
    Generate navigation constraints for looking at an entity

    :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at.
    :param offset: offset the angle with respect to the entity
    :return: navigation constraints. If the entity does not resolve, None is returned.
    :rtype: tuple(PositionConstraint, OrientationConstraint)
    """
    # Orientation constraint is the entity itself...
    entity_lookat = entity_lookat_designator.resolve()
    if not entity_lookat:
        rospy.logerr("Could not resolve entity_lookat_designator".format(entity_lookat_designator))
        return None
    vector_stamped = VectorStamped(entity_lookat.pose, rospy.Time(), entity_lookat.frame_id)
    look_at = tf2_ros.convert(vector_stamped, PointStamped).point

    pc = None
    oc = OrientationConstraint(look_at=look_at, angle_offset=offset, frame=entity_lookat.pose.frame_id)
    return pc, oc
