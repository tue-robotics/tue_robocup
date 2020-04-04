from __future__ import absolute_import

# ROS
import rospy

# TU/e Robotics
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint
from robot_skills.util.kdl_conversions import kdl_vector_to_point_msg


def look_at_constraint(entity_lookat_designator, offset=0.0):
    """
    Generate navigation constraints for looking at an entity

    :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at.
    :param offset: offset the angle with respect to the entity
    :return: tuple(PositionConstraint, OrientationConstraint). If the entity does not resolve, None is returned.
    """
    # Orientation constraint is the entity itself...
    entity_lookat = entity_lookat_designator.resolve()
    if not entity_lookat:
        rospy.logerr("Could not resolve entity_lookat_designator".format(entity_lookat_designator))
        return None
    look_at = kdl_vector_to_point_msg(entity_lookat.pose.extractVectorStamped().vector)

    pc = None
    oc = OrientationConstraint(look_at=look_at, angle_offset=offset, frame=entity_lookat.pose.frame_id)
    return pc, oc
