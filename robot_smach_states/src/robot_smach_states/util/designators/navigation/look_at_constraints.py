from __future__ import absolute_import

# ROS
import rospy
from geometry_msgs.msg import *

# TU/e Robotics
from .navigation import NavigationConstraintsDesignator
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import kdl_vector_to_point_msg
from .. import check_resolve_type


class LookAtConstraintsDesignator(NavigationConstraintsDesignator):
    """
    Constructor

    :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at. This is used
    to compute the orientation constraint.
    :param name: Optional name of the constraint designator
    """
    def __init__(self, entity_lookat_designator, offset=0.0, name=None):
        super(LookAtConstraintsDesignator, self).__init__(name=name)
        check_resolve_type(entity_lookat_designator, Entity)  # Check that the entity_designator resolves to an Entity
        self.entity_lookat_designator = entity_lookat_designator
        self.offset = offset

    def _resolve(self):
        return self.generate_constraint(self.entity_lookat_designator, self.offset)

    @staticmethod
    def generate_constraint(entity_lookat_designator, offset=0.0):
        """
        Staticmethod generating the orientation constraint.
        By implementing this as a staticmethod, it can also be used for other purposes.

        :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at. This is used
            to compute the orientation constraint.
        :param offset: offset the angle with respect to the entity
        :return: (tuple(PositionConstraint, OrientationConstraint)). If the entity does not resolve, None is returned.
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
