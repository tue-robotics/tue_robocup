from __future__ import absolute_import

# ROS
import rospy
from geometry_msgs.msg import *

# TU/e Robotics
from .navigation import NavigationConstraintsDesignator
from cb_planner_msgs_srvs.msg import PoseConstraint, OrientationConstraint, PositionConstraint
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import kdl_vector_to_point_msg
from .. import check_resolve_type


class SymbolicConstraintsDesignator(NavigationConstraintsDesignator):
    """
    Constructor

    :param robot: robot object
    :param entity_designator_area_name_map: dictionary mapping EdEntityDesignators to a string or designator
     resolving to a string, representing the area, e.g., entity_designator_area_name_map[<EdEntity>] = 'in_front_of'.
    :param name: Optional name of the constraint designator
    """
    def __init__(self, robot, entity_designator_area_name_map, name=None):
        super(SymbolicConstraintsDesignator, self).__init__(name=name)
        self.robot = robot

        # Check that the entity_designator_area_name_map's keys all resolve to EntityInfo's
        assert (all(entity_desig.resolve_type == Entity for entity_desig in entity_designator_area_name_map.keys()))
        self.entity_designator_area_name_map = entity_designator_area_name_map

    def _resolve(self):
        return self.generate_constraint(self.robot, self.entity_designator_area_name_map)

    @staticmethod
    def generate_constraint(robot, entity_designator_area_name_map):
        """
        Staticmethod generating the position and orientation constraint.
        By implementing this as a staticmethod, it can also be used for other purposes.

        :param robot: robot object
        :param entity_designator_area_name_map: dictionary mapping EdEntityDesignators to a string or designator
            resolving to a string, representing the area, e.g., entity_designator_area_name_map[<EdEntity>] = 'in_front_of'.
        :return: (tuple(PositionConstraint, OrientationConstraint)). If one of the entities does not resolve,
            None is returned.
        """
        entity_id_area_name_map = {}
        for desig, area_name in entity_designator_area_name_map.iteritems():
            entity = desig.resolve()
            try:
                area_name = area_name.resolve()
            except:
                pass
            if entity:
                entity_id_area_name_map[entity.id] = area_name
            else:
                rospy.logerr("Designator {0} in entity_designator_area_name_map resolved to {1}.".format(desig, entity))
                # Put a None item in the dict. We check on that and if there's a None, something failed.
                entity_id_area_name_map[entity] = area_name

        if None in entity_id_area_name_map:
            rospy.logerr("At least 1 designator in self.entity_designator_area_name_map failed")
            return None

        rospy.logdebug("Navigating to symbolic {}".format(entity_id_area_name_map))

        pc = robot.ed.navigation.get_position_constraint(entity_id_area_name_map)
        oc = None
        constraint = PoseConstraint(pc=pc, oc=oc)
        return constraint


class RoomConstraintsDesignator(SymbolicConstraintsDesignator):
    def __init__(self, robot, entity_designator_room):
        """
        :param robot: robot object
        :type robot: robot
        :param entity_designator_room: Designator to the room
        :type entity_designator_room: Designator resolving to an ED entity
        """
        room_area = "in"
        super(RoomConstraintsDesignator, self).__init__(robot, {entity_designator_room: room_area})
