#! /usr/bin/env python

from robot_smach_states.navigation import NavigateTo

from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from geometry_msgs.msg import *
from robot_skills.util.kdl_conversions import kdlVectorToPointMsg

from robot_smach_states.util.designators import check_resolve_type
from robot_skills.util.entity import Entity

import rospy

# ----------------------------------------------------------------------------------------------------


class NavigateToSymbolic(NavigateTo):
    """ Navigation class to navigate to a semantically annotated goal, e.g., in front of the dinner table.
    """
    def __init__(self, robot, entity_designator_area_name_map, entity_lookat_designator):
        """ Constructor

        :param robot: robot object
        :param entity_designator_area_name_map: dictionary mapping EdEntityDesignators to a string or designator
        resolving to a string, representing the area, e.g., entity_designator_area_name_map[<EdEntity>] = 'in_front_of'.
        :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at. This is used
        to compute the orientation constraint.
        """
        super(NavigateToSymbolic, self).__init__(robot)

        self.robot = robot

        # Check that the entity_designator_area_name_map's keys all resolve to EntityInfo's
        assert(all(entity_desig.resolve_type == Entity for entity_desig in entity_designator_area_name_map.keys()))
        self.entity_designator_area_name_map = entity_designator_area_name_map

        check_resolve_type(entity_lookat_designator, Entity)  # Check that the entity_designator resolves to an Entity
        self.entity_lookat_designator = entity_lookat_designator

    def generateConstraint(self):
        """ PositionConstraint """
        entity_id_area_name_map = {}
        for desig, area_name in self.entity_designator_area_name_map.iteritems():
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

        pc = self.robot.ed.navigation.get_position_constraint(entity_id_area_name_map)

        # Orientation constraint is the entity itself...
        entity_lookat = self.entity_lookat_designator.resolve()
        if not entity_lookat:
            rospy.logerr("Could not resolve entity_lookat_designator".format(self.entity_lookat_designator))
            return None

        look_at = kdlVectorToPointMsg(entity_lookat.pose.extractVectorStamped().vector)
        oc = OrientationConstraint(look_at=look_at, frame=entity_lookat.pose.frame_id)

        return pc, oc


class NavigateToRoom(NavigateToSymbolic):
    def __init__(self, robot, entity_designator_area, entity_lookat_designator):
        """
        Navigate to a room, which only has the area: 'in'
        :param robot: robot object
        :type robot: robot
        :param entity_designator_area: Designator to the area
        :type entity_designator_area: EdEntityDesignator
        :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at. This is used
        to compute the orientation constraint.
        :type entity_lookat_designator: EdEntityDesignator
        """
        area = "in"
        super(NavigateToRoom, self).__init__(robot, {entity_designator_area: area}, entity_lookat_designator)
