#! /usr/bin/env python

from robot_smach_states.navigation import NavigateTo

from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from geometry_msgs.msg import *

from robot_smach_states.util.designators import check_resolve_type
from robot_skills.util.entity import Entity

import rospy

# ----------------------------------------------------------------------------------------------------

class NavigateToSymbolic(NavigateTo):
    def __init__(self, robot, entity_designator_area_name_map, entity_lookat_designator):
        super(NavigateToSymbolic, self).__init__(robot)

        self.robot                 = robot

        #Check that the entity_designator_area_name_map's keys all resolve to EntityInfo's
        assert(all(entity_desig.resolve_type == Entity for entity_desig in entity_designator_area_name_map.keys()))
        self.entity_designator_area_name_map     = entity_designator_area_name_map

        check_resolve_type(entity_lookat_designator, Entity) #Check that the entity_designator resolves to an Entity
        self.entity_lookat_designator = entity_lookat_designator

    def generateConstraint(self):
        ''' PositionConstraint '''
        entity_id_area_name_map = {}
        for desig, area_name in self.entity_designator_area_name_map.iteritems():
            entity = desig.resolve()
            if entity:
                entity_id_area_name_map[entity.id] = area_name
            else:
                rospy.logerr("Designator {0} in entity_designator_area_name_map resolved to {1}.".format(desig, entity))
                entity_id_area_name_map[entity] = area_name #Put a None item in the dict. We check on that and if there's a None, something failed.

        if None in entity_id_area_name_map:
            rospy.logerr("At least 1 designator in self.entity_designator_area_name_map failed")
            return None


        pc = self.robot.ed.navigation.get_position_constraint(entity_id_area_name_map)

        #Orientation constraint is the entity itself...
        entity_lookat = self.entity_lookat_designator.resolve()
        if not entity_lookat:
            rospy.logerr("Could not resolve entity_lookat_designator".format(self.entity_lookat_designator))
            return None

        oc = OrientationConstraint(look_at=entity_lookat.pose.position, frame="/map")

        return pc, oc
