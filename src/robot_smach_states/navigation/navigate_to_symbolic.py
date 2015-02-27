#! /usr/bin/env python

from robot_smach_states.navigation import NavigateTo

from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from geometry_msgs.msg import *

import rospy

# ----------------------------------------------------------------------------------------------------

class NavigateToSymbolic(NavigateTo):
    def __init__(self, robot, entity_designator, area_name):
        super(NavigateToSymbolic, self).__init__(robot)

        self.robot                 = robot
        self.entity_designator     = entity_designator
        self.area_name             = area_name

    def generateConstraint(self):
        entity_id = self.entity_designator.resolve().id

        ''' PositionConstraint '''
        pc = self.robot.ed.navigation.get_position_constraint(entity_id, self.area_name)

        ''' Orientation constraint is the entity itself...'''
        oc = OrientationConstraint(frame=entity_id)

        return pc, oc
