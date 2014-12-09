#! /usr/bin/env python

from robot_smach_states.navigation import NavigateTo

from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from geometry_msgs.msg import *

import rospy

import math


# ----------------------------------------------------------------------------------------------------
class NavigateToGrasp(NavigateTo):
    def __init__(self, robot, designator, side):
        super(NavigateToGrasp, self).__init__(robot)

        self.robot    = robot
        self.designator = designator
        if side == 'left' or side == 'right':
            self.side = side
        else:
            rospy.logerr('NavigateToGrasp: side = {0}. Please specify left or right, will default to left'.format(side))
            self.side = 'left'

    def generateConstraint(self):

        x_offset = self.robot.grasp_offset.x
        if self.side == 'left':
            y_offset = self.robot.grasp_offset.y
        elif self.side == 'right':
            y_offset = -self.robot.grasp_offset.y
        radius = math.sqrt(x_offset*x_offset + y_offset*y_offset)

        entity_id = self.designator.resolve()
        e = self.robot.ed.get_entity(entity_id)

        if not e:
            rospy.logerr("No such entity")
            return None

        print e

        if not e:
            rospy.logerr("No such entity")
            return None

        try:
            pose = e.pose #TODO Janno: Not all entities have pose information
            x = pose.position.x
            y = pose.position.y
        except KeyError, ke:
            rospy.logerr("Could not determine pose: ".format(ke))
            return None

        try:
            rz = e.pose.orientation.z
        except KeyError, ke:
            rospy.logerr("Could not determine pose.rz: ".format(ke))
            rz = 0

        rospy.logwarn("Should Sjoerd check the newest model data in???")

        # Outer radius
        ro = "(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, radius+0.075)
        ri = "(x-%f)^2+(y-%f)^2 > %f^2"%(x, y, radius-0.075)
        pc = PositionConstraint(constraint=ri+" and "+ro, frame="/map")
        oc = OrientationConstraint(look_at=Point(x_offset, y_offset, 0.0), frame="/map")

        return pc, oc