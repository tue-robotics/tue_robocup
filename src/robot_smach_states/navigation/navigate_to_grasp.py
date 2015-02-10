#! /usr/bin/env python

from robot_smach_states.navigation import NavigateTo

from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from geometry_msgs.msg import *

from util.designators import Designator

import rospy

import math


# ----------------------------------------------------------------------------------------------------
class NavigateToGrasp(NavigateTo):
    def __init__(self, robot, entity_designator, arm_designator=None):
        super(NavigateToGrasp, self).__init__(robot)

        self.robot    = robot
        self.entity_designator = entity_designator

        self.arm_designator = arm_designator
        if not arm_designator:
            rospy.logerr('NavigateToGrasp: side should be determined by entity_designator. Please specify left or right, will default to left')
            self.arm_designator = Designator(robot.leftArm)

    def generateConstraint(self):
        arm = self.arm_designator.resolve()
        side = arm.side

        x_offset = self.robot.grasp_offset.x
        if side == 'left':
            y_offset = self.robot.grasp_offset.y
        elif side == 'right':
            y_offset = -self.robot.grasp_offset.y
        radius = math.sqrt(x_offset*x_offset + y_offset*y_offset)

        entity = self.entity_designator.resolve()

        if not entity:
            rospy.logerr("No such entity")
            return None

        print entity

        if not entity:
            rospy.logerr("No such entity")
            return None

        try:
            pose = entity.pose #TODO Janno: Not all entities have pose information
            x = pose.position.x
            y = pose.position.y
        except KeyError, ke:
            rospy.logerr("Could not determine pose: ".format(ke))
            return None

        try:
            rz = entity.pose.orientation.z
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