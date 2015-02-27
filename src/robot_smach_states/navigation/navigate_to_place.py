#! /usr/bin/env python

from robot_smach_states.navigation import NavigateTo

from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from geometry_msgs.msg import *

from robot_smach_states.util.designators import Designator

import rospy

import math


# ----------------------------------------------------------------------------------------------------
class NavigateToPlace(NavigateTo):
    def __init__(self, robot, place_pose_designator, arm_designator=None):
        """Navigate so that the arm can reach the place point
        @param place_pose_designator designator that resolves to a geometry_msgs.msg.PoseStamped
        @param arm which arm to eventually place with?
        """
        super(NavigateToPlace, self).__init__(robot)

        self.robot    = robot
        self.place_pose_designator = place_pose_designator

        self.arm_designator = arm_designator
        if not arm_designator:
            rospy.logerr('NavigateToPlace: side should be determined by arm_designator. Please specify left or right, will default to left')
            self.arm_designator = Designator(robot.leftArm)

    def generateConstraint(self):
        arm = self.arm_designator.resolve()
        
        x_offset = self.robot.grasp_offset.x
        if arm == self.robot.arms['left']:
            y_offset = self.robot.grasp_offset.y
        elif arm == self.robot.arms['right']:
            y_offset = -self.robot.grasp_offset.y
        radius = math.sqrt(x_offset*x_offset + y_offset*y_offset)

        place_pose = self.place_pose_designator.resolve()

        if not place_pose:
            rospy.logerr("No such place_pose")
            return None

        print place_pose

        if not place_pose:
            rospy.logerr("No such place_pose")
            return None

        try:
            x = place_pose.pose.position.x
            y = place_pose.pose.position.y
        except KeyError, ke:
            rospy.logerr("Could not determine pose: ".format(ke))
            return None

        rospy.logwarn("Should Sjoerd check the newest model data in???")

        # Outer radius
        ro = "(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, radius+0.075)
        ri = "(x-%f)^2+(y-%f)^2 > %f^2"%(x, y, radius-0.075)
        pc = PositionConstraint(constraint=ri+" and "+ro, frame=place_pose.header.frame_id)
        oc = OrientationConstraint(look_at=Point(x_offset, y_offset, 0.0), frame=place_pose.header.frame_id)

        return pc, oc