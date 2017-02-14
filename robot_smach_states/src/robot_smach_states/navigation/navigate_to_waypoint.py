#! /usr/bin/env python

from robot_smach_states.navigation import NavigateTo

from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from geometry_msgs.msg import *

from robot_smach_states.util.designators import check_resolve_type
from robot_skills.util.entity import Entity
from robot_skills.util import transformations as tf

import rospy

# ----------------------------------------------------------------------------------------------------

class NavigateToWaypoint(NavigateTo):
    def __init__(self, robot, waypoint_designator, radius = 0.15, look_at_designator=None, speak=True):
        """@param waypoint_designator resolves to a waypoint stored in ED"""
        super(NavigateToWaypoint, self).__init__(robot, speak=speak)

        self.robot               = robot

        check_resolve_type(waypoint_designator, Entity) #Check that the waypoint_designator resolves to an Entity
        if look_at_designator is not None:
            check_resolve_type(look_at_designator, Entity) #Check that the look_at_designator resolves to an Entity

        self.waypoint_designator = waypoint_designator
        self.radius              = radius
        self.look_at_designator = look_at_designator

    def generateConstraint(self):
        e = self.waypoint_designator.resolve()

        if not e:
            rospy.logerr("NavigateToWaypoint::generateConstraint: No entity could be resolved from designator '%s'" % self.waypoint_designator)
            return None

        try:
            x = e.pose.position.x
            y = e.pose.position.y
            rz = tf.euler_z_from_quaternion(e.pose.orientation)
        except:
            return None

        pc = PositionConstraint(constraint="(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, self.radius), frame="/map")

        oc = None
        if self.look_at_designator:
            look_at_e = self.look_at_designator.resolve()
            if look_at_e:
                oc = OrientationConstraint(frame=look_at_e.id)
            else:
                rospy.logerr("NavigateToWaypoint::generateConstraint: No entity could be resolved from designator '%s'" % self.look_at_designator)

        if not oc:
            oc = OrientationConstraint(look_at=Point(x+10, y, 0.0), angle_offset=rz, frame="/map")

        return pc, oc
