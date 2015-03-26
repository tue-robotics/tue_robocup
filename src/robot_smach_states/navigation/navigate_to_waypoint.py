#! /usr/bin/env python

from robot_smach_states.navigation import NavigateTo

from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from geometry_msgs.msg import *
import ed.msg

import rospy

# ----------------------------------------------------------------------------------------------------

class NavigateToWaypoint(NavigateTo):
    def __init__(self, robot, waypoint_designator, radius = 0.15):
        """@param waypoint_designator resolves to a waypoint stored in ED"""
        super(NavigateToWaypoint, self).__init__(robot)

        self.robot               = robot

        assert(waypoint_designator.resolve_type == ed.msg.EntityInfo) #Check that the waypoint_designator resolves to an Entity
        self.waypoint_designator = waypoint_designator
        self.radius              = radius

    def generateConstraint(self):
        e = self.waypoint_designator.resolve()

        if not e:
            rospy.logerr("No such entity")
            return None

        try:
            pose = e.data["pose"]
            x = pose["x"]
            y = pose["y"]
        except KeyError:
            rospy.logerr(KeyError)
            return None

        try:
            rz = e.data["pose"]["rz"]
        except KeyError:
            rz = 0

        pc = PositionConstraint(constraint="(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, self.radius), frame="/map")
        oc = OrientationConstraint(look_at=Point(x+1, y, 0.0), angle_offset=rz, frame="/map")

        return pc, oc
