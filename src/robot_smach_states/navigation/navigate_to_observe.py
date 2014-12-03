#! /usr/bin/env python

from robot_smach_states.navigation import NavigateTo

from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from geometry_msgs.msg import *

import rospy

# ----------------------------------------------------------------------------------------------------

class NavigateToObserve(NavigateTo):
    def __init__(self, robot, entity_id, radius = 1):
        super(NavigateToObserve, self).__init__(robot)

        self.robot    = robot
        self.entity_id = entity_id
        self.radius   = radius

    def generateConstraint(self):
        e = self.robot.ed.getEntity(id=self.entity_id)

        if not e:
            rospy.logerr("No such entity")
            return None

        x = e.pose.position.x
        y = e.pose.position.y    

        print "DFGDFG" 

        rospy.logwarn("Should Sjoerd check the newest model data in???")

        pc = PositionConstraint(constraint="(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, self.radius), frame="/map")
        oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="/map")

        return pc, oc

    def breakOut(self):
        return False