#! /usr/bin/env python

from robot_smach_states.navigation import NavigateTo

from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from geometry_msgs.msg import *

import rospy


# ----------------------------------------------------------------------------------------------------
class NavigateToExplore(NavigateTo):
    def __init__(self, robot, entity_id, radius = .7, exclude_radius = 0.3):
        super(NavigateToExplore, self).__init__(robot)

        self.robot    = robot
        self.entity_id = entity_id
        self.radius   = radius
        self.exclude_radius = exclude_radius
        self.visited_list = []

    def generateConstraint(self):

        # Add current pose to visited list
        current_pose = self.robot.base.get_location()
        self.visited_list.append(current_pose)

        e = self.robot.ed.get_entity(id=self.entity_id)

        if not e:
            rospy.logerr("No such entity")
            return None

        ch = e.convex_hull

        x = e.pose.position.x
        y = e.pose.position.y

        ch.append(ch[0])

        pci = ""

        # Loop over convex hull and add to constraint
        for i in xrange(len(ch) - 1):
            dx = ch[i+1].x - ch[i].x
            dy = ch[i+1].y - ch[i].y

            length = (dx * dx + dy * dy)**.5

            xs = ch[i].x + (dy/length)*self.radius
            ys = ch[i].y - (dx/length)*self.radius
            
            if i != 0:
                pci = pci + ' and '

            pci = pci + "-(x-%f)*%f+(y-%f)*%f > 0.0 "%(xs, dy, ys, dx)

        # Loop over visited list
        for i in range(len(self.visited_list)):
            xe = self.visited_list[i].pose.position.x
            ye = self.visited_list[i].pose.position.y
            rospy.loginfo('xe = {0}, ye = {1}'.format(xe,ye))

            pci = pci + ' and '
            pci = pci + "(x-%f)^2+(y-%f)^2 > %f^2"%(xe, ye, self.exclude_radius)


        pc = PositionConstraint(constraint=pci, frame="/map")
        oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="/map")

        return pc, oc

    def breakOut(self):
        return False