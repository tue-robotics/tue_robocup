#! /usr/bin/env python

from robot_smach_states.navigation import NavigateTo

from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from geometry_msgs.msg import *

from robot_smach_states.util.designators import check_resolve_type
from robot_skills.util.entity import Entity

import rospy
import math

# ----------------------------------------------------------------------------------------------------
class NavigateToObserve(NavigateTo):
    def __init__(self, robot, entity_designator, radius = .7):
        """@param entity_designator resolves to the entity the robot should observe"""
        super(NavigateToObserve, self).__init__(robot)

        self.robot    = robot
        check_resolve_type(entity_designator, Entity) #Check that the entity_designator resolves to an Entity
        self.entity_designator = entity_designator
        self.radius   = radius

        # ITS NOT DEPRECATED BECAUSE ITS DOES DIFFERENT THINGS THAN NavigateToSymbolic
        # rospy.logerr('NavigateToObserve --> THIS NAVIGATE STATEMACHINE IS DEPRECATED, Use NavigateToSymbolic instead! Check the challenge_navigation for an example!')

    def generateConstraint(self):
        e = self.entity_designator.resolve()

        if not e:
            rospy.logerr("No such entity")
            return None

        ch = e.shape.convex_hull

        #if len(ch) == 0:
        #    rospy.logerr("{0} has no convex hull so cannot NavigateToObserve there".format(e.id))
        #    return None

        x = e._pose.p.x()
        y = e._pose.p.y()

        if len(ch) > 0:

            # Only append the first part again if it's more than 1 cm apart of the last
            # if math.hypot( (ch[0].x - ch[-1].x), (ch[0].y - ch[-1].y) ) > 0.01:
            #     ch.append(ch[0])

            pci = ""

            for i in xrange(len(ch)):
                j = (i+1)%len(ch)
                dx = ch[j].x() - ch[i].x()
                dy = ch[j].y() - ch[i].y()

                length = (dx * dx + dy * dy)**.5

                xs = x + ch[i].x() + (dy/length)*self.radius
                ys = y + ch[i].y() - (dx/length)*self.radius

                if i != 0:
                    pci = pci + ' and '

                pci = pci + "-(x-%f)*%f+(y-%f)*%f > 0.0 "%(xs, dy, ys, dx)

        else:
            ro = "(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, self.radius+0.075)
            ri = "(x-%f)^2+(y-%f)^2 > %f^2"%(x, y, self.radius-0.075)
            pci = ri+" and "+ro

        pc = PositionConstraint(constraint=pci, frame="/map")
        oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="/map")

        return pc, oc
