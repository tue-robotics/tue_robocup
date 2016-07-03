#! /usr/bin/env python

from robot_smach_states.navigation import NavigateTo

from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from geometry_msgs.msg import *

import rospy

# ----------------------------------------------------------------------------------------------------

class NavigateToPose(NavigateTo):
    def __init__(self, robot, x, y, rz, radius=0.15, frame_id="/map"):
        super(NavigateToPose, self).__init__(robot)

        self.x = x
        self.y = y
        self.rz = rz
        self.radius = radius
        self._frame_id = frame_id

    def generateConstraint(self):
        pc = PositionConstraint(constraint="(x-%f)^2+(y-%f)^2 < %f^2"%(self.x, self.y, self.radius), frame=self._frame_id)
        oc = OrientationConstraint(look_at=Point(self.x+1, self.y, 0.0), angle_offset=self.rz, frame=self._frame_id)

        return pc, oc


# class NavigateToPose(NavigateTo):
#     def __init__(self, robot, x, y, rz):
#         NavigateTo.__init__(robot, constraint_args={'x': x, 'y': y, 'rz': rz})

#     def generateConstraint(self, userdata):
#         userdata.position_constraint = PositionConstraint(constraint="(x-%d)+(x-%d)^2 < %d"%(x,y,radius), frame="/map")
#         userdata.orientation_constraint = OrientationConstraint(look_at=Point(x+1,y), angle_offset=rz, frame="/map")

#         return 'succeeded'

#     def breakOut(self):
#         if bla:
#             return True
#         else:
#             return False




