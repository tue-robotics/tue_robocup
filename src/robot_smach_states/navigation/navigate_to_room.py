#! /usr/bin/env python

from robot_smach_states.navigation import NavigateTo

from cb_planner_msgs_srvs.srv import *
from cb_planner_msgs_srvs.msg import *
from geometry_msgs.msg import *

import rospy

# ----------------------------------------------------------------------------------------------------

class NavigateToRoom(NavigateTo):
    def __init__(self, robot, room_designator):
        super(NavigateToRoom, self).__init__(robot)

        self.robot               = robot
        self.room_designator     = room_designator

    def generateConstraint(self):
        # _id = self.room_designator.resolve()
        # e = self.robot.ed.get_entity(id=_id)

        # if not e:
        #     rospy.logerr("No such entity")
        #     return None

        # try:
        #     pose = e.data["pose"]
        #     x = pose["x"]
        #     y = pose["y"]
        # except KeyError:
        #     rospy.logerr(KeyError)
        #     return None

        # try:
        #     rz = e.data["pose"]["rz"]
        # except KeyError:
        #     rz = 0

        import random
        room = random.choice(["living_room", "bedroom", "hallway", "workshop"])
        if room == "living_room":
            xmin = 0.0
            xmax = 3.0
            ymin = -1.0
            ymax = 3.0
        elif room == "bedroom":
            xmin = -3.0
            xmax = -1.0
            ymin = -1.0
            ymax = 0.5
        elif room == "hallway":
            xmin = 1.5
            xmax = 3.0
            ymin = 3.5
            ymax = 5.0
        elif room == "workshop":
            xmin = -3.5
            xmax = -0.5
            ymin = 1.0
            ymax = 3.5
        else:
            rospy.logerr("Don't know where to go")

        ''' PositionConstraint '''
        pcs = "x > %f and x < %f and y > %f and y < %f"%(xmin, xmax, ymin, ymax)
        pc = PositionConstraint(constraint=pcs, frame="/map")

        ''' Orientation constraint is not too relevant, but we'll look at the center...'''
        #oc = OrientationConstraint(look_at=Point(x+1, y, 0.0), angle_offset=rz, frame="/map")
        oc = OrientationConstraint(look_at=Point((xmin+xmax)/2, (ymin+ymax)/2, 0.0), angle_offset = 3.0, frame="/map")

        return pc, oc
