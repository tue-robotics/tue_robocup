from __future__ import absolute_import

from builtins import range

# ROS
from geometry_msgs.msg import *
import rospy

# TU/e Robotics
from cb_base_navigation_msgs.msg import *
import ed_msgs.msg
from .navigation import NavigateTo
from ..util.designators import check_resolve_type


class NavigateToExplore(NavigateTo):
    def __init__(self, robot, constraint_designator, breakout_designator, radius=0.7, exclude_radius=0.3, reset_head=True, speak=True, reset_pose=True):
        """
        :param constraint_designator a Designator that resolves to the entity to explore
        :param breakout_designator when this Designator successfully resolves, the state signals it is done.
        For example, it could resolve to an item of a class you are looking for
        :param reset_head: Whether or not the head should be used for obstacle avoidance during navigation.
        :param speak: Whether or not the robot should speak during navigation
        :param reset_pose: Whether or not the robot is allowed to change its pose for navigation.
        """
        super(NavigateToExplore, self).__init__(robot, self.generateConstraint, reset_head=reset_head, speak=speak, reset_pose=reset_pose)

        self.robot = robot

        check_resolve_type(constraint_designator, ed_msgs.msg.EntityInfo)  # Check that the constraint designator resolves to an Entity
        self.constraint_designator = constraint_designator

        check_resolve_type(breakout_designator, ed_msgs.msg.EntityInfo)  # Check that the constraint designator resolves to an Entity
        self.breakout_designator = breakout_designator
        self.radius = radius
        self.exclude_radius = exclude_radius
        self.visited_list = []

    def generateConstraint(self):

        # Add current pose to visited list
        current_pose = self.robot.base.get_location().frame
        self.visited_list.append(current_pose)

        e = self.constraint_designator.resolve()

        if not e:
            rospy.logerr("No such entity")
            return None

        rospy.logdebug("Navigating to explore entity '{}'".format(e.uuid))

        ch = e.convex_hull

        x = e.pose.frame.p.x()
        y = e.pose.frame.p.y()

        ch.append(ch[0])

        pci = ""

        # Loop over convex hull and add to constraint
        for i in range(len(ch) - 1):
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
            xe = self.visited_list[i].p.x()
            ye = self.visited_list[i].p.y()
            rospy.loginfo('xe = {0}, ye = {1}'.format(xe,ye))

            pci = pci + ' and '
            pci = pci + "(x-%f)^2+(y-%f)^2 > %f^2"%(xe, ye, self.exclude_radius)

        pc = PositionConstraint(constraint=pci, frame="map")
        oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="map")

        return pc, oc

    def breakOut(self):

        entity = self.breakout_designator.resolve()
        if not entity:
            return True

        rospy.loginfo("Breakout: entity_id = {0}".format(entity.uuid))

        return False
