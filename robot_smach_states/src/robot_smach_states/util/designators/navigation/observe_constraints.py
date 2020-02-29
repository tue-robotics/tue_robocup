from __future__ import absolute_import

# ROS
import rospy
from geometry_msgs.msg import *

# TU/e Robotics
from .navigation import NavigationConstraintsDesignator
from cb_planner_msgs_srvs.msg import OrientationConstraint, PositionConstraint
from .. import check_resolve_type
from robot_skills.util.entity import Entity


class ObserveConstraintsDesignator(NavigationConstraintsDesignator):
    """ Navigates to a radius from an ed entity. If a convex hull is present, the distance
    to the convex hull is used. Otherwise, the distance to the pose of the entity is used
    """
    def __init__(self, robot, entity_designator, radius=0.7, margin=0.075, name=None):
        """
        :param robot: (Robot) object
        :param entity_designator: EdEntityDesignator for the object to observe
        :param radius: (float) desired distance to the pose of the entity [m]
        :param margin: (float) allowed margin w.r.t. specified radius on both sides [m]
        """
        super(ObserveConstraintsDesignator, self).__init__(name=name)
        self.robot = robot
        check_resolve_type(entity_designator, Entity)  # Check that the entity_designator resolves to an Entity
        self.entity_designator = entity_designator
        self.radius = radius
        self.margin = margin

    def _resolve(self):
        e = self.entity_designator.resolve()

        if not e:
            rospy.logerr("No entity from {}".format(self.entity_designator))
            return None

        rospy.logdebug("Navigating to observe entity '{}'".format(e.id))

        try:
            ch = e.shape.convex_hull
        except NotImplementedError:
            # In case of an entity without convex hull, we might not want to use this
            ch = []

        x = e.pose.frame.p.x()
        y = e.pose.frame.p.y()
        assert e.pose.frame_id.strip("/") == "map", "ObserveConstraintsDesignator assumes entities are defined w.r.t. map frame"

        if len(ch) > 0:  # If a convex hull is present, use this to create the position constraint

            # Only append the first part again if it's more than 1 cm apart of the last
            # if math.hypot( (ch[0].x - ch[-1].x), (ch[0].y - ch[-1].y) ) > 0.01:
            #     ch.append(ch[0])

            pci = ""  # Create an empty position constraint

            for i in xrange(len(ch)):  # Iterate over the convex hull
                j = (i + 1) % len(ch)
                dx = ch[j].x() - ch[i].x()
                dy = ch[j].y() - ch[i].y()

                length = (dx * dx + dy * dy) ** .5

                xs = x + ch[i].x() + (dy / length) * self.radius
                ys = y + ch[i].y() - (dx / length) * self.radius

                if i != 0:
                    pci = pci + ' and '

                pci = pci + "-(x-%f)*%f+(y-%f)*%f > 0.0 " % (xs, dy, ys, dx)

        else:  # If not, simply use the x and y position
            outer_radius = max(0., self.radius + self.margin)
            inner_radius = max(0., self.radius - self.margin)

            ro = "(x-%f)^2+(y-%f)^2 < %f^2" % (x, y, outer_radius)
            ri = "(x-%f)^2+(y-%f)^2 > %f^2" % (x, y, inner_radius)
            pci = ri + " and " + ro

        pc = PositionConstraint(constraint=pci, frame="/map")  # Create the position constraint from the string
        oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="/map")  # Create the orientation constaint, looking
        # at the entity pose

        return pc, oc

