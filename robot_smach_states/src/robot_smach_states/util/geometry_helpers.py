# System
import math

# ROS
import PyKDL as kdl
import rospy

# TU/e Robotics
from robot_skills.util.kdl_conversions import point_msg_to_kdl_vector


def wrap_angle_pi(angle):
    """
    Wraps between -pi and +pi

    :param angle: Input angle
    :return: Wrapped angle
    """
    angle = angle % (2 * math.pi)
    if angle > math.pi:
        return angle - 2 * math.pi
    elif angle < -math.pi:
        return angle + 2 * math.pi
    return angle


def isLeftOfLine(p, l):
    """ Checks whether point p is left of line l
        p: geometry_msgs.Point
        l: array of size two of geometry_msgs.Point.
        Note that only x and y are evaluated
    """
    A = l[0]
    B = l[1]
    if ( (B.x() - A.x()) * (p.y() - A.y()) - (B.y() - A.y()) * (p.x() - A.x()) ) > 0:
        return True
    else:
        return False
    #position = sign( (Bx-Ax)*(Y-Ay) - (By-Ay)*(X-Ax) )


def isPointInsideHull(p, chull):
    """ Checks whether point p is inside the convex hull ch
        p: geometry_msgs.Point
        ch: array of geometry_msgs.Point. Note that the order is supposed to be anti-clockwise
        Note that only p.x and p.y are evaluated
    """

    ''' Copy list '''
    ch = list(chull)

    ''' Loop over lines of chull '''
    for i in xrange(len(ch)):
        j = (i+1)%len(ch)
        ''' Check whether the point is left of the line '''
        if not isLeftOfLine(p, [ch[i], ch[j]]):
            return False

    return True


def onTopOff(subject, container, ht=0.1):
    """ Checks whether the entity 'subject' is on top of entity 'container'
        @param subject the EntityInfo which may be on top of the container, e.g. a cup
        @param container: the EntityInfo that may be supporting the other subject, e.g. a table.
        ht: height threshold: the bottom of entity and the top of container need to be within ht m
    """
    ''' First: check if container actually has a convex hull '''
    if len(container.convex_hull) == 0:
        rospy.logerr('Entity {0} has no convex hull'.format(container.id))
        return False

    ''' Second: turn points into KDL objects and offset '''
    convex_hull_obj = [point_msg_to_kdl_vector(p) for p in container.convex_hull]   # convex hull in object frame
    convex_hull = offsetConvexHull(convex_hull_obj, container.pose.frame)  # convex hull in map frame

    ''' Third: check if center point of entity is within convex hull of container '''
    subject_center_pose = subject.pose.frame
    if not isPointInsideHull(subject_center_pose.p, convex_hull):
        return False

    subject_bottom = subject.pose.frame.p.z()+subject.z_min
    container_top = container.pose.frame.p.z()+container.z_max
    if math.fabs(subject_bottom - container_top) > ht:
        return False

    return True


def onTopOffForDesignator(container_designator):
    """Returns a function that will tell if an entity is on top of the entity designated by container_designator.
    This function can be used as a designator-criterium.
    E.g. criteria_funcs = [onTopOffForDesignator(EdEntityDesignator(...))]"""
    def on_top(entity):
        container_entity = container_designator.resolve()
        return onTopOff(entity, container_entity)
    return on_top


def offsetConvexHull(input_ch, offset):
    """ Offsets the the convex hull 'input_ch' with 'offset'. This can be used, e.g., when a convex hull is desired in map
    frame while it is given in object frame. In that case, 'offset' represents the object pose in map frame.

    :param input_ch: list with kdl Vectors
    :param offset: KDL frame representing the offset with which to multiply the convex_hull
    :return: list with KDL vectors representing the convex hull
    """
    out_ch = []
    for p in input_ch:
        pf = kdl.Frame(kdl.Rotation(), p)  # ToDo: is this necessary???
        pf = offset * pf
        p_out = kdl.Vector(pf.p)
        out_ch.append(p_out)
    return out_ch
