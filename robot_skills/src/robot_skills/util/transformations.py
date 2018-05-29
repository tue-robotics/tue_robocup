#!/usr/bin/env python

# System
import math

# ROS
import geometry_msgs.msg
import rospy
import tf


def euler_z_to_quaternion(angle):
    """Calculate a quaternion based on a Z angle

    >>> from math import pi
    >>> euler_z_to_quaternion(pi/2)
    x: 0.0
    y: 0.0
    z: 0.707106781187
    w: 0.707106781187
    """
    orientation_goal = geometry_msgs.msg.Quaternion()
    try:
        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
    except TypeError, te:
        rospy.logerr(str(angle) + " cannot be parsed as a float: {0}".format(te))
    orientation_goal.x = quaternion[0]
    orientation_goal.y = quaternion[1]
    orientation_goal.z = quaternion[2]
    orientation_goal.w = quaternion[3]

    return orientation_goal


def euler_z_from_quaternion(quaternion):
    """Return the yaw (z) angle of a quaternion

    >>> from geometry_msgs.msg import Quaternion
    >>> q = Quaternion()
    >>> euler_z_from_quaternion(q)
    0.0
    """
    try:
        [rx,ry,rz] = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

    except TypeError, te:
        rospy.logerr("Quaternion {0} cannot be transformed to Euler".format(te))
        return None

    return rz


def compute_relative_angle(absolute_angle, robot_orientation):

    robot_rotation=[robot_orientation.x,robot_orientation.y,robot_orientation.z,robot_orientation.w]
    robot_absolute_angle = tf.transformations.euler_from_quaternion(robot_rotation,'sxyz')

    relative_angle = absolute_angle - robot_absolute_angle[2]

    return relative_angle


def transform_into_base_coordinates(target_position, robot_position, robot_orientation):

    angle = compute_angle(target_position, robot_position)
    relative_angle = compute_relative_angle(angle, robot_orientation)
    distance = compute_distance(target_position, robot_position)
    rx = math.cos(relative_angle)*distance
    ry = math.sin(relative_angle)*distance
    rz = target_position.z
    return [rx, ry, rz]


def compute_distance(target_position, robot_position):
        dx = target_position.x - robot_position.x
        dy = target_position.y - robot_position.y
        distance = math.sqrt(dx*dx+dy*dy)
        return distance


def compute_angle(target_position, robot_position):

        dx = target_position.x - robot_position.x
        dy = target_position.y - robot_position.y
        angle = math.atan2(dy,dx)
        return angle


def transform_into_non_conflicting_position(target_position, robot_position, radius=0.6):

        dx = robot_position.x - target_position.x
        dy = robot_position.y - target_position.y
        distance = math.sqrt(dx*dx + dy*dy)

        dx = radius * (dx / distance)
        dy = radius * (dy / distance)

        target_position.x += dx
        target_position.y += dy

        return target_position
        """
        angle = atan2(dy,dx)

        if distance > radius:
            target_position.x = robot_position.x + cos(angle)*(distance - radius)
            target_position.y = robot_position.y + sin(angle)*(distance - radius)

        return target_position
        """


def tf_transform(coordinates, inputframe, outputframe, tf_listener):
    # Should probably be called transform_point

    if isinstance(coordinates, geometry_msgs.msg.Point):
        ps = geometry_msgs.msg.PointStamped(point=coordinates)
        ps.header.frame_id = inputframe
        ps.header.stamp = rospy.Time()
    elif isinstance(coordinates, geometry_msgs.msg.PointStamped):
        #If coordinates is already a PointStamped, dont make one.
        ps = coordinates
    else:
        rospy.logerr("coordinates is neither a Point nor a PointStamped? Trying my best, but might crash or work due to duck-typing")
        ps = geometry_msgs.msg.PointStamped(point=coordinates)
        ps.header.frame_id = inputframe
        ps.header.stamp = rospy.Time()

    output_coordinates = tf_listener.transformPoint(outputframe, ps)
    return output_coordinates.point


if __name__ == '__main__':
    import doctest
    doctest.testmod()
