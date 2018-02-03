#!/usr/bin/python

"""Construct complicated ROS messages as easy as possible"""
# ROS
import geometry_msgs.msg as gm
import rospy
import std_msgs.msg as std
import tf

number = (int, long, float)


def Point(x=0, y=0, z=0):
    """Make a Point
    >>> Point(1.1, 2.2, 3.3)
    x: 1.1
    y: 2.2
    z: 3.3
    """
    return gm.Point(x,y,z)


def Header(frame_id="/map", stamp=None):
    """Make a Header
    >>> h = Header("/base_link")
    >>> assert h.stamp.secs > 0
    >>> assert h.stamp.nsecs > 0
    """
    if not stamp:
        _time = rospy.Time.now()
    else:
        _time = stamp
    header = std.Header(stamp=_time, frame_id=frame_id)

    return header


def PointStamped(x=0, y=0, z=0, frame_id="/map", stamp=None, point=None):
    if not stamp:
        stamp = rospy.get_rostime()
    if not point:
      return gm.PointStamped(header=Header(frame_id, stamp), point=Point(x,y,z))
    else:
      return gm.PointStamped(header=Header(frame_id, stamp), point=point)


def Quaternion(x=0, y=0, z=0, w=0, roll=0, pitch=0, yaw=0):
    """Supply either at least one of x, y, z, w
    or at least one of roll, pitch, yaw."""
    if x or y or z or w:
        return gm.Quaternion(x, y, z, w)
    elif roll or pitch or yaw:
        quat_parts = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        quat = Quaternion(*quat_parts)
        return quat
    else:
        # Assume unit quaternion
        return gm.Quaternion(0.0,0.0,0.0,1.0)


def Pose(x=0, y=0, z=0, phi=0, roll=0, pitch=0, yaw=0):
    """
    >>> pose = Pose(yaw=0.5)
    >>> pose
    position:
      x: 0
      y: 0
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.247403959255
      w: 0.968912421711
    """
    if phi:
      rospy.logerr("Please specify yaw instead of phi. Phi will be removed!!!")

    z_rotation = phi or yaw #Get the one that is not 0

    quat = Quaternion(roll=roll, pitch=pitch, yaw=z_rotation)

    pos = Point(x ,y, z)

    return gm.Pose(pos, quat)


def PoseStamped(x=0, y=0, z=0, phi=0,
    roll=0, pitch=0, yaw=0,
    frame_id="/map", stamp=None, pointstamped=None):
    """Build a geometry_msgs.msgs.PoseStamped from any number of arguments.
    Each value defaults to 0
    >>> ps = PoseStamped(yaw=0.5)
    >>> ps.pose
    position:
      x: 0
      y: 0
      z: 0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.247403959255
      w: 0.968912421711
    """
    if not stamp:
        stamp = rospy.get_rostime()

    if pointstamped:
        return gm.PoseStamped(header=pointstamped.header,
                              pose=gm.Pose(pointstamped.point, Quaternion()))
    elif isinstance(x, number) and isinstance(y, number) and isinstance(z, number):
        return gm.PoseStamped(header=Header(frame_id, stamp),
                              pose=Pose(x, y, z, phi, roll, pitch,yaw))
    else:
        raise ValueError("Either supply a number for x, y and z or a PointStamped to pointstamped")


if __name__ == "__main__":
    rospy.init_node("msg_constructors_tester")

    import doctest
    doctest.testmod()
