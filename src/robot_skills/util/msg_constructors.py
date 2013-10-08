#!/usr/bin/python

"""Contstruct complicated ROS messages as easy as possible"""
import roslib; roslib.load_manifest("robot_skills")
import rospy

import std_msgs.msg as std
import geometry_msgs.msg as gm

import transformations

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

def PointStamped(x=0, y=0, z=0, frame_id="/map", stamp=None):
    if not stamp:
        stamp = rospy.get_rostime()

    return gm.PointStamped(header=Header(frame_id, stamp), point=Point(x,y,z))

def Quaternion(x=0, y=0, z=0, w=0):
    return gm.Quaternion(x, y, z, w)

def Pose(x=0, y=0, z=0, phi=0):
    quat = transformations.euler_z_to_quaternion(phi)
    pos = Point(x ,y, z)

    return gm.Pose(pos, quat)

def PoseStamped2D(x=0, y=0, z=0, phi=0, frame_id="/map", stamp=None, pointstamped=None):
    """Build a posestamped from any number of arguments"""
    number = (int, long, float)
    
    if not stamp:
        stamp = rospy.get_rostime()

    if pointstamped:
        return gm.PoseStamped(header=pointstamped.header, 
                              pose=gm.Pose(pointstamped.point, Quaternion()))
    elif isinstance(x, number) and isinstance(y, number) and isinstance(z, number):
        return gm.PoseStamped(header=Header(frame_id, stamp), 
                              pose=Pose(x, y, z, phi))

if __name__ == "__main__":
    rospy.init_node("msg_constructors_tester")

    import doctest
    doctest.testmod()