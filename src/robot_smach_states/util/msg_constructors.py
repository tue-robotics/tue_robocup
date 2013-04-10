#!/usr/bin/python

"""Contstruct complicated ROS messages as easy as possible"""
import roslib; roslib.load_manifest("robot_smach_states")
import rospy

import std_msgs.msg as std
import geometry_msgs.msg as gm

import transformations

def Point(x=0, y=0,z=0):
    return gm.Point(x,y,z)

def Header(frame_id="/map", stamp=None):
    if not stamp:
        _time = rospy.Time.now()
    else:
        _time = stamp
    header = std.Header(stamp=_time, frame_id=frame_id)

    return header

def PointStamped(x=0, y=0,z=0, frame_id="/map", stamp=None):
    return gm.PointStamped(header=Header(frame_id, stamp), point=Point(x,y,z))

def Quaternion(x=0, y=0,z=0, w=0):
    return gm.Quaternion(x, y,z, w)

def Pose2D(x=0, y=0, phi=0):
    quat = transformations.euler_z_to_quaternion(phi)
    pos = Point(x,y)

    return gm.Pose(pos, quat)

def PoseStamped2D(x=0, y=0, phi=0, frame_id="/map", stamp=None):
    return gm.PoseStamped(header=Header(frame_id, stamp), pose=Pose2D(x,y,phi))