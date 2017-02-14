import PyKDL as kdl

import geometry_msgs.msg as gm


class FrameStamped(object):
    """Kdl alternative for a geometry_msgs.PoseStamped.
    This class consists of a kdl.Frame and the frame_id w.r.t which the Frame is defined"""

    def __init__(self, frame, frame_id):
        assert isinstance(frame, kdl.Frame)
        self.frame = frame
        self.frame_id = frame_id

    def __repr__(self):
        return "{frame} @ {fid}".format(frame=self.frame, fid=self.frame_id)

    def projectToFrame(self, frame_id, tf_listener):
        transformed_pose = tf_listener.transformPose(frame_id, kdlFrameStampedToPoseStampedMsg(self))
        return kdlFrameStampedFromPoseStampedMsg(transformed_pose)


def pointMsgToKdlVector(point):
    """
    Convert a ROS geometry_msgs.msg.Point message to a PyKDL.Vector object
    :type point:  geometry_msgs.msg.Point
    :rtype: PyKDL.Vector

    >>> point = gm.Point(1, 2, 3)
    >>> v = pointMsgToKdlVector(point)
    >>> v.x()
    1.0
    >>> v.y()
    2.0
    >>> v.z()
    3.0
    """
    return kdl.Vector(point.x, point.y, point.z)

def kdlVectorToPointMsg(vector):
    """
    Convert a PyKDL.Vector object to a ROS geometry_msgs.msg.Point message
    :type vector: PyKDL.Vector
    :rtype: geometry_msgs.msg.Point

    >>> vector = kdl.Vector(1, 2, 3)
    >>> point = kdlVectorToPointMsg(vector)
    >>> point.x
    1.0
    >>> point.y
    2.0
    >>> point.z
    3.0
    """
    return gm.Point(vector.x(), vector.y(), vector.z())

def kdlRotationToQuaternionMsg(rotation):
    """
    Convert a PyKDL.Rotation object to a ROS geometry_msgs.msg.Quaternion message
    :param rotation: Rotation to be converted
    :type rotation: PyKDL.Rotation
    :rtype: geometry_msgs.msg.Quaternion

    >>> rot = kdl.Rotation.Quaternion(1, 0, 0, 0)
    >>> quat_msg = kdlRotationToQuaternionMsg(rot)
    >>> (quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w)
    (1.0, 0.0, 0.0, 0.0)
    >>> from math import pi
    >>> rot = kdl.Rotation.RPY(pi/2, 0, 0)
    >>> quat_msg = kdlRotationToQuaternionMsg(rot)
    >>> (quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w)
    (0.7071067811865475, 0.0, 0.0, 0.7071067811865476)
    """
    return gm.Quaternion(*rotation.GetQuaternion())

def quaternionMsgToKdlRotation(quaternion):
    """
    Convert a geometry_msgs.msg.Quaternion message to a ROS PyKDL.Rotation object
    :param quaternion: Rotation to be converted
    :type quaternion: geometry_msgs.msg.Quaternion
    :rtype: PyKDL.Rotation

    >>> quat_msg = gm.Quaternion(1.0, 0.0, 0.0, 1.0)
    >>> rot = quaternionMsgToKdlRotation(quat_msg)
    >>> rot.GetRPY()
    (1.5707963267948966, -0.0, 0.0)
    """
    return kdl.Rotation.Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)

def poseMsgToKdlFrame(pose):
    """
    Convert a geometry_msgs.msg.Pose message to a PyKDL.Frame object
    :param pose: Pose to be converted
    :type pose: geometry_msgs.msg.Pose
    :rtype: PyKDL.Frame

    >>> pose = gm.Pose(gm.Point(1, 2, 3), gm.Quaternion(1, 0, 0, 0))
    >>> frame = poseMsgToKdlFrame(pose)
    >>> frame.p
    [           1,           2,           3]
    >>> frame.M.GetQuaternion()
    (1.0, 0.0, 0.0, 0.0)
    """
    rot = kdl.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    trans = kdl.Vector(pose.position.x ,pose.position.y, pose.position.z)
    return kdl.Frame(rot, trans)

def kdlFrameToPoseMsg(frame):
    """
    Convert a ROS PyKDL.Frame object to a geometry_msgs.msg.Pose message
    :param frame: Frame to be converted
    :type frame: PyKDL.Frame
    :rtype: geometry_msgs.msg.Pose

    >>> frame = kdl.Frame(kdl.Rotation.Quaternion(1, 0, 0, 0), kdl.Vector(1, 2, 3))
    >>> pose = kdlFrameToPoseMsg(frame)
    >>> (pose.position.x, pose.position.y, pose.position.z)
    (1.0, 2.0, 3.0)
    >>> (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    (1.0, 0.0, 0.0, 0.0)
    """
    pose = gm.Pose()
    pose.position = kdlVectorToPointMsg(frame.p)
    pose.orientation = kdlRotationToQuaternionMsg(frame.M)
    return pose

def kdlFrameStampedToPoseStampedMsg(frame_stamped):
    """
    Convert a ROS PyKDL.Frame object to a geometry_msgs.msg.Pose message
    :param frame: Frame to be converted
    :type frame: PyKDL.Frame
    :rtype: geometry_msgs.msg.Pose

    >>> frame = kdl.Frame(kdl.Rotation.Quaternion(1, 0, 0, 0), kdl.Vector(1, 2, 3))
    >>> pose = kdlFrameToPoseMsg(frame)
    >>> (pose.position.x, pose.position.y, pose.position.z)
    (1.0, 2.0, 3.0)
    >>> (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    (1.0, 0.0, 0.0, 0.0)
    """
    pose_stamped = gm.PoseStamped()
    pose_stamped.header.frame_id = frame_stamped.frame_id
    pose_stamped.pose.position = kdlVectorToPointMsg(frame_stamped.frame.p)
    pose_stamped.pose.orientation = kdlRotationToQuaternionMsg(frame_stamped.frame.M)
    return pose_stamped

def kdlFrameFromXYZRPY(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    """
    Create a PyKDL.Frame from raw scalars
    :param x:
    :param y:
    :param z:
    :param roll:
    :param pitch:
    :param yaw:
    :return:
    :rtype: PyKDL.Frame
    """
    return kdl.Frame(kdl.Rotation.RPY(roll, pitch, yaw), kdl.Vector(x,y,z))

def kdlFrameStampedFromPoseStampedMsg(pose_stamped):
    """Convert a PoseStamped to FrameStamped
    :param pose_stamped the PoseStamped to be converted
    :returns FrameStamped"""
    assert isinstance(pose_stamped, gm.PoseStamped)
    return FrameStamped(frame=poseMsgToKdlFrame(pose_stamped.pose),
                        frame_id=pose_stamped.header.frame_id)

def kdlFrameStampedFromXYZRPY(x=0, y=0, z=0, roll=0, pitch=0, yaw=0, frame_id="/map"):
    """
    Create a FrameStamped from raw scalars and a frame_id
    :param x:
    :param y:
    :param z:
    :param roll:
    :param pitch:
    :param yaw:
    :param frame_id:
    :return:
    :rtype: FrameStamped
    """
    return FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(roll, pitch, yaw), kdl.Vector(x,y,z)),
                        frame_id=frame_id)

if __name__ == "__main__":
    import doctest
    doctest.testmod()
