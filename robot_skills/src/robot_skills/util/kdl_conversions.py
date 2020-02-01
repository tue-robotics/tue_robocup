# System
from copy import deepcopy

# ROS
import geometry_msgs.msg as gm
import PyKDL as kdl
import rospy


class FrameStamped(object):
    """Kdl alternative for a geometry_msgs.PoseStamped.

    This class consists of a kdl.Frame and the frame_id w.r.t which the Frame is defined"""

    def __init__(self, frame, frame_id, stamp=None):
        assert isinstance(frame, kdl.Frame)
        self.frame = frame
        self.frame_id = frame_id
        self.stamp = stamp

    def __repr__(self):
        xyz = "(x={x}, y={y}, z={z})".format(x=self.frame.p.x(), y=self.frame.p.y(), z=self.frame.p.z())
        r, p, y = self.frame.M.GetRPY()
        rpy = "(r={x}, p={y}, y={z})".format(x=r, y=p, z=y)
        return "FrameStamped(pos:{pos}, rot:{rot} @ {fid})".format(pos=xyz, rot=rpy, fid=self.frame_id)

    def projectToFrame(self, frame_id, tf_listener):
        """
        Computes a new FrameStamped from this frame in the requested frame ID

        :param frame_id: (str) frame id of the new FrameStamped
        :param tf_listener: (TF Listener) tf listener used for the computation of the transformation
        :return: (FrameStamped) with provided frame id
        """
        tf_listener.waitForTransform(self.frame_id, frame_id, time=rospy.Time(0), timeout=rospy.Duration(1))
        transformed_pose = tf_listener.transformPose(frame_id, kdl_frame_stamped_to_pose_stamped_msg(self))
        return kdl_frame_stamped_from_pose_stamped_msg(transformed_pose)

    def extractVectorStamped(self):
        """Extract only the position of this FrameStamped, without the orientation but with the frame_id metadata

        :return: VectorStamped
        >>> fs = FrameStamped(kdl.Frame(kdl.Rotation.Quaternion(1, 0, 0, 0), kdl.Vector(1, 2, 3)), "/map")
        >>> fs.extractVectorStamped()
        VectorStamped([           1,           2,           3] @ /map)
        """
        return deepcopy(VectorStamped(frame_id=self.frame_id, vector=self.frame.p))


def frame_stamped(frame_id, x, y, z, roll=0, pitch=0, yaw=0):
    """ Creates a FrameStamped object from the provided parameters

    :param frame_id: string with frame id
    :param x:
    :param y:
    :param z:
    :param roll:
    :param pitch:
    :param yaw:
    :return:
    """
    return FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(roll, pitch, yaw),
                                        kdl.Vector(x, y, z)),
                        frame_id=frame_id)


class VectorStamped(object):
    def __init__(self, x=0, y=0, z=0, frame_id="/map", vector=None):
        if vector:
            self.vector = vector
        else:
            self.vector = kdl.Vector(x, y, z)
        self.frame_id = frame_id

    def __repr__(self):
        return "VectorStamped({vector} @ {fid})".format(vector=self.vector, fid=self.frame_id)

    def projectToFrame(self, frame_id, tf_listener):
        tf_listener.waitForTransform(self.frame_id, frame_id, time=rospy.Time(0), timeout=rospy.Duration(1))
        transformed_point = tf_listener.transformPoint(frame_id, kdl_vector_stamped_to_point_stamped(self))
        return kdl_vector_stamped_from_point_stamped_msg(transformed_point)

    def __eq__(self, other):
        if isinstance(other, VectorStamped):
            return self.vector == other.vector and \
                   self.frame_id == other.frame_id
        else:
            return False


def point_msg_to_kdl_vector(point):
    """
    Convert a ROS geometry_msgs.msg.Point message to a PyKDL.Vector object

    :type point:  geometry_msgs.msg.Point
    :rtype: PyKDL.Vector

    >>> point = gm.Point(1, 2, 3)
    >>> v = point_msg_to_kdl_vector(point)
    >>> v.x()
    1.0
    >>> v.y()
    2.0
    >>> v.z()
    3.0
    """
    return kdl.Vector(point.x, point.y, point.z)

def kdl_vector_to_point_msg(vector):
    """
    Convert a PyKDL.Vector object to a ROS geometry_msgs.msg.Point message

    :type vector: PyKDL.Vector
    :rtype: geometry_msgs.msg.Point

    >>> vector = kdl.Vector(1, 2, 3)
    >>> point = kdl_vector_to_point_msg(vector)
    >>> point.x
    1.0
    >>> point.y
    2.0
    >>> point.z
    3.0
    """
    return gm.Point(vector.x(), vector.y(), vector.z())

def kdl_rotation_to_quaternion_msg(rotation):
    """
    Convert a PyKDL.Rotation object to a ROS geometry_msgs.msg.Quaternion message

    :param rotation: Rotation to be converted
    :type rotation: PyKDL.Rotation
    :rtype: geometry_msgs.msg.Quaternion

    >>> rot = kdl.Rotation.Quaternion(1, 0, 0, 0)
    >>> quat_msg = kdl_rotation_to_quaternion_msg(rot)
    >>> (quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w)
    (1.0, 0.0, 0.0, 0.0)
    >>> from math import pi
    >>> rot = kdl.Rotation.RPY(pi/2, 0, 0)
    >>> quat_msg = kdl_rotation_to_quaternion_msg(rot)
    >>> (quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w)
    (0.7071067811865475, 0.0, 0.0, 0.7071067811865476)
    """
    return gm.Quaternion(*rotation.GetQuaternion())

def quaternion_msg_to_kdl_rotation(quaternion):
    """
    Convert a geometry_msgs.msg.Quaternion message to a ROS PyKDL.Rotation object

    :param quaternion: Rotation to be converted
    :type quaternion: geometry_msgs.msg.Quaternion
    :rtype: PyKDL.Rotation

    >>> quat_msg = gm.Quaternion(1.0, 0.0, 0.0, 1.0)
    >>> rot = quaternion_msg_to_kdl_rotation(quat_msg)
    >>> rot.GetRPY()
    (1.5707963267948966, -0.0, 0.0)
    """
    return kdl.Rotation.Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)

def pose_msg_to_kdl_frame(pose):
    """
    Convert a geometry_msgs.msg.Pose message to a PyKDL.Frame object

    :param pose: Pose to be converted
    :type pose: geometry_msgs.msg.Pose
    :rtype: PyKDL.Frame

    >>> pose = gm.Pose(gm.Point(1, 2, 3), gm.Quaternion(1, 0, 0, 0))
    >>> frame = pose_msg_to_kdl_frame(pose)
    >>> frame.p
    [           1,           2,           3]
    >>> frame.M.GetQuaternion()
    (1.0, 0.0, 0.0, 0.0)
    """
    rot = kdl.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    trans = kdl.Vector(pose.position.x ,pose.position.y, pose.position.z)
    return kdl.Frame(rot, trans)

def kdl_frame_to_pose_msg(frame):
    """
    Convert a ROS PyKDL.Frame object to a geometry_msgs.msg.Pose message

    :param frame: Frame to be converted
    :type frame: PyKDL.Frame
    :rtype: geometry_msgs.msg.Pose

    >>> frame = kdl.Frame(kdl.Rotation.Quaternion(1, 0, 0, 0), kdl.Vector(1, 2, 3))
    >>> pose = kdl_frame_to_pose_msg(frame)
    >>> (pose.position.x, pose.position.y, pose.position.z)
    (1.0, 2.0, 3.0)
    >>> (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    (1.0, 0.0, 0.0, 0.0)
    """
    pose = gm.Pose()
    pose.position = kdl_vector_to_point_msg(frame.p)
    pose.orientation = kdl_rotation_to_quaternion_msg(frame.M)
    return pose

def kdl_frame_stamped_to_pose_stamped_msg(frame_stamped):
    """
    Convert a ROS PyKDL.Frame object to a geometry_msgs.msg.Pose message

    :param frame: Frame to be converted
    :type frame: PyKDL.Frame
    :rtype: geometry_msgs.msg.Pose

    >>> frame = kdl.Frame(kdl.Rotation.Quaternion(1, 0, 0, 0), kdl.Vector(1, 2, 3))
    >>> pose = kdl_frame_to_pose_msg(frame)
    >>> (pose.position.x, pose.position.y, pose.position.z)
    (1.0, 2.0, 3.0)
    >>> (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    (1.0, 0.0, 0.0, 0.0)
    """
    pose_stamped = gm.PoseStamped()
    pose_stamped.header.frame_id = frame_stamped.frame_id
    pose_stamped.pose.position = kdl_vector_to_point_msg(frame_stamped.frame.p)
    pose_stamped.pose.orientation = kdl_rotation_to_quaternion_msg(frame_stamped.frame.M)
    return pose_stamped

def kdl_frame_from_XYZRPY(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    """
    Create a PyKDL.Frame from raw scalars

    :param x: The X value of the position
    :param y: The Y value of the position
    :param z: The Z value of the position
    :param roll: The roll of the orientation
    :param pitch: The pitch of the orientation
    :param yaw: The yaw of the orientation
    :return: the given position and orientation represented as a kdl.Frame
    :rtype: PyKDL.Frame
    """
    return kdl.Frame(kdl.Rotation.RPY(roll, pitch, yaw), kdl.Vector(x,y,z))

def kdl_frame_stamped_from_pose_stamped_msg(pose_stamped):
    """Convert a PoseStamped to FrameStamped

    :param pose_stamped: the PoseStamped to be converted
    :return: FrameStamped"""
    assert isinstance(pose_stamped, gm.PoseStamped)
    return FrameStamped(frame=pose_msg_to_kdl_frame(pose_stamped.pose),
                        frame_id=pose_stamped.header.frame_id,
                        stamp=pose_stamped.header.stamp)

def kdl_frame_stamped_from_XYZRPY(x=0, y=0, z=0, roll=0, pitch=0, yaw=0, frame_id="/map"):
    """
    Create a FrameStamped from raw scalars

    :param x: The X value of the position
    :param y: The Y value of the position
    :param z: The Z value of the position
    :param roll: The roll of the orientation
    :param pitch: The pitch of the orientation
    :param yaw: The yaw of the orientation
    :param frame_id: the frame_id in which the frame is defined
    :return: the given position and orientation represented as a kdl.Frame
    :rtype: FrameStamped
    """
    return FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(roll, pitch, yaw), kdl.Vector(x,y,z)),
                        frame_id=frame_id)

def kdl_vector_stamped_from_point_stamped_msg(point_stamped):
    """Convert a PointStamped to VectorStamped

    :param point_stamped: the PointStamped to be converted
    :return: VectorStamped"""
    assert isinstance(point_stamped, gm.PointStamped), "point_stamped is not a geometry_msgs/PointStamped but a {}".format(type(point_stamped))
    return VectorStamped(vector=point_msg_to_kdl_vector(point_stamped.point),
                         frame_id=point_stamped.header.frame_id)

def kdl_vector_stamped_to_point_stamped(vector_stamped):
    """Convert a VectorStamped to a PointStamped

    :param vector_stamped the VectorStamped to be converted
    :return: PointStamped"""
    ps = gm.PointStamped()
    ps.header.frame_id = vector_stamped.frame_id
    ps.point = gm.Point(vector_stamped.vector.x(),
                        vector_stamped.vector.y(),
                        vector_stamped.vector.z())
    return ps


if __name__ == "__main__":
    import doctest
    doctest.testmod()
