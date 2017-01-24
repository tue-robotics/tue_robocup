# ROS
import geometry_msgs.msg as gm
import PyKDL as kdl


def point_msg_to_kdl_vector(point):
    """ Converts a geometry_msgs.msg.Point to a kdl Vector

    :param point: geometry_msgs.msg.Point
    :return: kdl Vector
    """
    return kdl.Vector(point.x, point.y, point.z)


def quaternion_msg_to_kdl_rotation(quat):
    """ Converts a geometry_msgs.msg.Quaternion to a kdl Rotation

    :param quat: geometry_msgs.msg.Quaternion
    :return: kdl Rotation
    """
    return kdl.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)


def pose_msg_to_kdl_frame(pose):
    """ Converts a geometry_msg.msg.Pose to a kdl Frame

    :param pose: geometry_msg.msg.Pose
    :return: kdl Frame
    """
    rot = quaternion_msg_to_kdl_rotation(pose.orientation)
    trans = point_msg_to_kdl_vector(pose.position)
    return kdl.Frame(rot, trans)


def kdl_vector_to_point_msg(vector):
    """ Converts a kdl Vector to a geometry_msgs.msg.Point

    :param vector: kdl Vector
    :return: geometry_msgs.msg.Point
    """
    return gm.Point(vector.x(), vector.y(), vector.z())


def kdl_rotation_to_quaternion_msg(rotation):
    """ Converts a kdl Rotation to a geometry_msgs.msg.Quaternion

    :param rotation: kdl Rotation
    :return: geometry_msgs.msg.Quaternion
    """
    return gm.Quaternion(*rotation.GetQuaternion())


def kdl_frame_to_pose_msg(frame):
    """ Converts a kdl Frame to a geometry_msgs.msg.Pose

    :param frame: kdl Frame
    :return: geometry_msgs.msg.Pose
    """
    return gm.Pose(kdl_vector_to_point_msg(frame.p), kdl_rotation_to_quaternion_msg(frame.M))
