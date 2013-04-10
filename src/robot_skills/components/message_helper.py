import roslib; roslib.load_manifest('tue_execution_pack')
import rospy
import geometry_msgs.msg
import amigo_msgs.msg
import visualization_msgs.msg
import tf
from amigo_head_ref.msg import HeadRefAction, HeadRefGoal

def point(x, y, z):
    """
    Helper method for creating a geometry_msgs Point
    """
    goal = geometry_msgs.msg.Point(x, y, z)
    return goal

def pose2D(x, y, theta):
    pose =  geometry_msgs.msg.Pose2D(x, y, theta)
    return pose

def point_stamped(x, y, z, frame_id='/map'):
    """
    Helper method for creating a geometry_msgs Point
    """
    ps = geometry_msgs.msg.PointStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp = rospy.Time.now()
    ps.point.x = x
    ps.point.y = y
    ps.point.z = z
    return ps

def stamp_point(point, frame_id='/map'):
    """
    Stamp a point with the current time, returns PointStamped
    """
    ps = geometry_msgs.msg.PointStamped()
    ps.point = point
    ps.header.frame_id = frame_id
    ps.header.stamp = rospy.Time.now()
    
    return ps
    
def quaternion( x, y, z, w):
    """
    Create a quaternion from parameters
    """
    orientation_goal = geometry_msgs.msg.Quaternion()
    orientation_goal.x = x
    orientation_goal.y = y
    orientation_goal.z = z
    orientation_goal.w = w


def quaternion_from_axis_angle(axis, angle):
    """
    Create quaternion from axis angle, uses a tf transform
    """
    q = tf.transformations.quaternion_about_axis(angle, axis)
    return quaternion(q[0], q[1], q[2], q[3])
    
def quaternion_from_angle_z(angle):
    return quaternion_from_axis_angle((0,0,1), angle)


def visualization_marker(target_location, id, r, g, b):
    """
    Helper method for creating a visualization marker
    """
    location_marker = visualization_msgs.msg.Marker()
    location_marker.header.frame_id = "/map"
    location_marker.header.stamp = rospy.Time.now()
    location_marker.ns = "executioner"
    location_marker.id = id
    location_marker.type = 2
    location_marker.action = 0
    location_marker.pose.position.x = target_location.x
    location_marker.pose.position.y = target_location.y
    location_marker.pose.position.z = target_location.z
    location_marker.pose.orientation.x = 0.0
    location_marker.pose.orientation.y = 0.0
    location_marker.pose.orientation.z = 0.0
    location_marker.pose.orientation.w = 0.0
    location_marker.scale.x = 0.1
    location_marker.scale.y = 0.1
    location_marker.scale.z = 0.1
    location_marker.color.a = 1.0
    location_marker.color.r = r
    location_marker.color.g = g
    location_marker.color.b = b
    return location_marker

def head_ref_action(position, frame_id='/map'):
    """
    Helper method for creating a head ref action
    """
    head_goal = HeadRefGoal()#geometry_msgs.msg.PointStamped()
    head_goal.target_point.header.stamp = rospy.get_rostime();
    head_goal.target_point.header.frame_id = frame_id
    head_goal.target_point.point = position