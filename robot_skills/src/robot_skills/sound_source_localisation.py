# System
import math

# ROS
from geometry_msgs.msg import PoseStamped
import rospy
import tf

# TU/e Robotics
from robot_skills.robot_part import RobotPart


class SSL(RobotPart):
    """
    Sound source localization interface for the robot

     We only expose the get_last_yaw method here that transforms the incoming PoseStamped from the back-end to a
     yaw in the frame of the incoming message. The user of this method should take into account that the yaw is
     represented in this frame.

    :param topic: Incoming PoseStamped
    """
    def __init__(self, robot_name, tf_listener):
        """
        constructor

        :param robot_name: robot_name
        :param tf_listener: tf_server.TFClient()
        """
        super(SSL, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._sub = self.create_subscriber('/{}/ssl/direction_of_arrival'.format(self.robot_name), PoseStamped, self._callback, queue_size=1)
        self._last_msg = None
        self._last_received_time = rospy.Time(0)

    @property
    def operational(self):
        return self._sub.get_num_connections() >= 1

    def _callback(self, msg):
        """
        callback

        :param msg: incoming msg, PoseStamped
        :return: no return
        """
        self._last_msg = msg
        self._last_received_time = rospy.Time.now()

    def get_last_yaw(self, max_age_seconds=2):
        """
        Get last yaw rotation of the sound source localisation in the frame of the device

        :param max_age_seconds: maximum age of the last received direction of the sound, when older None is returned
        :type max_age_seconds: int
        :return: yaw
        """
        if not self._last_msg or rospy.Time.now() - self._last_received_time > rospy.Duration(max_age_seconds):
            return None
        q = self._last_msg.pose.orientation
        return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2] + math.pi
