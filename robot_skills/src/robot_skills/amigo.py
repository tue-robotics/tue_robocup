#! /usr/bin/env python

import robot
import rospy
import tf
import math

from geometry_msgs.msg import PoseStamped


class SSL(object):
    def __init__(self, topic):
        self._sub = rospy.Subscriber(topic, PoseStamped, self._callback, queue_size=1)
        self._last_msg = None
        self._last_received_time = rospy.Time(0)

    def _callback(self, msg):
        self._last_msg = msg
        self._last_received_time = rospy.Time.now()

    def get_last_yaw(self, max_age_seconds=2):
        if not self._last_msg or rospy.Time.now() - self._last_received_time > rospy.Duration(max_age_seconds):
            return None
        q = self._last_msg.pose.orientation
        return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2] + math.pi


class Amigo(robot.Robot):

    """docstring for Amigo"""
    def __init__(self, dontInclude=[], wait_services=False):
        super(Amigo, self).__init__(robot_name="amigo", wait_services=wait_services)
        self.ssl = SSL("/amigo/ssl/direction_of_arrival")
