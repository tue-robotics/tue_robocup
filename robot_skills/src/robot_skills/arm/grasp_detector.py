from __future__ import absolute_import

import rospy
from geometry_msgs.msg import WrenchStamped

from statistics import mean

# TU/e Robotics
from robot_skills.robot_part import RobotPart


class GraspDetector(RobotPart):
    """
    Class for detecting whether or not the robot is holding something

    :param robot_name: Which robot is this part of?
    :param tf_buffer: tf2_ros.Buffer for use in RobotPart
    :param wrench_topic: Topic to use for WrenchStamped measurement
    """
    def __init__(self, robot_name, tf_buffer, wrench_topic):
        super(GraspDetector, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self._topic = wrench_topic
        self.latest_msg = None
        self.msg_list = []
        self.threshold_torque_y = -0.45
        self.measuring_for = 15
        self.start_time = rospy.Time.now()
        self.wrench_sub = self.create_subscriber(self._topic, WrenchStamped, self._wrench_callback, queue_size=1)

    def _wrench_callback(self, msg):
        """
        callback method that will be executed every time a new wrench message is received.
        :param msg: incoming wrench message
        """
        self.latest_msg = msg

        if not rospy.Time.now() > (self.start_time + rospy.Duration(self.measuring_for)):
            self.msg_list.append(msg)


    def detect(self):
        """
        Function call that can be made whenever we want to know whether we are holding something.
        :return: True if we are holding something
                 False if we are not.
        """
        if mean(self.msg_list) < self.threshold_torque_y:
            return True, "Grasp successful"
        else:
            return False, "Grasp failed"
