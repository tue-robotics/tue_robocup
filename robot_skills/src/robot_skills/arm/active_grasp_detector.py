from __future__ import absolute_import
import rospy
from sensor_msgs.msg import JointState

from robot_skills.robot_part import RobotPart


class ActiveGraspDetector(RobotPart):

    def __init__(self, robot_name, tf_buffer, wrench_topic):
    """
    Class for detecting whether or not the robot is holding something

    :param robot_name: Name of the robot
    :param tf_buffer: tf2_ros.Buffer for use in RobotPart
    :param wrench_topic: Topic to use for measurement
    """
        
        super(ActiveGraspDetector, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self._topic = wrench_topic

        self.wrench_sub = self.create_subscriber(self._topic, JointState, self._joint_callback, queue_size=1)

        self.threshold_difference = 0.075 # Difference between base and final position
        self.minimum_position = None # To be determined

        self.current_position = None
        self.position_difference = None


    def _joint_callback(self, msg):
        """
        Will be executed every time a new message is received.
        :param msg: input joint message
        """
        if msg.name == 'hand_motor_joint':
            self.current_position = msg.position


    def detect(self):
        """
        If called returns it the robot is holding something or not

        :return: True if the robot is holding something
        :return: False if it is not holding something
        """

        if self.current_position < minimum_position: # Hand not open enough to hold something
            return False

