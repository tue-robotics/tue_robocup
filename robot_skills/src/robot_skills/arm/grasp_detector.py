from __future__ import absolute_import

import rospy
from geometry_msgs.msg import WrenchStamped

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
        self.start_time = None
        self.threshold_torque_y = -0.46  # Nm
        self.measuring_ideal = rospy.Duration(2.5)  # seconds
        self.measuring_max = rospy.Duration(10.0) # seconds
        self.wrench_sub = self.create_subscriber(self._topic, WrenchStamped, self._wrench_callback, queue_size=1)

        self.torque_list = []
        self.record = True

    def _wrench_callback(self, msg):
        """
        callback method that will be executed every time a new wrench message is received.
        :param msg: incoming wrench message
        """
        if self.record:
            self.torque_list.append(msg.wrench.torque.y)

        if rospy.Time.now() > (self.start_time + self.measuring_max):
            self.record = False  # stop recording messages
            rospy.logwarn('Stopped recording: too much time elapsed before reading torque_list')

    def start_recording(self):
        """"
        Function that allows the start of recording the torque data.
        :return: True in order to start the recording
        """
        rospy.loginfo('Start recording')

        # Reset starting time
        self.start_time = rospy.Time.now()

        # Reset the list of torque y
        self.torque_list = []

        self.record = True  # set flag to start recording


    def detect(self):
        """
        Function call that can be made whenever we want to know whether we are holding something.
        :return: True if we are holding something
                 False if we are not.
        """
        # Set the end time for the measurement
        end_time = self.start_time + self.measuring_ideal

        # Wait until measurement time has elapsed
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.record = False  # stop recording messages

        rospy.loginfo('Stopped recording')

        # Check whether the mean of the torque in the measured period is below the threshold
        if sum(self.torque_list)/len(self.torque_list) < self.threshold_torque_y:
            return True
        else:
            return False


