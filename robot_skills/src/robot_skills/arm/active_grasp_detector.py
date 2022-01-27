from __future__ import absolute_import

import rospy
 
from sensor_msgs.msg import JointState


# TU/e Robotics
from robot_skills.robot_part import RobotPart


class ActiveGraspDetector(RobotPart):
	"""
	Class for detecting whether or not the robot is holding something using ""

	:param robot_name: Name of the robot
	:param tf_buffer: tf2_ros.Buffer for use in RobotPart
	:param wrench_topic: Topic to use for measurement
	"""

	def __init__(self, robot_name, tf_buffer, wrench_topic):
	        super(GraspDetector, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
	        self._topic = wrench_topic
	        self.start_time = None
	        self.threshold_difference = 0.075 # Values may vary on standby

	        # self.measuring_ideal ?
	  	self.measuring_max = rospy.Duration(10.0) # seconds

	        self.sensor_list = []
	        self.record = False



	def start_recording(self):
	    """"
	    Function that allows the start of recording the sensor data.
	    :return: True in order to start the recording
	    """
	    rospy.loginfo('Start recording')

	    # Reset starting time
	    self.start_time = rospy.Time.now()

	    # Resets the sensor list
	    self.sensor_list = []

	    self.record = True  # sets flag to start recording


	def detect(self):
	    """
	    If called returns it the robot is holding something or not

	    :return: True if the robot is holding something
	    :return: False if it is not holding something
	    """
