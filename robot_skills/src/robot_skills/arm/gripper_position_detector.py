from __future__ import absolute_import
import rospy
from sensor_msgs.msg import JointState

from robot_skills.robot_part import RobotPart


class GripperPositionDetector(RobotPart):
    def __init__(self, robot_name, tf_buffer, joint_topic):
        """
        Class for detecting whether the robot is holding something

        :param robot_name: Name of the robot
        :param tf_buffer: tf2_ros.Buffer for use in RobotPart
        :param joint_topic: Topic to use for measurement
        """
        super(GripperPositionDetector, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self._topic = joint_topic
        self.timeout = rospy.Duration(2.0)  # seconds
        self.wrench_sub = self.create_subscriber(self._topic, JointState, self._joint_callback, queue_size=1)
        self.current_position = None
        self.store_position = False  # Flag to store the position
        self.start_time = None

    def _joint_callback(self, msg):
        """
        Will be executed every time a new message is received.

        :param msg: input joint message
        """
        if self.store_position:
            self.current_position = msg.position[msg.name.index("hand_motor_joint")]
            self.store_position = False

    def detect(self):
        """
        Returns the position of the gripper if possible

        :return: The current position as a double, a None value is returned if position can't be retrieved
        """
        # Reset starting time
        self.start_time = rospy.Time.now()
        self.current_position = None  # Reset the value
        self.store_position = True

        while self.current_position is None:  # Wait until value is stored
            if rospy.Time.now() > (self.start_time + self.timeout):  # If timeout
                self.store_position = False  # stop waiting for messages
                rospy.logwarn('Stopped waiting for messages: Position of gripper could not be retrieved')
                break

        return self.current_position
