from __future__ import print_function

import rospy
import std_msgs
from robot_skills.robot_part import RobotPart


class HandoverDetector(RobotPart):
    """
    Sensor functionality to detect when a handover is taking place
    """
    def __init__(self, robot_name: str, tf_buffer: str, name: str) -> None:
        """
        constructor

        :param robot_name: robot_name
        :param tf_buffer: tf2_ros.Buffer
        :param name: string used to identify the sensor
        """
        super(HandoverDetector, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self.name = name

    def _generic_handover(self, direction: str, timeout: float = 10.0) -> bool:
        """
        Detect a handover an item to/from the gripper from/to a human.
        Feels if user slightly pulls or pushes (the item in) the arm. On timeout, it will return False.

        :param direction: direction of handover
        :param timeout: timeout in seconds
        :return: Success
        """
        pub = rospy.Publisher('/{}/handover_detector_{}/toggle_{}'.format(self.robot_name, self.name, direction),
                              std_msgs.msg.Bool, queue_size=1, latch=True)
        pub.publish(True)

        try:
            rospy.wait_for_message('/{}/handover_detector_{}/result'.format(self.robot_name, self.name),
                                   std_msgs.msg.Bool, timeout)
            return True
        except rospy.ROSException as e:
            rospy.logerr(e)
            return False

    def handover_to_human(self, timeout: float = 10.0) -> bool:
        """
        Handover an item from the gripper to a human.
        Feels if user slightly pushes/pulls an item in the gripper. On timeout, it will return False.

        :param timeout: timeout in seconds
        :return: Success
        """
        return self._generic_handover('robot2human', timeout)

    def handover_to_robot(self, timeout: float = 10.0) -> bool:
        """
        Handover an item from a human to the robot.
        Feels if user slightly pushes/pulls an item in the gripper. On timeout, it will return False.

        :param timeout: timeout in seconds
        :return: Success
        """
        return self._generic_handover('human2robot', timeout)

