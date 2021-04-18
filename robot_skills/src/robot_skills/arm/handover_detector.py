from __future__ import print_function

import rospy
import std_msgs

from robot_skills.robot_part import RobotPart


class HandoverDetector(RobotPart):
    """
    Sensor functionality to detect when a handover is taking place
    """
    def __init__(self, robot_name, tf_listener, side):
        """
        constructor

        :param robot_name: robot_name
        :param tf_listener: tf_server.TFClient()
        :param side: string used to identify the sensor
        """
        super(HandoverDetector, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self.side = side

    def _generic_handover(self, direction, timeout=10):
        """
        Detect a handover an item to/from the gripper from/to a human.
        Feels if user slightly pulls or pushes (the item in) the arm. On timeout, it will return False.

        :param direction: direction of handover
        :type direction: str
        :param timeout: timeout in seconds
        :type timeout: float
        :return: Success
        :rtype: bool
        """
        pub = rospy.Publisher('/{}/handoverdetector_{}/toggle_{}'.format(self.robot_name, self.side, direction),
                              std_msgs.msg.Bool, queue_size=1, latch=True)
        pub.publish(True)

        try:
            rospy.wait_for_message('/{}/handoverdetector_{}/result'.format(self.robot_name, self.side),
                                   std_msgs.msg.Bool, timeout)
            return True
        except rospy.ROSException as e:
            rospy.logerr(e)
            return False

    def handover_to_human(self, timeout=10):
        """
        Handover an item from the gripper to a human.
        Feels if user slightly pushes/pulls an item in the gripper. On timeout, it will return False.

        :param timeout: timeout in seconds
        :type timeout: float
        :return: Success
        :rtype: bool
        """
        return self._generic_handover('robot2human', timeout)

    def handover_to_robot(self, timeout=10):
        """
        Handover an item from a human to the robot.
        Feels if user slightly pushes/pulls an item in the gripper. On timeout, it will return False.

        :param timeout: timeout in seconds
        :type timeout: float
        :return: Success
        :rtype: bool
        """
        return self._generic_handover('human2robot', timeout)
