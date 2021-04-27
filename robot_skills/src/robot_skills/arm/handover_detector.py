from __future__ import print_function

import rospy
from std_msgs.msg import Bool

from robot_skills.robot_part import RobotPart


class HandoverDetector(RobotPart):
    """
    Sensor functionality to detect when a handover is taking place
    """
    def __init__(self, robot_name, tf_listener, name):
        """
        constructor

        :param robot_name: robot_name
        :param tf_listener: tf_server.TFClient()
        :param name: string used to identify the sensor
        """
        super(HandoverDetector, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self.name = name

    def handover_to_human(self, timeout=10):
        """
        Handover an item from the gripper to a human.

        Feels if user slightly pulls or pushes (the item in) the arm. On timeout, it will return False.
        :param timeout: timeout in seconds
        :return: True or False
        """
        pub = rospy.Publisher('/' + self.robot_name + '/' + self.name + '/toggle_robot2human', Bool, queue_size=1,
                              latch=True)
        pub.publish(Bool(True))

        try:
            rospy.wait_for_message('/' + self.robot_name + '/' + self.name + '/result', Bool, timeout)
            # print('/'+self.robot_name+'/'+self.name+'/result')
            return True
        except rospy.ROSException as e:
            rospy.logerr(e)
            return False

    def handover_to_robot(self, timeout=10):
        """
        Handover an item from a human to the robot.

        Feels if user slightly pushes an item in the gripper. On timeout, it will return False.
        :param timeout: timeout in seconds
        :return: True or False
        """
        pub = rospy.Publisher('/' + self.robot_name + '/' + self.name + '/toggle_human2robot', Bool, queue_size=1,
                              latch=True)
        pub.publish(Bool(True))

        try:
            rospy.wait_for_message('/' + self.robot_name + '/' + self.name + '/result', Bool, timeout)
            # print('/'+self.robot_name+'/'+self.name+'/result')
            return True
        except rospy.ROSException as e:
            rospy.logerr(e)
            return False
