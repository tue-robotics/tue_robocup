import rospy
import PyKDL as kdl

from robot_skills.robot_part import RobotPart
from tue_manipulation_msgs.msg import GripperCommandGoal, GripperCommandAction

class HandoverDetector(RobotPart):
    def __init__(self, robot_name, tf_listener, side):
        super(Gripper, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self.robot_name = robot_name
        self.side = side

    def handover_to_human(self, timeout=10):
        """
        Handover an item from the gripper to a human.

        Feels if user slightly pulls or pushes (the item in) the arm. On timeout, it will return False.
        :param timeout: timeout in seconds
        :return: True or False
        """
        pub = rospy.Publisher('/' + self.robot_name + '/handoverdetector_' + self.side + '/toggle_robot2human',
                              std_msgs.msg.Bool, queue_size=1, latch=True)
        pub.publish(std_msgs.msg.Bool(True))

        try:
            rospy.wait_for_message('/' + self.robot_name + '/handoverdetector_' + self.side + '/result', std_msgs.msg.Bool,
                                   timeout)
            # print('/'+self.robot_name+'/handoverdetector_'+self.side+'/result')
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
        pub = rospy.Publisher('/' + self.robot_name + '/handoverdetector_' + self.side + '/toggle_human2robot',
                              std_msgs.msg.Bool, queue_size=1, latch=True)
        pub.publish(std_msgs.msg.Bool(True))

        try:
            rospy.wait_for_message('/' + self.robot_name + '/handoverdetector_' + self.side + '/result', std_msgs.msg.Bool,
                                   timeout)
            # print('/'+self.robot_name+'/handoverdetector_'+self.side+'/result')
            return True
        except rospy.ROSException as e:
            rospy.logerr(e)
            return False
