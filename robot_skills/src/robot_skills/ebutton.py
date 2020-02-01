# ROS
import rospy
from std_msgs.msg import Bool

# TU/e Robotics
from robot_skills.robot_part import RobotPart


class EButton(RobotPart):
    def __init__(self, robot_name, tf_listener, topic=None):
        """
        Interface to amigo emergency switch. Listens to /emergency_switch topic

        :param robot_name: (str) string indicates the robot name
        :param tf_listener: (tf.Listener) (default argument for robot parts)
        :param topic: (str) fully qualified topic (optional).
            If not provided, "/<robot_name>/emergency_switch" will be used.
        """
        super(EButton, self).__init__(robot_name=robot_name, tf_listener=tf_listener)

        rospy.logdebug("Initializing ebutton listener")
        self._ebuttonstatus = True
        topic = topic or "/{}/emergency_switch".format(robot_name)
        self._topic = self.create_subscriber(topic, Bool, self._listen)

    def close(self):
        pass

    def _listen(self, s):
        """
        Callback methods that listens to /emergency_switch
        """
        self._ebuttonstatus = s.data

    def read_ebutton(self):
        return self._ebuttonstatus
