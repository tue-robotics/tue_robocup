# ROS
import rospy
from std_msgs.msg import Bool

# TU/e Robotics
from robot_part import RobotPart


class EButton(RobotPart):
    """
    Interface to amigo emergency switch. Listens to /emergency_switch topic
    """

    def __init__(self, robot_name, tf_listener):
        super(EButton, self).__init__(robot_name=robot_name, tf_listener=tf_listener)

        rospy.logdebug("Initializing ebutton listener")
        self._ebuttonstatus = True
        self._topic = self.create_subscriber("/{}/emergency_switch".format(robot_name), Bool, self._listen)

    def close(self):
        pass

    def _listen(self, s):
        """
        Callback methods that listens to /emergency_switch
        """
        self._ebuttonstatus = s.data

    def read_ebutton(self):
        return self._ebuttonstatus
