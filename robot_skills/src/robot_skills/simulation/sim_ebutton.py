# TU/e Robotics
from robot_skills.robot_part import RobotPart


class SimEButton(RobotPart):
    # noinspection PyUnusedLocal
    def __init__(self, robot_name, tf_listener, topic=None):
        """
        Interface to robot emergency switch. Provides the same interface as the real e-button

        :param robot_name: (str) string indicates the robot name
        :param tf_listener: (tf.Listener) (default argument for robot parts)
        :param topic: (str) fully qualified topic (optional).
            If not provided, "/<robot_name>/emergency_switch" will be used.
        """
        super(SimEButton, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        pass

    def close(self):
        pass

    @staticmethod
    def read_ebutton():
        """
        Always returns False as if the e-button is not pressed

        :return: (bool)
        """
        return False
