# ROS
import rospy

# TU/e Robotics
from robot_skills.robot_part import RobotPart


class Ears(RobotPart):
    """
    Interface to amigo Ears.
    Works with dragonfly speech recognition (as of december 2014)

    Function listen explained on wiki: http://servicerobot.cstwiki.wtb.tue.nl/index.php?title=Using_the_dragonfly_speech_recognition
    """
    def __init__(self, robot_name, tf_listener, pre_hook=None, post_hook=None):
        super(Ears, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._pre_hook = pre_hook
        self._post_hook = post_hook

    # Function listens explained on wiki: http://servicerobot.cstwiki.wtb.tue.nl/index.php?title=Using_the_dragonfly_
    # speech_recognition
    def recognize(self, spec, choices={}, time_out=rospy.Duration(10)):

        if hasattr(self._pre_hook, '__call__'):
            self._pre_hook()

        answer = self._hmi.old_query(spec, choices, timeout=time_out.to_sec())  # self._hmi needs to be set in robot

        if hasattr(self._post_hook, '__call__'):
            self._post_hook()

        return answer
