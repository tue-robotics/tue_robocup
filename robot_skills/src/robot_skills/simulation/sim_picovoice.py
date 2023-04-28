from typing import List, Optional
from robot_skills.robot_part import RobotPart

from hmi.common import HMIResult


class SimPicoVoice(RobotPart):
    # noinspection PyUnusedLocal
    def __init__(self, robot_name, tf_buffer):
        """
        Interface to picovoice. Provides the same interface as the real picovoice interface,
        but always returns an empty HMIResult

        :param robot_name: (str) string indicates the robot name
        :param tf_buffer: (tf2_ros.Buffer) (default argument for robot parts)
        """
        super(SimPicoVoice, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        pass

    @staticmethod
    def get_intent(
        context_url: str,
        intents: Optional[List[str]] = None,
        require_endpoint: bool = True,
        timeout: float = 10.0,
    ) -> HMIResult:
        """
        Always returns an emtpy HMIResult
        """
        return HMIResult(sentence="", semantics={})
