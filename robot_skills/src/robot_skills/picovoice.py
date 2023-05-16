from typing import List, Optional
import rospy

from hmi.client import TimeoutException
from hmi.common import HMIResult
from picovoice_msgs.msg import GetIntentAction, GetIntentGoal, GetIntentResult
from robot_skills.robot_part import RobotPart


class PicoVoice(RobotPart):
    def __init__(self, robot_name: str, tf_buffer):
        super().__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self._action_client = self.create_simple_action_client(f"/{robot_name}/get_intent", GetIntentAction)

    def get_intent(
        self,
        context_url: str,
        intents: Optional[List[str]] = None,
        require_endpoint: bool = True,
        timeout: float = 10.0,
    ) -> HMIResult:
        """
        get_intent

        :param context_url: PicoVoice context
        :param intents: Only accept these intents, when none provided, no filtering is applied
        :param require_endpoint: endpoint is required or not
        :param timeout: timeout
        :return: intent
        :raises TimeoutException: In case of timeout
        """
        if intents is None:
            intents = []
        rospy.loginfo(f"PV get_intent: {context_url=}, {intents=}, {require_endpoint=}, {timeout=}")
        timeout = rospy.Duration.from_sec(timeout)
        goal = GetIntentGoal(context_url=context_url, intents=intents, require_endpoint=require_endpoint)
        rospy.logdebug(f"PicoVoice.get_intent: {goal=}")
        if not self._action_client.send_goal_and_wait(goal, execute_timeout=timeout, preempt_timeout=timeout):
            rospy.logerr("PicoVoice.get_intent: failed")
            raise TimeoutException("PicoVoice.get_intent: failed")

        result = self._action_client.get_result()  # type: GetIntentResult
        rospy.logdebug(f"PicoVoice.get_intent: {result=}")

        if result is None:
            rospy.logerr("PicoVoice.get_intent: result: None")
            raise TimeoutException("PicoVoice.get_intent: result: None")
        if not result.is_understood:
            rospy.logwarn("PicoVoice.get_intent: Not understood")
            raise TimeoutException("PicoVoice.get_intent: Not understood")
        hmi_result = HMIResult(sentence="", semantics={slot.key: slot.value.replace(" ", "_") for slot in result.slots})
        rospy.loginfo(f"PicoVoice.get_intent: result={hmi_result}")
        return hmi_result
