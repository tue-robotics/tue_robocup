import actionlib
import rospy
from picovoice_msgs.msg import GetIntentAction, GetIntentGoal
from robot_skills.simulation import is_sim_mode

import smach
from hmi import TimeoutException, HMIResult


class GetIntent:
    def __init__(self):
        self._client = actionlib.SimpleActionClient("/get_intent", GetIntentAction)

    def query(self):
        if self._client.send_goal_and_wait(GetIntentGoal(
            context_url='restaurant_take_the_order',
            require_endpoint=True
        ), preempt_timeout=rospy.Duration(10), execute_timeout=rospy.Duration(10)):
            result = self._client.get_result()
            if result is None:
                rospy.logerr("Picovoice result None")
                raise TimeoutException("Picovoice result None")
            if not result.is_understood:
                rospy.logwarn("Not understood")
                raise TimeoutException("Not understood")
            return HMIResult(sentence="", semantics={slot.key: slot.value for slot in result.slots})
        else:
            rospy.logerr("Picovoice failed")
            raise TimeoutException("Picovoice failed")


class AskTakeTheOrder(smach.State):
    """ Wait for the waiving person """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['yes', 'wait', 'timeout'])
        self._get_intent = GetIntent()

        self.robot = robot

    def execute(self, userdata=None):
        cgrammar = """
        C['yes'] -> {robot} take the order
        C['wait'] -> {robot} wait
        """.format(robot=self.robot.robot_name)
        for i in range(3):
            try:
                if is_sim_mode():
                    rospy.loginfo("In Sim Mode we don't use picovoice")
                    speech_result = self.robot.hmi.query(description="Should I get the order?",
                                                         grammar=cgrammar, target="C")
                    return speech_result.semantics
                else:
                    speech_result = self._get_intent.query()
                    if "take the order" in str(speech_result.semantics):
                        return 'yes'
                    return 'wait'
            except TimeoutException:
                pass
        return 'timeout'
