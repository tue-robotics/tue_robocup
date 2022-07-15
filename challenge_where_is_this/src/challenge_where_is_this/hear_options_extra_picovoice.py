import os

import rospy
from actionlib import SimpleActionClient
from picovoice_msgs.msg import GetIntentAction, GetIntentGoal

import robot_smach_states.util.designators as ds
import smach
from hmi import HMIResult
from robot_skills import get_robot
from robot_smach_states.util.designators import VariableDesignator


class HearOptionsExtraPicovoice(smach.State):
    def __init__(self, robot, context, speech_result_designator, timeout=10.0, look_at_standing_person=True):
        smach.State.__init__(self, outcomes=["heard", "no_result"])

        self.robot = robot
        self._client = SimpleActionClient("/get_intent", GetIntentAction)

        ds.check_resolve_type(speech_result_designator, HMIResult)
        ds.is_writeable(speech_result_designator)

        self.context = context
        self.speech_result_designator = speech_result_designator
        self.timeout = timeout
        self.look_at_standing_person = look_at_standing_person

    def _speech(self):
        rospy.loginfo("HearOptionsExtraPicovoice: Waiting for speech result")
        if self._client.send_goal_and_wait(
            GetIntentGoal(context_url=self.context, require_endpoint=True),
            preempt_timeout=rospy.Duration.from_sec(self.timeout),
            execute_timeout=rospy.Duration.from_sec(self.timeout),
        ):
            rospy.loginfo("Getting result from picovoice")
            result = self._client.get_result()
            if result is None:
                rospy.logerr("Picovoice result None")
                return False
            if not result.is_understood:
                rospy.logwarn("Not understood")
                return False

            self.speech_result_designator.write(
                HMIResult(sentence="", semantics={slot.key: slot.value for slot in result.slots})
            )

            return True
        return False

    def execute(self, userdata=None):
        if self.look_at_standing_person:
            self.robot.head.look_at_standing_person()

        outcome = "heard" if self._speech() else "no_result"

        if self.look_at_standing_person:
            self.robot.head.cancel_goal()

        return outcome


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    hero = get_robot("hero")
    designator = VariableDesignator(resolve_type=HMIResult)
    HearOptionsExtraPicovoice(hero, "restaurant", designator.writeable).execute()
    print(designator.resolve())
