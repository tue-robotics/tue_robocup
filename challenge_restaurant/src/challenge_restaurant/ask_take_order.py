import smach
from hmi.client import TimeoutException
from hmi.common import HMIResult

import robot_smach_states.util.designators as ds
from robot_smach_states.human_interaction.human_interaction import HearOptionsExtraPicoVoice


class AskTakeTheOrder(smach.State):
    """
    Ask the operator if the robot should take the order.
    """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['yes', 'wait', 'timeout'])

        self.robot = robot

    def execute(self, userdata=None):
        cgrammar = """
        C['yes'] -> {robot} take the order
        C['wait'] -> {robot} wait
        """.format(robot=self.robot.robot_name)
        for i in range(3):
            try:
                speech_result = self.robot.hmi.query(description="Should I get the order?",
                                                     grammar=cgrammar, target="C")
                return speech_result.semantics
            except TimeoutException:
                pass
        return 'timeout'


class AskTakeTheOrderPicoVoice(HearOptionsExtraPicoVoice):
    def __init__(self, robot, timeout=20, look_at_standing_person=True):
        self.speech_result_designator = ds.VariableDesignator(resolve_type=HMIResult)
        super().__init__(
            robot,
            "yesOrNo",
            self.speech_result_designator.writeable,
            timeout=timeout,
            look_at_standing_person=look_at_standing_person,
        )
        smach.State.__init__(self, outcomes=["yes", "wait", "no_result"])

    def execute(self, userdata=None):
        result = super().execute(userdata)
        if result == "no_result":
            return "no_result"

        hmi_result = self.speech_result_designator.resolve()
        if not hmi_result or not hmi_result.semantics:
            return "no_result"

        if "yes" in hmi_result.semantics:
            return "yes"
        elif "no" in hmi_result.semantics:
            return "wait"
        else:
            return "no_result"
