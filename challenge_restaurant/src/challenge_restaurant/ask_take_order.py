import smach
from hmi import TimeoutException


class AskTakeTheOrder(smach.State):
    """ Wait for the waiving person """

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
