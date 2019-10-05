import smach
import rospy

from robot_smach_states.util.designators.checks import check_type


class DummyState(smach.State):
    """
    DummyState which returns the input as the outcome
    """

    def __init__(self, outcomes, result):
        """
        :param result: (VDesignator) or (str) The result to be returned
        :param outcomes: str or [str] Possible outcomes
        """
        check_type(outcomes, str, [str])
        outcomes = list(outcomes)
        assert len(outcomes) >= 1, "Minimal one outcome should be specified"
        outcomes.append("invalid_outcome")
        smach.State.__init__(self, outcomes=outcomes)
        check_type(result, str)
        self.result = result

    def execute(self, userdata=None):
        result = self.result.resolve() if hasattr(self.result, "resolve") else self.result
        if result in self._outcomes:
            return result
        else:
            rospy.loginfo("Outcome '{}' is invalid. Aborting.".format(result))
            return "invalid_outcome"


if __name__ == "__main__":
    rospy.init_node('dummy_state_machine')

    sm = smach.StateMachine(outcomes=['outcome1', 'outcome2', 'outcome3', 'invalid_outcome'])

    with sm:
        sm.add('FOO',
               DummyState(outcomes=['succeeded', 'failed', 'aborted'], result="succeeded"),
               transitions={'succeeded': 'outcome1',
                            'failed': 'outcome2',
                            'aborted': 'outcome3',
                            'invalid_outcome': 'invalid_outcome'})

    sm.execute()
