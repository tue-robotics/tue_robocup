import smach
import rospy

import robot_smach_states.util.designators as ds

__all__ = ["DummyState"]


class DummyState(smach.State):
    """
    DummyState which returns the input as the outcome
    """

    def __init__(self, result_designator, *args, **kwargs):
        """
        :param result_designator: (VariableDesignator) or (str) The result to
            be returned
        """
        super(DummyState,
              self).__init__(outcomes=["succeeded", "failed", "aborted"])
        ds.check_type(result_designator, str)
        self.result = result_designator.resolve() if hasattr(
            result_designator, "resolve") else result_designator

    def execute(self, userdata=None):
        if self.result in self._outcomes:
            return self.result
        else:
            rospy.loginfo("Outcome '{}' is invalid. Aborting.".format(self.result))
            return "aborted"


if __name__ == "__main__":
    rospy.init_node('dummy_state_machine')

    sm = smach.StateMachine(outcomes=['outcome1', 'outcome2', 'outcome3'])

    with sm:
        sm.add('FOO',
               DummyState("succeeded"),
               transitions={
                   'succeeded': 'outcome1',
                   'failed': 'outcome2',
                   'aborted': 'outcome3'
               })

    sm.execute()
