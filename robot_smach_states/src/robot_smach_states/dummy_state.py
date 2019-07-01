import smach
import robot_smach_states.util.designators as ds

__all__ = ["DummyState"]


class DummyState(smach.State):
    """
    DummyState to which returns the input outcome
    """
    def __init__(self, result_designator):
        """
        :param result_designator: (VariableDesignator) or (str) The result to
            be returned
        """
        super(DummyState, self).__init__(outcomes=["succeeded", "failed", "aborted"])
        ds.check_type(result_designator, "str")
        self.result = result_designator.resolve() if hasattr(result_designator,"resolve") else result_designator

    def execute(self, userdata=None):
        if self.result in self._outcomes:
            return self.result
        else:
            return "aborted"
