#! /usr/bin/env python
# ROS
import smach
import rospy

# TU/e Robotics
from robot_smach_states.util.designators import Designator


class State(smach.State):
    def __init__(self, *args, **kwargs):
        smach.State.__init__(self, outcomes=kwargs['outcomes'])
        self.__dict__['init_arguments'] = args
        print "Using State in {} is deprecated, use smach.State instead and implement execute(self, userdata) " \
              "instead of run(self, ...)".format(type(self))

    def execute(self, userdata=None):
        resolved_arguments = {key: (value.resolve() if hasattr(value, "resolve") else value) for key, value
            in self.__dict__['init_arguments'][0].iteritems()}
        del resolved_arguments['self']

        if not all(resolved_arguments):
            # Make a list of all keys that resolve to None (because not None == True)
            unresolved_arguments = filter(lambda x: not x, resolved_arguments)
            rospy.logerr("Values for {0} could not be resolved".format(unresolved_arguments))

        return self.run(**resolved_arguments)


class TestState(State):
    """
    >>> teststate = TestState("Yes", "this", "works")
    Using State in <class 'robot_smach_states.state.TestState'> is deprecated, use smach.State instead and implement
    execute(self, userdata) instead of run(self, ...)
    >>> teststate.execute()
    Yes this works
    'yes'

    >>> teststate2 = TestState(Designator("Also"), "works", Designator("with designators"))
    Using State in <class 'robot_smach_states.state.TestState'> is deprecated, use smach.State instead and implement
    execute(self, userdata) instead of run(self, ...)
    >>> teststate2.execute()
    Also works with designators
    'yes'"""
    def __init__(self, robot, sentence, blaat):
        State.__init__(self, locals(), outcomes=['yes', 'no'])

    def run(self, robot, sentence, blaat):
        print robot, sentence, blaat
        return "yes"


class Test(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        with self:
            smach.StateMachine.add('TEST_STATE1',
                                   TestState("Yes", "this", "works"),
                                   transitions={'yes': 'TEST_STATE2', 'no': 'failed'})

            smach.StateMachine.add('TEST_STATE2',
                                   TestState(Designator("Also"), "works", Designator("with designators")),
                                   transitions={'yes': 'succeeded', 'no': 'failed'})


if __name__ == "__main__":
    import doctest
    doctest.testmod()

    sm = Test()
    sm.execute()
