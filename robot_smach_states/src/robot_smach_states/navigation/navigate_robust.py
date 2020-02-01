# System
import collections

# ROS
import smach

# Robot smach states
from robot_smach_states.human_interaction.human_interaction import Say
from robot_smach_states.utility import WaitTime
from .navigation import NavigateTo


SENTENCES = collections.deque([
    "I seem to have difficulties getting there",
    "Let's try a different route",
    "How can I get there?"
])


class NavigateRobust(smach.StateMachine):
    def __init__(self, robot, options, wait_time=3.0):
        """
        Class that hooks up a number of navigation options. If arrived, this state machine will exit with result
        arrived. If not unreachable or goal not defined, the robot will wait for <waittime> seconds and proceed to the
        next navigation option until either it succeeds or runs out of options.

        N.B.: on preemption, "unreachable" is returned (since the NavigateTo class has no "preempted" outcome).

        :param robot: (Robot) api object
        :param options: (list(NavigateTo)) list of different navigation options (instances of some derivative of the
            'NavigateTo' class, e.g., 'NavigateToSymbolic', 'NavigateToWaypoint', 'NavigateToRoom' etc.
        :param wait_time: (float) time the robot waits before trying different options.
        """
        assert all([isinstance(option, NavigateTo) for option in options]), "Not all options are 'NavigateTo' instances"
        smach.StateMachine.__init__(self, outcomes=["arrived", "unreachable", "goal_not_defined"])

        with self:

            # Add all options except the last.
            for idx, option in enumerate(options[:-1]):

                # Add an option
                smach.StateMachine.add("NAVIGATE{}".format(idx), option,
                                       transitions={
                                           "arrived": "arrived",
                                           "unreachable": "SAY{}".format(idx),
                                           "goal_not_defined": "SAY{}".format(idx),
                                       })

                # Have the robot say something
                smach.StateMachine.add("SAY{}".format(idx),
                                       Say(robot, sentence=SENTENCES[0], block=False),
                                       transitions={"spoken": "WAIT{}".format(idx)}
                )
                SENTENCES.rotate()

                # Have the robot wait for a while
                smach.StateMachine.add("WAIT{}".format(idx),
                                       WaitTime(robot, waittime=wait_time),
                                       transitions={"waited": "NAVIGATE{}".format(idx+1),
                                                    "preempted": "unreachable"})

            # Add the final option
            smach.StateMachine.add("NAVIGATE{}".format(idx + 1),
                                   options[-1],
                                   transitions={
                                       "arrived": "arrived",
                                       "unreachable": "unreachable",
                                       "goal_not_defined": "goal_not_defined",
                                   })
