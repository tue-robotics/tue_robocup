# ROS
import smach

# TU/e Robotics
import robot_smach_states as states


class ChallengeFinal(smach.StateMachine):
    """ State machine for the final challenge """
    def __init__(self, robot):
        """ Constructor
        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        with self:
            smach.StateMachine.add("SAY_WORKING",
                                   states.Say(robot, "Hey it's working",
                                              block=False),
                                   transitions={'spoken': 'Done'})
