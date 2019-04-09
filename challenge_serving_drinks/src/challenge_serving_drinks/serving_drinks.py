# ROS
import smach

# Robot smach states
import robot_smach_states as states


class ServingDrinks(smach.StateMachine):
    """ This is my example state machine

    """
    def __init__(self, robot):
        """ Initialization method

        :param robot: robot api object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        with self:
            smach.StateMachine.add("HELLO_WORLD", states.Say(robot, "Hello world"), transitions={"spoken": "succeeded"})
