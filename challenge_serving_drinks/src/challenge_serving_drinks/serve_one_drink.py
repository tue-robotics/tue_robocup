# ROS
import rospy
import smach

# Robot smach states
import robot_smach_states as states
from robot_skills.robot import Robot


class ServeOneDrink(smach.StateMachine):
    """ This is my example state machine

    """
    def __init__(self, robot):
        # type: (Robot) -> str
        """ Initialization method

        :param robot: robot api object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])
        rospy.logwarn("Constructing ServeOneDrink state")

        with self:
            smach.StateMachine.add("SAY_BOO", states.Say(robot, "Boo"), transitions={"spoken": "succeeded"})
