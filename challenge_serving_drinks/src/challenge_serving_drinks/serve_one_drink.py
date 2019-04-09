# ROS
import rospy
import smach

# Robot smach states
import robot_smach_states as states
from robot_skills.robot import Robot

# Serving drinks
from .get_order import GetOrder


class ServeOneDrink(smach.StateMachine):
    """ Serves on drink to an operator

    """
    def __init__(self, robot):
        # type: (Robot) -> str
        """ Initialization method

        :param robot: robot api object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        person_designator = None  # ToDo: fill!
        drink_designator = None  # ToDo: fill!

        with self:

            smach.StateMachine.add(
                "GET_ORDER",
                GetOrder(robot, person_designator, drink_designator),
                transitions={"succeeded": "succeeded",
                             "failed": "failed"}
            )

            # Move to bar (if not in inspect state)

            # Inspect bar

            # Grasp drink

            # Move to room

            # Look around and identify the person who ordered the drink

            # Move to this person

            # Hand over the drink
