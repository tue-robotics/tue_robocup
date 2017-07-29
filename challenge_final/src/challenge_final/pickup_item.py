import robot_smach_states as states
import smach
from challenge_storing_groceries.manipulate_machine import DefaultGrabDesignator, GrabSingleItem


class PickupItem(smach.StateMachine):
    """ State machine to pick up an item. Inspects the entity that has been pointed at and grabs the item on top
     of it
    """

    def __init__(self, robot, furniture_designator):
        """ Constructor
        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add("INSPECT_FURNITURE",
                                   states.Inspect(robot, furniture_designator),
                                   transitions={"done": "PICKUP_FROM_FURNITURE",
                                                "failed": "PICKUP_FROM_FURNITURE"})

            # ToDo: move GrabSingleItem to robot smach states
            smach.StateMachine.add("PICKUP_FROM_FURNITURE",  # Pickup the thing standing on the furniture
                                   GrabSingleItem(robot, DefaultGrabDesignator(robot,
                                                                               furniture_designator, "on_top_of")),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "failed"})
