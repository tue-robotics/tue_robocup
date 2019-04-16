# ROS
import rospy
import smach

# Robot smach states
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills.robot import Robot

# Serving drinks
from .get_order import GetOrder

# Knowledge  ToDo: move to robocup_knowledge package
BAR_ID = "bar"
ROOM_ID = "living_room"


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
        drink_designator = ds.EdEntityDesignator(robot=robot)
        bar_designator = ds.EdEntityDesignator(robot=robot, id=BAR_ID)
        arm_designator = ds.UnoccupiedArmDesignator(all_arms=robot.arms, preferred_arm=None)
        room_designator = ds.EdEntityDesignator(robot=robot, id=ROOM_ID)

        with self:

            smach.StateMachine.add(
                "GET_ORDER",
                GetOrder(robot=robot, person_designator=person_designator, drink_designator=drink_designator),
                transitions={"succeeded": "INSPECT_BAR",
                             "failed": "failed"}
            )

            # Inspect bar
            # ToDo: optimize so that we don't re-inspect the bar if we already know where the object is located
            smach.StateMachine.add(
                "INSPECT_BAR",
                states.Inspect(robot=robot, entityDes=bar_designator, navigation_area="in_front_of"),
                transitions={"done": "GRASP_DRINK",
                             "failed": "failed"}  # ToDo: fallback?
            )

            # Grasp drink
            smach.StateMachine.add(
                "GRASP_DRINK",
                states.Grab(robot=robot, item=drink_designator, arm=arm_designator),
                transitions={"done": "MOVE_TO_ROOM",
                             "failed": "failed"}  # ToDo: fallback?
            )

            # Move to room
            smach.StateMachine.add(
                "MOVE_TO_ROOM",
                states.NavigateToRoom(robot=robot, entity_designator_room=room_designator),
                transitions={"arrived": "HAND_OVER",
                             "unreachable": "failed",  # ToDo: fallback
                             "goal_not_defined": "failed"}
            )

            # Look around and identify the person who ordered the drink
            # ToDo: fill in

            # Move to this person
            # ToDo: fill in

            # Hand over the drink
            smach.StateMachine.add(
                "HAND_OVER",
                states.HandoverToHuman(robot=robot, arm_designator=arm_designator),
                transitions={"succeeded": "succeeded",
                             "failed": "failed"}  # ToDo: fallback
            )
