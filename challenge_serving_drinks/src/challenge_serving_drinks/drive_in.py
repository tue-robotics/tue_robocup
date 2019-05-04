# ROS
import smach

# Robot smach states
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robocup_knowledge import knowledge_loader

# Serving drinks
from .sd_states import AskAvailability

# Knowledge
CHALLENGE_KNOWLEDGE = knowledge_loader.load_knowledge("challenge_serving_drinks")


class DriveIn(smach.StateMachine):
    """ Serves on drink to an operator

    """
    def __init__(self, robot):
        """ Initialization method

        :param robot: robot api object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        bar_designator = ds.EdEntityDesignator(robot=robot, id=CHALLENGE_KNOWLEDGE.bar_id, name='bar_des')
        room_designator = ds.EdEntityDesignator(robot=robot, id=CHALLENGE_KNOWLEDGE.room_id, name='room_des')
        drink_str_des = ds.VariableDesignator(resolve_type=str, name='drink_str_des')
        unavailable_drink_des = ds.EdEntityDesignator(robot=robot, type_designator=drink_str_des, name='unav_drink_des')

        with self:
            # Initialize
            smach.StateMachine.add(
                'INITIALIZE',
                states.Initialize(robot),
                transitions={"initialized": "SET_INITIAL_POSE",
                             "abort": "aborted"}
            )

            smach.StateMachine.add(
                'SET_INITIAL_POSE',
                states.SetInitialPose(robot, CHALLENGE_KNOWLEDGE.starting_point),
                transitions={"done": "INSPECT_BAR",
                             "preempted": "aborted",
                             "error": "INSPECT_BAR"}
            )

            # Inspect bar
            smach.StateMachine.add(
                "INSPECT_BAR",
                states.Inspect(
                    robot=robot,
                    entityDes=bar_designator,
                    navigation_area="in_front_of"),
                transitions={"done": "NAVIGATE_TO_ROOM",  # ToDo: transition to "AVAILABLE_DRINKS"
                             "failed": "INSPECT_FALLBACK"}
            )

            # Inspect fallback - ask the bartender which drink is unavailable and store the unavailable drink
            smach.StateMachine.add(
                "INSPECT_FALLBACK",
                AskAvailability(
                    robot=robot,
                    unavailable_drink_designator=unavailable_drink_des),
                transitions={"succeeded": "NAVIGATE_TO_ROOM",
                             "failed": "NAVIGATE_TO_ROOM"},
            )

            # Store the available drinks  # ToDo: implement storing the available drinks
            # smach.StateMachine.add(
            #     "AVAILABLE_DRINKS",
            #     StoreDrinks(
            #         robot=robot,
            #         available_drinks_designator=available_drinks_designator.writeable),
            #     transitions={"succeeded": "NAVIGATE_TO_ROOM"},
            # )

            smach.StateMachine.add(
                "NAVIGATE_TO_ROOM",
                states.NavigateToRoom(
                    robot=robot,
                    entity_designator_room=room_designator),
                transitions={"arrived": "succeeded",
                             "unreachable": "failed",
                             "goal_not_defined": "aborted"}
            )
