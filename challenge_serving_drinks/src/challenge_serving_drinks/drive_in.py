# ROS
import smach

# Robot smach states
import robot_smach_states as states
from robocup_knowledge import knowledge_loader

# Serving drinks
from .sd_states import AskAvailability

# Knowledge
challenge_knowledge = knowledge_loader.load_knowledge("challenge_serving_drinks")


class DriveIn(smach.StateMachine):
    """
    Initialization, inspecting and driving in the room
    """
    def __init__(self, robot, bar_designator, room_designator, objects_list_des, unav_drink_des):
        """
        Initialize at the designated place, drive to and inspect the bar, store the detected drinks or ask for the
        unavailable drink, and navigate to the room where are the operators
        :param robot: robot api object
        :param bar_designator: entity designator of the bar location
        :param room_designator: entity designator of the room location
        :param objects_list_des: variable designator of the detected drinks
        :param unav_drink_des: variable designator of the unavailable drink
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        # ToDo: check the storing of the available drinks: see inspect_shelves from challenge storing groceries,
        #  inspections designator is in smach states -> world model
        #  Classification result is in robot skills -> world model ed

        with self:
            # Initialize
            smach.StateMachine.add("INITIALIZE",
                                   states.StartChallengeRobust(robot=robot,
                                                               initial_pose=challenge_knowledge.starting_point,
                                                               use_entry_points=False, door=False),
                                   transitions={"Done": "INSPECT_BAR",
                                                "Aborted": "aborted",
                                                "Failed": "failed"})

            # Inspect bar and store the list of available drinks
            smach.StateMachine.add("INSPECT_BAR",
                                   states.Inspect(robot=robot, entityDes=bar_designator, navigation_area="in_front_of",
                                                  objectIDsDes=objects_list_des),
                                   transitions={"done": "INSPECT_FALLBACK",  # ToDo: transition to NAVIGATE_TO_ROOM
                                                "failed": "INSPECT_FALLBACK"})

            # Inspect fallback - ask the bartender which drink is unavailable and store the unavailable drink
            smach.StateMachine.add("INSPECT_FALLBACK",
                                   AskAvailability(robot=robot, unavailable_drink_designator=unav_drink_des.writeable),
                                   transitions={"succeeded": "NAVIGATE_TO_ROOM",
                                                "failed": "NAVIGATE_TO_ROOM"})

            # Navigate to the predefined room
            smach.StateMachine.add("NAVIGATE_TO_ROOM",
                                   states.NavigateToRoom(robot=robot, entity_designator_room=room_designator),
                                   transitions={"arrived": "succeeded",
                                                "unreachable": "failed",
                                                "goal_not_defined": "aborted"})
