# ROS
import smach

# Robot smach states
import robot_smach_states as states
from robocup_knowledge import knowledge_loader

# Serving drinks
from .sd_states import AskAvailability

# Knowledge
CHALLENGE_KNOWLEDGE = knowledge_loader.load_knowledge("challenge_serving_drinks")


class DriveIn(smach.StateMachine):
    """ Serves on drink to an operator

    """
    def __init__(self, robot, bar_designator, room_designator, objects_list_des, unav_drink_des):
        """ Initialization method

        :param robot: robot api object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        # ToDo: check the storing of the available drinks: see inspect_shelves from challenge storing groceries,
        #  inspections designator is in smach states -> world model
        #  Classification result is in robot skills -> world model ed

        with self:
            # Initialize
            smach.StateMachine.add(
                'INITIALIZE',
                states.Initialize(robot),
                transitions={"initialized": "WAIT_1",
                             "abort": "aborted"}
            )

            # Wait for one second so that initialise has time to finish
            smach.StateMachine.add(
                    "WAIT_1",
                    states.WaitTime(
                        robot=robot,
                        waittime=1),
                    transitions={"waited": "SET_INITIAL_POSE",
                                 "preempted": "SET_INITIAL_POSE"}
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
                    navigation_area="in_front_of",
                    objectIDsDes=objects_list_des),
                transitions={"done": "INSPECT_FALLBACK",  # ToDo: transition to NAVIGATE_TO_ROOM
                             "failed": "INSPECT_FALLBACK"}
            )

            # Inspect fallback - ask the bartender which drink is unavailable and store the unavailable drink
            smach.StateMachine.add(
                "INSPECT_FALLBACK",
                AskAvailability(
                    robot=robot,
                    unavailable_drink_designator=unav_drink_des.writeable),
                transitions={"succeeded": "NAVIGATE_TO_ROOM",
                             "failed": "NAVIGATE_TO_ROOM"},
            )

            # Navigate to the predefined room
            smach.StateMachine.add(
                "NAVIGATE_TO_ROOM",
                states.NavigateToRoom(
                    robot=robot,
                    entity_designator_room=room_designator),
                transitions={"arrived": "succeeded",
                             "unreachable": "failed",
                             "goal_not_defined": "aborted"}
            )
