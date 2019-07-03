# ROS
import smach

# Robot smach states
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills.classification_result import ClassificationResult

# Serving drinks
from .sd_states import AskAvailability


class CheckInspect(smach.state):
    def __init__(self, designator, *resolve_types):
        super(CheckInspect, self).__init__(outcomes=["true", "false"])
        ds.check_type(designator, *resolve_types)
        self.designator = designator

    def execute(self, userdata=None):
        val = self.designator.resolve() if hasattr(self.designator, "resolve") else self.designator
        if val:
            return "true"
        else:
            return "false"


class DriveIn(smach.StateMachine):
    """
    Initialize at the designated place, drive to and inspect the bar, store the detected drinks or ask for the
    unavailable drink, and navigate to the room where are the operators
    """
    def __init__(self, robot, initial_pose, bar_designator, room_designator,
                 objects_list_des, unav_drink_des, objects):
        """
        Initialization method
        :param robot: robot api object
        :param initial_pose: Initial pose of the robot
        :param bar_designator: (EntityDesignator) in which the bar location is stored
        :param room_designator: (EntityDesignator) in which the room location is stored
        :param objects_list_des: (VariableDesignator) in which the available drinks are stored
        :param unav_drink_des: (VariableDesignator) in which the unavailable drink is stored
        :param objects: Objects from common knowledge
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        # ToDo: check the storing of the available drinks: see inspect_shelves from challenge storing groceries,
        #  inspections designator is in smach states -> world model
        #  Classification result is in robot skills -> world model ed

        with self:
            # Initialize
            smach.StateMachine.add("INITIALIZE",
                                   states.StartChallengeRobust(robot=robot,
                                                               initial_pose=initial_pose,
                                                               use_entry_points=False, door=False),
                                   transitions={"Done": "INSPECT_BAR",
                                                "Aborted": "aborted",
                                                "Failed": "failed"})

            # Inspect bar and store the list of available drinks
            smach.StateMachine.add("INSPECT_BAR",
                                   states.Inspect(robot=robot, entityDes=bar_designator, navigation_area="in_front_of",
                                                  objectIDsDes=objects_list_des),
                                   transitions={"done": "CHECK_INSPECT_RESULT",
                                                "failed": "INSPECT_FALLBACK"})

            smach.StateMachine.add("CHECK_INSPECT_RESULT",
                                   CheckInspect(objects_list_des, [ClassificationResult]),
                                   transitions={"true": "NAVIGATE_TO_ROOM",
                                                "false": "INSPECT_FALLBACK"})

            # Inspect fallback - ask the bartender which drink is unavailable and store the unavailable drink
            smach.StateMachine.add("INSPECT_FALLBACK",
                                   AskAvailability(robot=robot,
                                                   unavailable_drink_designator=unav_drink_des.writeable,
                                                   objects=objects),
                                   transitions={"succeeded": "NAVIGATE_TO_ROOM",
                                                "failed": "NAVIGATE_TO_ROOM"})

            # Navigate to the predefined room
            smach.StateMachine.add("NAVIGATE_TO_ROOM",
                                   states.NavigateToRoom(robot=robot, entity_designator_room=room_designator),
                                   transitions={"arrived": "succeeded",
                                                "unreachable": "failed",
                                                "goal_not_defined": "aborted"})
