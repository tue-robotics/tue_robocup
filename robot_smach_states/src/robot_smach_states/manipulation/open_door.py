# ROS
import smach

# TU/e Robotics
from robot_skills.robot import Robot

# Robot Smach States
from robot_smach_states.human_interaction.human_interaction import Say
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToSymbolic
from robot_smach_states.util.designators import Designator, EdEntityDesignator


class PassDoor(smach.StateMachine):
    def __init__(
        self, robot: Robot, door_designator: EdEntityDesignator, before_area: Designator, behind_area: Designator
    ):
        """
        Mock state for passing doors (this means it should be replaced by the real implementation)

        :param robot: robot API object
        :param door_designator: designator returning the door entity to pass through
        :param before_area: indicates the area in front of the door
        :param behind_area: indicates the area behind the door
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "NAVIGATE_TO_DOOR",
                NavigateToSymbolic(
                    robot,
                    {door_designator: before_area},
                    door_designator,
                ),
                transitions={
                    "arrived": "SAY_PASSING",
                    "unreachable": "failed",
                    "goal_not_defined": "failed",
                }
            )

            smach.StateMachine.add(
                "SAY_DOOR_OPEN",
                Say(robot, "I do hope this door is open!"),
                transitions={"spoken": "DRIVE_THROUGH_DOOR"}
            )

            smach.StateMachine.add(
                "DRIVE_THROUGH_DOOR",
                NavigateToSymbolic(
                    robot,
                    {door_designator: behind_area},
                    door_designator,  # ToDo: This way, the robot looks the wrong way
                ),
                transitions={
                    "arrived": "succeeded",
                    "unreachable": "failed",
                    "goal_not_defined": "failed",
                }
            )
