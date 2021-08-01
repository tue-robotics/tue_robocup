# ROS
import smach

# TU/e Robotics
from robot_skills.robot import Robot

# Robot Smach States
from robot_smach_states.human_interaction.human_interaction import Say
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToSymbolic
from robot_smach_states.util.designators import Designator, EdEntityDesignator


class PushObject(smach.StateMachine):
    def __init__(
        self, robot: Robot, entity_designator: EdEntityDesignator, before_area: Designator
    ):
        """
        Mock state for passing doors (this means it should be replaced by the real implementation)

        :param robot: robot API object
        :param object_designator: designator returning the entity to push aside
        :param before_area: indicates the area in front of the entity
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "NAVIGATE_TO_object",
                NavigateToSymbolic(
                    robot,
                    {entity_designator: before_area},
                    entity_designator,
                ),
                transitions={
                    "arrived": "SAY_PUSH_OBJECT",
                    "unreachable": "failed",
                    "goal_not_defined": "failed",
                }
            )

            smach.StateMachine.add(
                "SAY_PUSH_OBJECT",
                Say(robot, "I do hope this table gets out of my way!"),
                transitions={"spoken": "succeeded"}
            )
