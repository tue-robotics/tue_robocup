# ROS
import smach

# TU/e Robotics
from robot_skills.arm.arms import PublicArm
from robot_skills.robot import Robot
from robot_skills.util.entity import Entity

# Robot smach states
from robot_smach_states.navigation import NavigateToGrasp
from robot_smach_states.util.designators import ArmDesignator, Designator, check_type


class MoveToGrasp(smach.StateMachine):
    def __init__(self, robot, item, arm):
        # type: (Robot, Designator, ArmDesignator) -> None
        """
        Moves the robot base to a suitable grasp pose. Hereto, it can use the NavigateToGrasp state
        and the ControlToPose state

        :param robot: Robot API object
        :param item: designator that resolves to the item to grab
        :param arm: designator that resolves to the arm to use for grasping
        """
        smach.StateMachine.__init__(self, outcomes=["unreachable", "goal_not_defined", "arrived"])

        # Check types
        check_type(item, Entity)
        check_type(arm, PublicArm)

        # Create state machine
        with self:
            smach.StateMachine.add("NAVIGATE_TO_GRAB", NavigateToGrasp(robot, item, arm),
                                   transitions={"unreachable": "unreachable",
                                                "goal_not_defined": "goal_not_defined",
                                                "arrived": "arrived"})

