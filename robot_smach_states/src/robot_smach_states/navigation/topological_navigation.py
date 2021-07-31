# ROS
import smach

# TU/e Robotics
from robot_skills.robot import Robot

# ToDo: integrate with ROS Service to planner node/plugin
# ToDo: add checks on succeeded/fails


class GetNavigationActionPlan(smach.State):
    def __init__(self, robot: Robot, constraint_function: callable):
        """
        Gets an action plan from the topological action server

        :param robot: robot API object
        """
        smach.State.__init__(
            self, outcomes=["unreachable", "goal_not_defined", "goal_ok", "preempted"], output_keys=["action_plan"]
        )
        self.robot = robot
        # ToDo: convert to CB state?

    def execute(self, userdata: smach.UserData) -> str:
        # Queries planner
        # Example/test code
        from .navigate_to_pose import NavigateToPose
        from robot_smach_states.human_interaction.human_interaction import Say
        plan = [
            NavigateToPose(self.robot, 1.0, 4.5, 3.14),
            Say(self.robot, "Now I pretend to open a door"),
            NavigateToPose(self.robot, 0.0, 4.5, 0.0),
        ]
        userdata.action_plan = plan
        actions = self.robot.topological_planner.get_plan()
        print(f"Actions: {actions}")
        return "goal_ok"


class ExecuteNavigationActionPlan(smach.State):
    def __init__(self, robot: Robot):
        """
        Executes the action plan

        :param robot: robot API object
        """
        smach.State.__init__(
            self, outcomes=["succeeded", "arrived", "blocked", "preempted"], input_keys=["action_plan"])
        self.robot = robot
        # ToDo: convert to CB state?

    def execute(self, userdata: smach.UserData) -> str:
        for action in userdata.action_plan:
            action.execute()  # ToDo: process result
        return "succeeded"


class TopologicalNavigateTo(smach.StateMachine):
    def __init__(
        self,
        robot: Robot,
        constraint_function: callable,
        reset_head: bool = True,
        speak: bool = True,
        reset_pose: bool = True,
    ):
        """
        Move the robot to a specified location.

        :param robot: Robot object
        :param constraint_function: function resolving to a tuple(PositionConstraint, OrientationConstraint)
            telling the robot where to drive to.
        :param reset_head: Whether or not the head should be used for obstacle avoidance during navigation.
        :param speak: Whether or not the robot should speak during navigation
        :param reset_pose: Whether or not the robot is allowed to change its pose for navigation.
        """
        smach.StateMachine.__init__(self, outcomes=["arrived", "unreachable", "goal_not_defined"])
        self.robot = robot
        self.speak = speak

        with self:

            smach.StateMachine.add(
                "GET_PLAN",
                GetNavigationActionPlan(robot, constraint_function),
                transitions={
                    "unreachable": "unreachable",
                    "goal_not_defined": "goal_not_defined",
                    "goal_ok": "EXECUTE_PLAN",
                    "preempted": "unreachable",  # N.B.: NavigateTo does not support 'preempted'
                }
            )

            smach.StateMachine.add(
                "EXECUTE_PLAN",
                ExecuteNavigationActionPlan(robot),
                transitions={
                    "succeeded": "arrived",
                    "arrived": "arrived",
                    "blocked": "unreachable",
                    "preempted": "unreachable",  # N.B.: NavigateTo does not support 'preempted'
                }
            )
