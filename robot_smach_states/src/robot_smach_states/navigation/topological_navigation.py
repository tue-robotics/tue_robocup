# System
import typing

# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.robot import Robot
from topological_action_planner_msgs.msg import Edge, Node

# Robor Smach States
from robot_smach_states.util.designators import Designator, EdEntityDesignator, EntityByIdDesignator
from .navigate_to_symbolic import NavigateToSymbolic
from .navigate_to_waypoint import NavigateToWaypoint
from .navigation import NavigateTo

# ToDo: allow preemption and continuing to next nav waypoint without stopping
# ToDo: add checks on succeeded/fails
# ToDo: can we apply the 'open-closed principle'? I.e., refactor it such that we can add conversions 'from the outside'?


def convert_msgs_to_actions(robot: Robot, msgs: typing.List[Edge]) -> typing.List[smach.State]:
    """
    Converts a list of 'Edge' messages (typically the result of a call to the planner to a list of smach states to
    execute.

    :param robot: robot API object
    :param msgs:
    :return:
    """
    result = []
    for msg in msgs:
        if msg.action_type == Edge.ACTION_DRIVE:
            result.append(convert_drive_msg_to_action(robot, msg))
    return result


def convert_drive_msg_to_action(robot: Robot, msg: Edge) -> NavigateTo:
    if msg.destination.area:
        destination_entity_designator = EntityByIdDesignator(robot, msg.destination.entity)
        look_at_designator = EntityByIdDesignator(robot, msg.destination.entity)
        return NavigateToSymbolic(
            robot=robot,
            entity_designator_area_name_map={destination_entity_designator: msg.destination.area},
            entity_lookat_designator=look_at_designator,
        )
    else:  # Assume the robot has to navigate to a waypoint
        waypoint_designator = EntityByIdDesignator(robot, msg.destination.entity)
        return NavigateToWaypoint(
            robot=robot,
            waypoint_designator=waypoint_designator,
        )


class GetNavigationActionPlan(smach.State):
    def __init__(self, robot: Robot, entity_designator: EdEntityDesignator, area_designator: Designator = None):
        """
        Gets an action plan from the topological action server

        :param robot: robot API object
        :param entity_designator: designator resolving to the entity to navigate to
        :param area_designator: designator resolving to a string describing the area to navigate to
        """
        smach.State.__init__(
            self, outcomes=["unreachable", "goal_not_defined", "goal_ok", "preempted"], output_keys=["action_plan"]
        )
        self.robot = robot
        self.entity_designator = entity_designator
        self.area_designator = area_designator
        # ToDo: convert to CB state?

    def execute(self, userdata: smach.UserData) -> str:
        # Queries planner
        entity_id = self.entity_designator.resolve().id
        area = self.area_designator.resolve() if self.area_designator is not None else ""
        action_msgs = self.robot.topological_planner.get_plan(entity_id=entity_id, area=area)
        rospy.loginfo(f"Actions: {action_msgs}")
        actions = convert_msgs_to_actions(self.robot, action_msgs)
        userdata.action_plan = actions
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
        entity_designator: EdEntityDesignator,
        area_designator: Designator = None,
    ):
        """
        Move the robot to a specified location.

        :param robot: Robot object
        :param entity_designator: designator resolving to the entity to navigate to
        :param area_designator: designator resolving to a string describing the area to navigate to
        """
        smach.StateMachine.__init__(self, outcomes=["arrived", "unreachable", "goal_not_defined"])

        with self:

            smach.StateMachine.add(
                "GET_PLAN",
                GetNavigationActionPlan(robot, entity_designator, area_designator),
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
