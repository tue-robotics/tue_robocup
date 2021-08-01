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
from robot_smach_states.manipulation.open_door import PassDoor
from robot_smach_states.push_object import PushObject
from .navigation.navigate_to_symbolic import NavigateToSymbolic
from .navigation.navigate_to_waypoint import NavigateToWaypoint
from .navigation.navigation import NavigateTo

# ToDo: allow preemption and continuing to next nav waypoint without stopping
# ToDo: add recovery behaviour on fails
# ToDo: can we apply the 'open-closed principle'? I.e., refactor it such that we can add conversions 'from the outside'?


class TopologicalPlannerException(Exception):
    pass


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
        elif msg.action_type == Edge.ACTION_OPEN_DOOR:
            result.append(convert_open_door_msg_to_action(robot, msg))
        elif msg.action_type == Edge.ACTION_PUSH_OBJECT:
            result.append(convert_push_object_msg_to_action(robot, msg))
        else:
            raise TopologicalPlannerException(f"Do not have action for type {msg.action_type}")
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


def convert_open_door_msg_to_action(robot: Robot, msg: Edge) -> PassDoor:
    if msg.origin.entity != msg.destination.entity:
        raise TopologicalPlannerException(
            f"OpenDoor action: origin entity ({msg.origin.entity}) "
            f"does not match destination entity ({msg.destination.entity})"
        )
    if not msg.origin.area:
        raise TopologicalPlannerException(f"OpenDoor: 'before' area is empty")
    if not msg.destination.area:
        raise TopologicalPlannerException(f"OpenDoor: 'behind' area is empty")
    door_designator = EntityByIdDesignator(robot, msg.destination.entity)
    before_area = Designator(msg.origin.area, str)
    behind_area = Designator(msg.destination.area, str)
    return PassDoor(
        robot=robot,
        door_designator=door_designator,
        before_area=before_area,
        behind_area=behind_area,
    )


def convert_push_object_msg_to_action(robot: Robot, msg: Edge) -> PushObject:
    if not msg.origin.area:
        raise TopologicalPlannerException(f"PushObject: 'before' area is empty")
    entity_designator = EntityByIdDesignator(robot, msg.destination.entity)
    before_area = Designator(msg.origin.area, str)
    return PushObject(
        robot=robot,
        entity_designator=entity_designator,
        before_area=before_area,
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

        # action_msgs = self.robot.topological_planner.collapse_plan(action_msgs)
        # rospy.loginfo(f"After plan simplification: {action_msgs}")

        try:
            actions = convert_msgs_to_actions(self.robot, action_msgs)
        except TopologicalPlannerException as e:
            rospy.logerr(e.message)
            return "unreachable"
        userdata.action_plan = actions
        return "goal_ok"


class ExecuteNavigationActionPlan(smach.State):
    def __init__(self, robot: Robot):
        """
        Executes the action plan

        :param robot: robot API object
        """
        smach.State.__init__(
            self, outcomes=["succeeded", "arrived", "blocked", "preempted"], input_keys=["action_plan"], output_keys=["failing_edge"])
        self.robot = robot
        # ToDo: convert to CB state?

    def execute(self, userdata: smach.UserData) -> str:
        for action in userdata.action_plan:
            result = action.execute()  # ToDo: process result
            if result not in ['succeeded', 'done', 'arrived']:
                rospy.loginfo("action: {} had result {}".format(action, result))
                if result == "preempted":
                    return "preempted"
                userdata.failing_edge = action
                return "blocked"
        return "succeeded"


class UpdateNavigationGraph(smach.State):
    def __init__(self, robot: Robot):
        """
        Executes the action plan

        :param robot: robot API object
        """
        smach.State.__init__(
            self, outcomes=["done"], input_keys=["failing_edge"])
        self.robot = robot

    def execute(self, userdata: smach.UserData) -> str:
        self.robot.topological_planner.update_edge_cost(userdata.failing_edge, 50)  # TODO magic number
        return "done"


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
                    "blocked": "UPDATE_GRAPH",
                    "preempted": "unreachable",  # N.B.: NavigateTo does not support 'preempted'
                }
            )

            smach.StateMachine.add(
                "UPDATE_GRAPH",
                UpdateNavigationGraph(robot),
                transitions={
                    "done": "GET_PLAN",
                }
            )
