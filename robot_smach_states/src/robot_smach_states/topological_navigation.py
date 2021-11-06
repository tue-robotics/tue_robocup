# System
import typing

# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.arm import arms
from robot_skills.robot import Robot
from topological_action_planner_msgs.msg import Edge

# Robot Smach States
from robot_smach_states.manipulation.open_door import OpenDoor
from robot_smach_states.util.designators import (
    Designator,
    EdEntityDesignator,
    EntityByIdDesignator,
    UnoccupiedArmDesignator,
)
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


def convert_open_door_msg_to_action(robot: Robot, msg: Edge) -> OpenDoor:
    door_designator = EntityByIdDesignator(robot, msg.destination.entity)
    arm_des = UnoccupiedArmDesignator(
        robot,
        {
            "required_goals": ["reset", "handover"],
            "force_sensor_required": True,
            "required_gripper_types": [arms.GripperTypes.GRASPING],
        },
    )
    return OpenDoor(robot=robot, arm_designato=arm_des, door_designator=door_designator)


@smach.cb_interface(
    outcomes=["unreachable", "goal_not_defined", "goal_ok", "preempted"],
    output_keys=["action_plan"],
)
def get_topological_action_plan(
    userdata: smach.UserData,
    robot: Robot,
    entity_designator: EdEntityDesignator,
    area_designator: Designator = None,
) -> str:
    """
    Gets an action plan from the topological action server

    :param userdata: smach userdata
    :param robot: robot API object
    :param entity_designator: designator resolving to the entity to navigate to
    :param area_designator: designator resolving to a string describing the area to navigate to
    """
    # Queries planner
    entity_id = entity_designator.resolve().id
    area = area_designator.resolve() if area_designator is not None else ""
    action_msgs = robot.topological_planner.get_plan(entity_id=entity_id, area=area)
    rospy.loginfo(f"Actions: {action_msgs}")

    try:
        actions = convert_msgs_to_actions(robot, action_msgs)
    except TopologicalPlannerException as e:
        rospy.logerr(e.message)
        return "unreachable"
    userdata.action_plan = actions
    return "goal_ok"


@smach.cb_interface(
    outcomes=["succeeded", "arrived", "blocked", "preempted"],
    input_keys=["action_plan"],
    output_keys=["failing_edge"],
)
def execute_topological_plan(userdata: smach.UserData) -> str:
    """
    Sequentially executes all actions in the action plan provided through UserData

    :param userdata: smach userdata
    :return: outcome
    """
    for action in userdata.action_plan:
        result = action.execute()  # ToDo: process result
        if result not in ["succeeded", "done", "arrived"]:
            rospy.loginfo("action: {} had result {}".format(action, result))
            if result == "preempted":
                return "preempted"
            userdata.failing_edge = action
            return "blocked"
        rospy.sleep(1.0)
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
                smach.CBState(get_topological_action_plan),
                transitions={
                    "unreachable": "unreachable",
                    "goal_not_defined": "goal_not_defined",
                    "goal_ok": "EXECUTE_PLAN",
                    "preempted": "unreachable",  # N.B.: NavigateTo does not support 'preempted'
                },
            )

            smach.StateMachine.add(
                "EXECUTE_PLAN",
                smach.CBState(execute_topological_plan, robot, entity_designator, area_designator),
                transitions={
                    "succeeded": "arrived",
                    "arrived": "arrived",
                    "blocked": "unreachable",
                    "preempted": "unreachable",  # N.B.: NavigateTo does not support 'preempted'
                },
            )
