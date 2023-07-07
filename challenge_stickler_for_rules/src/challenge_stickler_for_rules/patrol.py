"""
Module contains states to patrol the environment while navigating to a goal.
"""
from __future__ import absolute_import

# ROS
import rospy
import smach

from robot_skills.robot import Robot
from robot_smach_states.navigation import navigation
from robot_smach_states.navigation.constraint_functions import symbolic_constraint, look_at_constraint, \
    combine_constraints
# Robot skills
from robot_smach_states.util.designators import EdEntityDesignator


class ExecutePlanPatrol(smach.State):
    """
    Similar to the "executePlan" smach state. The only difference is that after driving for x meters, "check for
    operator" is returned.
    """

    def __init__(self, robot, reset_pose=True):
        # type: (Robot, bool, bool) -> None
        """
        :param robot: (Robot) robot api object
        """
        smach.State.__init__(self, outcomes=["arrived", "blocked", "preempted"])
        self.robot = robot
        self.reset_pose = reset_pose

    def execute(self, userdata=None):
        # Move the robot to a suitable driving pose
        if self.reset_pose and self.robot.base.global_planner.path_length > 0.5:
            self.robot.go_to_driving_pose()

        rate = rospy.Rate(10.0)  # Loop at 10 Hz
        while not rospy.is_shutdown():

            if self.preempt_requested():
                self.robot.base.local_planner.cancelCurrentPlan()
                rospy.loginfo("execute: preempt_requested")
                return "preempted"

            status = self.robot.base.local_planner.getStatus()

            if status == "arrived":
                return "arrived"
            elif status == "blocked":
                return "blocked"

            # TODO look around and detect people.

            # TODO check if detected people are violating rules.

            # TODO if people have violated rules add a transition/outcome to deal with it.

            rate.sleep()


class Patrol(smach.StateMachine):
    def __init__(self, robot, constraint_function, speak=True, reset_pose=True):
        # type: (Robot, function, bool, bool) -> None
        """
        Base Smach state to patrol to a designated position. monitoring a set of functions.

        :param robot: (Robot) robot api object
        """
        smach.StateMachine.__init__(
            self, outcomes=["arrived", "unreachable", "goal_not_defined", "preempted"])
        self.robot = robot
        self.speak = speak
        self.execute_plan = ExecutePlanPatrol(robot=self.robot,
                                              )

        with self:
            smach.StateMachine.add("GET_AND_SET_PLAN", navigation.getPlan(self.robot, constraint_function, speak),
                                   transitions={"unreachable": "unreachable",
                                                "goal_not_defined": "goal_not_defined",
                                                "goal_ok": "MONITOR_PLAN"})

            smach.StateMachine.add("MONITOR_PLAN", self.execute_plan,
                                   transitions={"arrived": "arrived",
                                                "blocked": "PLAN_BLOCKED",
                                                "preempted": "preempted"})

            smach.StateMachine.add("PLAN_BLOCKED", navigation.planBlocked(self.robot),
                                   transitions={"blocked": "GET_AND_SET_PLAN",
                                                "free": "MONITOR_PLAN"})


class PatrolToSymbolic(Patrol):
    """
    Guidance class to navigate to a semantically annotated goal, e.g., in front of the dinner table.
    """

    def __init__(self, robot, entity_designator_area_name_map, entity_lookat_designator,
                 speak=True, reset_pose=True):
        # type: (Robot, dict, EdEntityDesignator, bool, bool) -> None
        """
        Constructor

        :param robot: robot object
        :param entity_designator_area_name_map: dictionary mapping EdEntityDesignators to a string or designator
            resolving to a string, representing the area, e.g., entity_designator_area_name_map[<EdEntity>] = 'in_front_of'.
        :param entity_lookat_designator: EdEntityDesignator defining the entity the robot should look at. This is used
            to compute the orientation constraint.
        """
        constr_fun = lambda: combine_constraints(
            [lambda: symbolic_constraint(robot, entity_designator_area_name_map),
             lambda: look_at_constraint(entity_lookat_designator)]
        )
        super(PatrolToSymbolic, self).__init__(robot=robot,
                                               constraint_function=constr_fun,
                                               speak=speak, reset_pose=reset_pose
                                               )


class PatrolToRoom(PatrolToSymbolic):
    """
    Patrol class to navigate to the 'in' area of the provided entity, typically a room.

    :param robot: robot object
    :param entity_designator_room: Designator to the room
    :param entity_lookat_designator: (Optional) Designator defining the entity the robot should look at. This is
        used to compute the orientation constraint. If not provided, the entity_designator_room is used.
    :param speak: Whether or not the robot should speak during navigation
    :param reset_pose: Whether or not the robot is allowed to change its pose for navigation.
    """

    def __init__(self, robot, entity_designator_room, entity_lookat_designator=None, speak=True,
                 reset_pose=True):
        room_area = "in"
        if not entity_lookat_designator:
            entity_lookat_designator = entity_designator_room
        super(PatrolToRoom, self).__init__(robot, {entity_designator_room: room_area}, entity_lookat_designator,
                                           speak=speak, reset_pose=reset_pose)
