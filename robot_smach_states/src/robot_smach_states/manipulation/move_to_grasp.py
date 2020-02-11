# System
import math

# ROS
import rospy
import geometry_msgs.msg
import PyKDL as kdl
import smach

# TU/e Robotics
from robot_skills.arms import PublicArm
from robot_skills.robot import Robot
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import kdl_frame_to_pose_msg, FrameStamped

# Robot smach states
from robot_smach_states.navigation import NavigateToGrasp
from robot_smach_states.navigation.control_to_pose import ControlToPose, ControlParameters
from robot_smach_states.util.designators import ArmDesignator, Designator, check_type


class _GoalPoseDesignator(Designator):
    def __init__(self):
        """
        Customer designator for this purpose. ToDo: use something more generic?
        """
        super(_GoalPoseDesignator, self).__init__(resolve_type=geometry_msgs.msg.PoseStamped)
        self._goal_pose = geometry_msgs.msg.PoseStamped()
        self._goal_pose.header.frame_id = "map"

    def lockable(self):
        return False

    def write(self, goal_pose_kdl):
        # type: (kdl.Frame) -> None
        """
        Stores the goal pose as a geometry msgs PoseStamped

        :param goal_pose_kdl: goal pose as kdl Frame in map
        """
        self._goal_pose.pose = kdl_frame_to_pose_msg(goal_pose_kdl)

    def resolve(self):
        return self._goal_pose


def _point_between_points_at_distance(p0, p1, reference_distance):
    # type: (kdl.Vector, kdl.Vector, float) -> kdl.Vector
    """
    Computes a point on the line between p0 and p1 at a distance of reference_distance from p0
    """
    vector = kdl.diff(p0, p1)
    return p0 + reference_distance / vector.Norm() * vector


def _create_straight_line_plan(start, goal, resolution=0.025):
    # type: (kdl.Vector, kdl.Vector, float) -> typing.List[geometry_msgs.msg.PoseStamped]
    """
    Computes a list of geometry_msgs.msg.PoseStamped ("/map" is added as frame id) between
    start end goal pose

    :param start: start position
    :param goal: goal position
    :param resolution: resolution of the plan
    :return plan
    """
    vector = kdl.diff(start, goal)
    nr_samples = int(math.ceil(vector.Norm() / resolution))
    result = []
    for i in range(nr_samples):
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = "/map"
        pose_stamped.pose = kdl_frame_to_pose_msg(kdl.Frame(start + float(i) / nr_samples * vector))
        result.append(pose_stamped)

    # Append goal pose
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = "/map"
    pose_stamped.pose = kdl_frame_to_pose_msg(kdl.Frame(goal))
    result.append(pose_stamped)
    return result


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

        goal_pose_designator = _GoalPoseDesignator()
        navigate_state = NavigateToGrasp(robot, item, arm)
        control_parameters = ControlParameters(0.5, 1.0, 0.3, 0.3, 0.3, 0.05, 0.1)
        distance_threshold = 0.5  # The plan must be valid and we don't want to 'ForceDrive' more than a this distance

        # Create state machine
        with self:
            @smach.cb_interface(input_keys=[], output_keys=[], outcomes=["control", "navigate"])
            def determine_approach(_=None):
                try:
                    entity_pose, radius, angle_offset = navigate_state.determine_offsets()
                    rospy.loginfo("Entity pose: {}, radius: {}, offset: {}".format(entity_pose, radius, angle_offset))
                except RuntimeError as e:
                    rospy.logwarn("Cannot compute offsets: {}. Will try to navigate.".format(e.message))
                    return "navigate"

                base_pose = robot.base.get_location()  # type: FrameStamped
                goal_position = self.compute_goal_position(base_pose.frame.p, entity_pose.frame.p, radius)
                rospy.logdebug("robot_position: {}".format(base_pose.frame.p))
                rospy.logdebug("entity position: {}".format(entity_pose.frame.p))
                rospy.logdebug("goal position: {}".format(goal_position))

                plan = _create_straight_line_plan(base_pose.frame.p, goal_position)
                for pose_stamped in plan:  # type: geometry_msgs.msg.PoseStamped
                    pose_stamped.pose.position.z = 0.0
                distance = robot.base.global_planner.computePathLength(plan)
                valid = robot.base.global_planner.checkPlan(plan)

                if not valid or distance > distance_threshold:
                    rospy.logwarn("Distance: {}, valid: {}".format(distance, valid))
                    if not valid:
                        for p in plan:
                            rospy.logwarn(p.pose.position)
                    return "navigate"

                goal_orientation = self.compute_goal_orientation(
                    base_pose.frame.p, entity_pose.frame.p, goal_position, angle_offset
                )
                goal_pose_designator.write(kdl.Frame(goal_orientation, goal_position))
                return "control"

            smach.StateMachine.add(
                "DETERMINE_APPROACH",
                smach.CBState(determine_approach),
                {"control": "CONTROL", "navigate": "NAVIGATE"}
            )

            smach.StateMachine.add(
                "CONTROL",
                ControlToPose(robot, goal_pose_designator, control_parameters),
                {"succeeded": "arrived", "failed": "unreachable"}
            )

            smach.StateMachine.add("NAVIGATE", navigate_state,
                                   transitions={"unreachable": "unreachable",
                                                "goal_not_defined": "goal_not_defined",
                                                "arrived": "arrived"})
            # ToDo: add final control?

    @staticmethod
    def compute_goal_position(robot_position, entity_position, radius):
        # type: (kdl.Vector, kdl.Vector, float) -> kdl.Vector
        """
        Computes the goal position for the control state based on the robot position, the entity position and the
        radius, i.e., the desired distance from the entity
        """
        robot_position_cp = kdl.Vector(robot_position)
        robot_position_cp.z(0.0)
        entity_position_cp = kdl.Vector(entity_position)
        entity_position_cp.z(0.0)
        return _point_between_points_at_distance(entity_position_cp, robot_position_cp, radius)

    @staticmethod
    def compute_goal_orientation(robot_position, entity_position, goal_position, angle_offset):
        # type: (kdl.Vector, kdl.Vector, entity_position, float) -> kdl.Rotation
        """
        Computes the goal orientation for the control state based on the robot position, the *goal* position and the
        desired angle offset
        """
        result = kdl.Rotation.RPY(
            0.0,
            0.0,
            math.atan2(goal_position.y() - robot_position.y(), goal_position.x() - robot_position.x())
        )
        result.DoRotZ(angle_offset)

        # If the robot is already closer to the entity than the desired radius, this will result in a 180 degree
        # wrong result. If this is the case: fix it
        if kdl.diff(robot_position, entity_position).Norm() < kdl.diff(goal_position, entity_position).Norm():
            result.DoRotZ(math.pi)

        return result

