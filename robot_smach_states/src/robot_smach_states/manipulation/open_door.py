# System
import typing

# ROS
import rospy
import smach
import actionlib
import math
import numpy
import PyKDL as kdl
from pykdl_ros import FrameStamped, VectorStamped
from geometry_msgs.msg import PointStamped, Point

# TU/e Robotics
from robot_skills.robot import Robot
from robot_skills.arm import arms
from robot_skills.util.entity import Entity
from cb_base_navigation_msgs.msg import OrientationConstraint, PositionConstraint

# Robot Smach States
from robot_smach_states.navigation import NavigateTo
from robot_smach_states.human_interaction.human_interaction import Say
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToSymbolic
from robot_smach_states.util.designators import check_type, Designator, EdEntityDesignator, UnoccupiedArmDesignator
from tue_msgs.msg import LocateDoorHandleGoal


class Direction:
    INWARD = "inward"
    OUTWARD = "outward"


class Door(Entity):
    HANDLE_ID = "handle"
    FRAME_LEFT_POINT_ID = "frame_left_point"
    FRAME_RIGHT_POINT_ID = "frame_right_point"

    def __init__(self, entity: Entity):
        super().__init__(
            identifier=entity.id,
            object_type=entity.type,
            frame_id=entity.frame_id,
            pose=entity.pose.frame,
            shape=entity.shape,
            volumes=entity.volumes,
            super_types=entity.super_types,
            last_update_time=entity.last_update_time,
        )

    @property
    def handle_pose(self) -> VectorStamped:
        """
        Returns the pose of the handle in map frame
        """
        return self._get_volume_center_point_in_map(self.HANDLE_ID)

    @property
    def frame_points(self) -> typing.List[VectorStamped]:
        """
        Returns the ground points of the door frame in map frame
        """
        return [self._get_volume_center_point_in_map(self.FRAME_LEFT_POINT_ID),
                self._get_volume_center_point_in_map(self.FRAME_RIGHT_POINT_ID)]

    def _get_volume_center_point_in_map(self, volume_id: str) -> VectorStamped:
        """
        Gets the center point of a volume (typically defined w.r.t. the entity frame) and converts this to map frame.

        :param volume_id: id of the volume to get the center point from
        :return: center point converted to map frame
        """
        cp_entity = self.volumes[volume_id].center_point
        cp_map = self.pose.frame * cp_entity
        return VectorStamped.from_xyz(cp_map.x(), cp_map.y(), cp_map.z(), rospy.Time.now(),"map")

    def get_direction(self, base_pose: FrameStamped) -> str:
        """
        Determines whether the open the door inward or outward.

        In the current implementation, this is simply deduced from the fact if the robot is currently closer to the
        'inward' or 'outward' volume

        :param base_pose: pose of the base
        :return: inward or outward
        """
        base_pos = base_pose.frame.p
        inward_pos = self._get_volume_center_point_in_map(Direction.INWARD).vector
        outward_pos = self._get_volume_center_point_in_map(Direction.OUTWARD).vector
        delta_inward = (base_pos - inward_pos).Norm()
        delta_outward = (base_pos - outward_pos).Norm()
        return Direction.INWARD if delta_inward < delta_outward else Direction.OUTWARD


class OpenDoor(smach.StateMachine):
    # def __init__(self, robot, arm_designator, door_designator, in_front_area_designator, behind_area_designator):
    def __init__(self, robot, door_designator, arm_designator=None):
        """
        Enter the arena by force driving through the door
        :param robot: robot object
        :param door_designator: door_designator object
        :param arm_designator: Designator resolving to an arm. Defaults to None to use an UnoccupiedArm
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        if arm_designator is None:
            arm_designator = UnoccupiedArmDesignator(robot, {"required_goals": ["reset", "handover"],
                                                             "force_sensor_required": True,
                                                             "required_gripper_types": [arms.GripperTypes.GRASPING]})
        check_type(arm_designator, arms.PublicArm)

        with self:

            smach.StateMachine.add('NAVIGATE_TO_HANDLE', NavigateToHandle(robot, door_designator, arm_designator),
                                   transitions={'unreachable': 'failed',
                                                'arrived': 'GET_DOOR_STATE',
                                                'goal_not_defined': 'failed'})

            smach.StateMachine.add('GET_DOOR_STATE', DetermineDoorState(robot, door_designator),
                                #    transitions={'open': 'NAVIGATE_THROUGH_DOOR',
                                   transitions={'open': 'succeeded',
                                                'closed': 'UPDATE_HANDLE_LOCATION',
                                                'intermediate': 'DETERMINE_DOOR_DIRECTION',
                                                'failed': 'failed'})

            smach.StateMachine.add('UPDATE_HANDLE_LOCATION', UpdateHandleLocation(robot, door_designator),
                                   transitions={'succeeded': 'GRASP_HANDLE',
                                                'failed': 'failed'})

            smach.StateMachine.add('GRASP_HANDLE', GraspHandle(robot, door_designator, arm_designator),
                                   transitions={'succeeded': 'UNLATCH_HANDLE',
                                                'failed': 'failed'})

            smach.StateMachine.add('UNLATCH_HANDLE', UnlatchHandle(robot, door_designator, arm_designator),
                                   transitions={'succeeded': 'DETERMINE_DOOR_DIRECTION',
                                                'failed': 'failed'})

            smach.StateMachine.add('DETERMINE_DOOR_DIRECTION', DetermineDoorDirection(robot, door_designator),
                                   transitions={'outward': 'PUSH_DOOR_OPEN',
                                                'inward': 'PULL_DOOR_OPEN',
                                                'failed': 'failed'})

            smach.StateMachine.add('PUSH_DOOR_OPEN', PushDoorOpen(robot, door_designator, arm_designator),
                                #    transitions={'succeeded': 'NAVIGATE_THROUGH_DOOR',
                                   transitions={'succeeded': 'succeeded',
                                                'failed': 'failed'})

            smach.StateMachine.add('PULL_DOOR_OPEN', PullDoorOpen(robot, door_designator, arm_designator),
                                #    transitions={'succeeded': 'NAVIGATE_THROUGH_DOOR',
                                   transitions={'succeeded': 'succeeded',
                                                'failed': 'failed'})

            # smach.StateMachine.add('PASS_DOOR', PassDoor(robot, door_designator, in_front_area_designator, behind_area_designator),
            #                        transitions={'succeeded': 'succeeded',
            #                                     'failed': 'failed'})


class NavigateToHandle(NavigateTo):
    def __init__(self, robot, door_des, arm_des):
        super(NavigateToHandle, self).__init__(robot, self.generateConstraint)

        self._robot = robot
        self._door_des = door_des
        self._arm_des = arm_des

    def generateConstraint(self):
        door = self._door_des.resolve()
        arm = self._arm_des.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "done"

        handle_point = kdl_con.kdl_vector_stamped_to_point_stamped(door.handle_pose)
        angle_offset =-math.atan2(arm.base_offset.y(), arm.base_offset.x())
        radius = 0.75*math.hypot(arm.base_offset.x(), arm.base_offset.y())

        handle_point_map = self._robot.tf_buffer.transform(handle_point, "map")
        x = handle_point_map.point.x
        y = handle_point_map.point.y

        # Outer radius
        ro = "(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, radius+0.15)
        ri = "(x-%f)^2+(y-%f)^2 > %f^2"%(x, y, radius-0.15)
        pc = PositionConstraint(constraint=ri+" and "+ro, frame="map")
        oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="map", angle_offset=angle_offset)

        return pc, oc


class DetermineDoorState(smach.State):
    def __init__(self, robot, door_des):
        """
        Constructor
        :param robot: robot object
        :type robot: robot
        """
        smach.State.__init__(self, outcomes=['open', 'closed', 'intermediate', 'failed'])
        self._robot = robot
        self._door_des = door_des

    def execute(self, userdata=None):
        door = self._door_des.resolve()
        return "closed"


class UpdateHandleLocation(smach.State):
    def __init__(self, robot, door_des):
        """
        Constructor
        :param robot: robot object
        :type robot: robot
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._robot = robot
        self._door_des = door_des

    def execute(self, userdata=None):
        door = self._door_des.resolve()
        handle_estimate = door.handle_pose
        self._robot.head.look_at_point(VectorStamped.from_framestamped(handle_estimate), timeout=0.0)

        self._robot.head.wait_for_motion_done()

        goal = LocateDoorHandleGoal()
        goal_estimate = PointStamped()
        goal_estimate.header.frame_id = handle_estimate.frame_id
        goal_estimate.point.x = handle_estimate.vector.x()
        goal_estimate.point.y = handle_estimate.vector.y()
        goal_estimate.point.z = handle_estimate.vector.z()

        goal.handle_location_estimate = goal_estimate
        rospy.logdebug("Goal for updating door handle action is {}".format(goal))
        self._robot.perception.locate_handle_client.send_goal(goal)
        self._robot.perception.locate_handle_client.wait_for_result(rospy.Duration.from_sec(5.0))

        state = self._robot.perception.locate_handle_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            result = self._robot.perception.locate_handle_client.get_result()
            rospy.logdebug("Updated handle location is{}".format(result))
            x = numpy.average([result.handle_edge_point1.point.x, result.handle_edge_point2.point.x])
            y = numpy.average([result.handle_edge_point1.point.y, result.handle_edge_point2.point.y])
            z = numpy.average([result.handle_edge_point1.point.z, result.handle_edge_point2.point.z])
            handle_loc = VectorStamped.from_xyz(x, y, z, rospy.Time.now(), result.handle_edge_point1.header.frame_id)
            # This should update the door entity (probs via the designator)
            door.handle_pose = self._robot.tf_buffer.transform(handle_loc, "map")

            return "succeeded"
        else:
            return "failed"


class GraspHandle(smach.State):
    def __init__(self, robot, door_des, arm_des):
        """
        Grasp the handle.
        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._robot = robot
        self._door_des = door_des
        self._arm_des = arm_des

    def execute(self, userdata=None):
        door = self._door_des.resolve()
        arm = self._arm_des.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "succeeded"

        arm.send_gripper_goal("open")

        handle_point = door.handle_pose

        align_door = 0.0  # -0.3 This depends on the orientation of the door wrt the base link frame

        handle_framestamped = FrameStamped(kdl.Frame(kdl.Rotation.RPY(-1.57, 0.0, align_door),
                                                     handle_point.vector),
                                           rospy.Time.now(),
                                           frame_id=handle_point.header.frame_id)
        goal_bl = self._robot.tf_buffer.transform(handle_framestamped, self._robot.base_link_frame)

        # Move to in front of handle, this is heavily tuned so not robust!
        goal_bl.frame.p.x(goal_bl.frame.p.x() - 0.08)
        goal_bl.frame.p.y(goal_bl.frame.p.y() + 0.01)

        goal_bl.frame.M = kdl.Rotation.RPY(-1.57, 0.0, align_door)  # Update the roll
        result = arm.send_goal(goal_bl, timeout=0.0)
        arm.wait_for_motion_done()

        if result:
            arm.send_gripper_goal('close', max_torque=1.0)
            return "succeeded"
        else:
            return "failed"


class UnlatchHandle(smach.State):
    def __init__(self, robot, door_des, arm_des):
        """
        Wait till the average distance in front of the robot is bigger than 1 meter. Only point in front of the middle
        are taken into account
        :param robot: robot object
        :param timeout: timeout for waiting till the door is opened
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self._move_dist = 0.03
        self._robot = robot
        self._door_des = door_des
        self._arm_des = arm_des

    def execute(self, userdata=None):
        door = self._door_des.resolve()
        arm = self._arm_des.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return None

        arm.send_gripper_goal('close', max_torque=1.0)

        current_pose = self._robot.tf_buffer.lookup_transform(self._robot.base_link_frame, 'hand_palm_link', rospy.Time(0))

        rot = current_pose.transform.rotation
        orientation = kdl.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w)
        (curr_r, curr_p, curr_y) = orientation.GetRPY()

        trans = current_pose.transform.translation
        next_z = trans.z - self._move_dist

        next_pose = FrameStamped.from_xyz_rpy(trans.x, trans.y, next_z, curr_r, curr_p, curr_y, rospy.Time.now(),
                                              self._robot.base_link_frame)

        result = arm.send_goal(next_pose)
        if result:
            return "succeeded"
        return "failed"


class DetermineDoorDirection(smach.State):
    def __init__(self, robot, door_des):
        """

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=['outward', 'inward', 'failed'])

        self._robot = robot
        self._door_des = door_des

    def execute(self, userdata=None):
        door = self._door_des.resolve()
        print(door.get_direction(self._robot.base.get_location()))
        if door.get_direction(self._robot.base.get_location()) == 'inward':
            return 'inward'
        elif door.get_direction(self._robot.base.get_location()) == 'outward':
            return 'outward'
        return 'failed'


class PushDoorOpen(smach.State):
    def __init__(self, robot, door_des, arm_des):
        """

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self._robot = robot
        self._door_des = door_des
        self._arm_des = arm_des

    def execute(self, userdata=None):
        door = self._door_des.resolve()
        arm = self._arm_des.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return None

        next_pose_framestamped = FrameStamped(kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(0.1, 0.05, 0.0)),
                                              rospy.Time.now(), "hand_palm_link")
        goal_bl = self._robot.tf_buffer.transform(next_pose_framestamped, self._robot.base_link_frame)

        arm_goal_res = arm.send_goal(goal_bl, timeout=0.0)
        arm.wait_for_motion_done()
        if arm_goal_res:
            arm.send_gripper_goal('open')

            door_frame_robot0 = self._robot.tf_buffer.transform(door.frame_points[0], self._robot.base_link_frame)
            door_frame_robot1 = self._robot.tf_buffer.transform(door.frame_points[1], self._robot.base_link_frame)
            x1 = door_frame_robot0.vector.x()
            y1 = door_frame_robot0.vector.y()
            x2 = door_frame_robot1.vector.x()
            y2 = door_frame_robot1.vector.y()
            x = (x1 + x2) / 2.0
            y = (y1 + y2) / 2.0

            self._robot.base.force_drive(0.1, y / (x / 0.1), 0, x / 0.1)
            arm.reset()
            arm.wait_for_motion_done()
            return 'succeeded'
        else:
            return 'failed'


class PullDoorOpen(smach.State):
    def __init__(self, robot, door_des, arm_des):
        """

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self._robot = robot
        self._door_des = door_des
        self._arm_des = arm_des

    def execute(self, userdata=None):
        door = self._door_des.resolve()
        # ToDo: hier moet je het nieuwe x, y coord bepalen achter het middelpunt vd deur
        arm = self._arm_des.resolve()
        result = self._robot.base.force_drive(-0.1, 0, 0, 2.0)
        rospy.sleep(2.0)
        arm.send_gripper_goal('open')
        # Move back and to the left
        next_pose_framestamped = FrameStamped(kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(-0.05, 0.0, 0.15)),
                                              rospy.Time.now(),
                                              "hand_palm_link")
        goal_bl = self._robot.tf_buffer.transform(next_pose_framestamped, self._robot.base_link_frame)

        arm.send_goal(goal_bl, timeout=0.0)
        arm.wait_for_motion_done()
        # Move forward and twist
        next_pose_framestamped = FrameStamped(kdl.Frame(kdl.Rotation.RPY(0.0, 1.0, 0.0), kdl.Vector(0.1, 0.0, 0.0)),
                                              rospy.Time.now(),
                                              "hand_palm_link")
        goal_bl = self._robot.tf_buffer.transform(next_pose_framestamped, self._robot.base_link_frame)

        arm.send_goal(goal_bl, timeout=0.0)
        arm.wait_for_motion_done()

        # ToDo: not sure if this is necessary
        self._robot.base.force_drive(-0.1, 0, 0, 2.0)

        return "succeeded"


# class NavigateThroughDoor(smach.State):
#     def __init__(self, robot, door_des):
#         """

#         :param robot: robot object
#         """
#         smach.State.__init__(self, outcomes=['succeeded', 'failed'])

#         self._robot = robot
#         self._door_des = door_des

#     def execute(self, userdata=None):
#         door = self._door_des.resolve()
#         door_points = door.frame_points["value"]

#        door_point_1_map = self._robot.tf_buffer.transform(door_points0], self._robot.base_link_frame)
#         door_point_2_map = self._robot.tf_buffer.transform(door_points[1], self._robot.base_link_frame)
#         x1 = door_frame_robot0.vector.x()
#         y1 = door_frame_robot0.vector.y()
#         x2 = door_frame_robot1.vector.x()
#         y2 = door_frame_robot1.vector.y()
#         x = (x1 + x2) / 2.0
#         y = (y1 + y2) / 2.0

#         self._robot.base.force_drive(0.1, y/(x/0.1), 0, x/0.1)

#         # Not sure what this does, test it!
#         if door.hinge_direction["value"] == "left":
#             th_vel = -0.2
#         elif door.hinge_direction["value"] == "right":
#             th_vel = 0.2
#         else:
#             arm.reset()
#             self._robot.head.reset()
#             return "succeeded"
#         self._robot.base.force_drive(0.0, 0.0, th_vel, 2.0)
#         rospy.sleep(2.0)
#         arm.reset()
#         self._robot.head.reset()
#         return "succeeded"


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
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        with self:
            smach.StateMachine.add(
                "NAVIGATE_TO_DOOR",
                NavigateToSymbolic(
                    robot,
                    {door_designator: before_area},
                    door_designator,
                ),
                transitions={
                    "arrived": "SAY_DOOR_OPEN",
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


if __name__ == "__main__":
    from robot_skills import get_robot

    rospy.init_node("josja_faalt")

    hero = get_robot('hero')
    door = hero.ed.get_entity(id="door")
    door_des = Designator(Door(door))
    test = OpenDoor(hero, door_des)
    test.execute()
