# ROS
import rospy
import smach
import actionlib
import math
import numpy
import PyKDL as kdl
from geometry_msgs.msg import PointStamped, Point

# TU/e Robotics
from robot_skills.robot import Robot
from robot_skills.arm import arms
import robot_skills.util.kdl_conversions as kdl_con
from cb_base_navigation_msgs.msg import OrientationConstraint, PositionConstraint

# Robot Smach States
from robot_smach_states.navigation import NavigateTo
from robot_smach_states.human_interaction.human_interaction import Say
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToSymbolic
from robot_smach_states.util.designators import Designator, EdEntityDesignator, UnoccupiedArmDesignator
from tue_msgs.msg import LocateDoorHandleAction, LocateDoorHandleGoal

class OpenDoor(smach.StateMachine):
    def __init__(self, robot):
        """
        Enter the arena by force driving through the door
        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=["done", "failed"])
        # self.robot = robot
        self._door = Door()
        point1 = PointStamped()
        point1.header.frame_id = "map"
        point1.point.x = 2.14
        point1.point.y = -0.9
        point1.point.z = 0.0
        point2 = PointStamped()
        point2.header.frame_id = "map"
        point2.point.x = 2.14
        point2.point.y = -1.7
        point2.point.z = 0.0
        self._door.frame_points = [point1, point2]
        self._door.direction = "outward"

        handle_loc = PointStamped()
        handle_loc.header.frame_id = "map"
        handle_loc.point.x = 2.28
        handle_loc.point.y = -0.96
        handle_loc.point.z = 1.065
        self._door._handle.location = handle_loc
        self._door._handle.direction = 'down'
        self._door._handle.grasp_orientation = "horizontal"

        self._arm_des = UnoccupiedArmDesignator(robot, {"required_goals": ["reset", "handover"],
                                                        "force_sensor_required": True,
                                                        "required_gripper_types": [arms.GripperTypes.GRASPING]})

        with self:

            smach.StateMachine.add('NAVIGATE_TO_HANDLE', NavigateToHandle(robot, self._door, self._arm_des),
                                   transitions={'unreachable': 'UPDATE_HANDLE_LOCATION',
                                                'arrived': 'UPDATE_HANDLE_LOCATION',
                                                'goal_not_defined': 'UPDATE_HANDLE_LOCATION'})

            smach.StateMachine.add("UPDATE_HANDLE_LOCATION",
                                   LocateHandleVision(robot, self._door),
                                   transitions={"done": "GRASP_HANDLE_MOTION_PLANNING"})

            smach.StateMachine.add("GRASP_HANDLE_MOTION_PLANNING",
                                   GraspHandleMotionPlanningSkill(robot, self._door, self._arm_des),
                                   transitions={"done": "UNLATCH_HANDLE"})

            smach.StateMachine.add("UNLATCH_HANDLE",
                                   UnlatchHandle(robot, self._door, self._arm_des),
                                   transitions={"done": "OPEN_DOOR_TO_INIT"})

            smach.StateMachine.add("OPEN_DOOR_TO_INIT",
                                   OpenDoorAndDrive(robot, self._door, self._arm_des),
                                   transitions={"done": "done"})


class NavigateToHandle(NavigateTo):
    def __init__(self, robot, door, arm_des):
        super(NavigateToHandle, self).__init__(robot, self.generateConstraint)

        self._robot = robot
        self._door = door
        self._arm_des = arm_des

    def generateConstraint(self):
        arm = self._arm_des.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "done"

        handle_point = self._door.handle.location["value"]
        angle_offset =-math.atan2(arm.base_offset.y(), arm.base_offset.x())
        radius = 0.75*math.hypot(arm.base_offset.x(), arm.base_offset.y())

        handle_point_map = self._robot.tf_buffer.transform(handle_point, "map")
        x = handle_point_map.point.x
        y = handle_point_map.point.y

        # Outer radius
        ro = "(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, radius+0.075)
        ri = "(x-%f)^2+(y-%f)^2 > %f^2"%(x, y, radius-0.075)
        pc = PositionConstraint(constraint=ri+" and "+ro, frame="/map")
        oc = OrientationConstraint(look_at=Point(x, y, 0.0), frame="/map", angle_offset=angle_offset)

        return pc, oc


class LocateHandleVision(smach.State):
    def __init__(self, robot, door):
        """
        Constructor
        :param robot: robot object
        :type robot: robot
        """
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot
        self._door = door
        self._locate_handle_client = actionlib.SimpleActionClient('/locate_handle', LocateDoorHandleAction)
        self._locate_handle_client.wait_for_server()

    def execute(self, userdata=None):
        handle_p = self._door._handle.location["value"]
        self._robot.head.look_at_point(kdl_con.VectorStamped(x=handle_p.point.x, y=handle_p.point.y,
                                                             z=handle_p.point.z), timeout=0.0)
        self._robot.head.wait_for_motion_done()

        goal = LocateDoorHandleGoal()
        goal.door_frame_point1 = self._door._handle.location["value"]
        goal.door_frame_point2 = self._door.frame_points["value"][1]
        rospy.loginfo("Goal is {}".format(goal))
        self._locate_handle_client.send_goal(goal)
        self._locate_handle_client.wait_for_result(rospy.Duration.from_sec(5.0))
        state = self._locate_handle_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            result = self._locate_handle_client.get_result()
            rospy.loginfo("Result is {}".format(result))
            rospy.loginfo("Successfully executed perception action.")
            # edge_p_1 = self._robot.tf_buffer.transform(result.handle_edge_point1, "map")
            # edge_p_2 = self._robot.tf_buffer.transform(result.handle_edge_point2, "map")
            # self._door.handle.edge_points["value"] = [edge_p_1, edge_p_2]
            handle_loc = PointStamped()
            handle_loc.header.frame_id = result.handle_edge_point1.header.frame_id
            handle_loc.point.x = numpy.average([result.handle_edge_point1.point.x, result.handle_edge_point2.point.x])
            handle_loc.point.y = numpy.average([result.handle_edge_point1.point.y, result.handle_edge_point2.point.y])
            handle_loc.point.z = numpy.average([result.handle_edge_point1.point.z, result.handle_edge_point2.point.z])
            handle_loc_map = self._robot.tf_buffer.transform(handle_loc, "map")
            self._door._handle.location = handle_loc

            return "done"
        else:
            rospy.loginfo("Failed perception action.")
            return "done"


class GraspHandleMotionPlanningSkill(smach.State):
    def __init__(self, robot, door, arm_des):
        """
        Wait till the average distance in front of the robot is bigger than 1 meter. Only point in front of the middle
        are taken into account
        :param robot: robot object
        :param timeout: timeout for waiting till the door is opened
        """
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot
        self._door = door
        self._arm_des = arm_des

    def execute(self, userdata=None):
        arm = self._arm_des.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "done"

        arm.send_gripper_goal("open")

        handle_point = self._door.handle.location["value"]
        print("point is {}".format(self._door.handle.location["value"]))

        handle_orien = -1.57
        align_door = 0.0 # -0.3
        if self._door.handle.grasp_orientation["value"] == "vertical":
            handle_orien = 0.0

        handle_framestamped = kdl_con.FrameStamped(kdl.Frame(kdl.Rotation.RPY(handle_orien, 0.0, align_door),
                                                             kdl.Vector(handle_point.point.x, handle_point.point.y,
                                                                        handle_point.point.z)),
                                                   frame_id=handle_point.header.frame_id)
        goal_bl = handle_framestamped.projectToFrame(self._robot.robot_name + '/base_link',
                                                     tf_buffer=self._robot.tf_buffer)

        # ToDo: not sure about the orientation here, should be thoroughly tested.
        # move to in front of handle
        goal_bl.frame.p.x(goal_bl.frame.p.x() - 0.08)  # Retract 10 cm
        goal_bl.frame.p.y(goal_bl.frame.p.y() + 0.01)  # Retract 10 cm

        goal_bl.frame.M = kdl.Rotation.RPY(handle_orien, 0.0, align_door)  # Update the roll
        result = arm.send_goal(goal_bl, timeout=0.0)
        arm.wait_for_motion_done()


        # move to handle
        handle_framestamped = kdl_con.FrameStamped(kdl.Frame(kdl.Rotation.RPY(handle_orien, 0.0, align_door),
                                                             kdl.Vector(handle_point.point.x, handle_point.point.y,
                                                                        handle_point.point.z)),
                                                   frame_id=handle_point.header.frame_id)
        goal_bl = handle_framestamped.projectToFrame(self._robot.robot_name + '/base_link',
                                                     tf_buffer=self._robot.tf_buffer)

        goal_bl.frame.p.x(goal_bl.frame.p.x() - 0.03)  # Retract
        goal_bl.frame.p.y(goal_bl.frame.p.y() + 0.02)  # Retract

        goal_bl.frame.M = kdl.Rotation.RPY(handle_orien, 0.0, align_door)  # Update the roll
        # result = arm.send_goal(goal_bl, timeout=0.0)
        # felt_edge = False
        # try:
            # arm._arm.force_sensor.wait_for_edge_up(5.0)  # timeout is 5 seconds
            # arm.cancel_goals()
            # felt_edge = True
            # goal_bl = goal_bl.projectToFrame(self._robot.robot_name + '/base_link', tf_listener=self._robot.tf_listener)
            # goal_bl.frame.p.x(goal_bl.frame.p.x() - 0.03)  # Retract 3 cm
            # arm.send_goal(goal_bl, timeout=0.0)
        # except TimeOutException:  # no force detected
            # rospy.logwarn("Did not detect force from the handle, continuing anyway.")

        if result or felt_edge:
            rospy.loginfo("Successfully executed action.")
            arm.send_gripper_goal('close', max_torque=1.0)
            return "done"
        else:
            rospy.loginfo("Failed action.")
            return "done"


class UnlatchHandle(smach.State):
    def __init__(self, robot, door, arm_des):
        """
        Wait till the average distance in front of the robot is bigger than 1 meter. Only point in front of the middle
        are taken into account
        :param robot: robot object
        :param timeout: timeout for waiting till the door is opened
        """
        smach.State.__init__(self, outcomes=["done"], output_keys=["action_out"])

        # self._tmc_robot = Robot()
        # self._whole_body = self._tmc_robot.try_get('whole_body')
        # self._move_dist = 0.05
        self._move_dist = 0.03
        self._robot = robot
        self._door = door
        self._arm_des = arm_des

    def execute(self, userdata):
        arm = self._arm_des.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return None

        # you want the roll in the gripper frame to be complient, but not the pitch and yaw.
        # ToDo: do sth with whole_body.end_effector_frame = 'base_link' ?
        # self._whole_body.impedance_config = 'compliance_soft'
        arm.send_gripper_goal('close', max_torque=1.0)
        move_vector = (0.0, 0.0, self._move_dist)
        # current_pose = self._robot.tf_buffer.transform(handle_point, "map")

        current_pose = self._robot.tf_buffer.lookup_transform('hero/base_link', 'grippoint', rospy.Time(0))

        orientation = kdl.Rotation.Quaternion(current_pose.transform.rotation.x, current_pose.transform.rotation.y, current_pose.transform.rotation.z, current_pose.transform.rotation.w)
        (curr_r, curr_p, curr_y) = orientation.GetRPY()
        # curr_r, curr_p, curr_y = euler_from_quaternion(current_pose[1])
        if self._door.handle.direction["value"] == "down":
            next_z = current_pose.transform.translation.z - self._move_dist
            # next_x, next_y, next_z = tuple(map(operator.sub, current_pose[0], move_vector))
        elif self._door.handle.direction["value"] == "up":
            next_z = current_pose.transform.translation.z + self._move_dist
            # next_x, next_y, next_z = tuple(map(operator.add, current_pose[0], move_vector))
        else:
            rospy.logerr("Handle direction ({}) not recognized.".format(self._door.handle.direction["value"]))
            return "done"

        next_pose = kdl_con.kdl_frame_stamped_from_XYZRPY(current_pose.transform.translation.x, current_pose.transform.translation.y, next_z, curr_r, curr_p, curr_y,
                                                          "hero/base_link")
        #     next_x, next_y, next_z = tuple(map(operator.sub, [current_pose.transform.translation], [move_vector]))
        # elif self._door.handle.direction["value"] == "up":
        #     next_x, next_y, next_z = tuple(map(operator.add, [current_pose.transform.translation], [move_vector]))
        # else:
        #     rospy.logerr("Handle direction ({}) not recognized.".format(self._door.handle.direction["value"]))
        #     return "done"

        # next_pose = kdl_con.kdl_frame_stamped_from_XYZRPY(next_x, next_y, next_z, curr_r, curr_p, curr_y,
        #                                                   "/" + self._robot.robot_name + "/base_link")

        result = arm.send_goal(next_pose)
        if result:
            rospy.loginfo("Successfully executed action.")
            return "done"
        rospy.loginfo("Failed action.")
        return "done"


class OpenDoorAndDrive(smach.State):
    def __init__(self, robot, door, arm_des):
        """
        Wait till the average distance in front of the robot is bigger than 1 meter. Only point in front of the middle
        are taken into account
        :param robot: robot object
        :param timeout: timeout for waiting till the door is opened
        """
        smach.State.__init__(self, outcomes=["done"])

        self._move_dist = 0.15
        self._robot = robot
        self._door = door
        self._arm_des = arm_des

    def execute(self, userdata=None):

        arm = self._arm_des.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return None
        result = False
        if self._door.direction["value"] == "outward":
            # you want the roll in the gripper frame to be complient, but not the pitch and yaw.
            # ToDo: do sth with whole_body.end_effector_frame = 'base_link' ?
            # self._whole_body.impedance_config = 'compliance_soft'
            next_pose_framestamped = kdl_con.FrameStamped(kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0),
                                                                    kdl.Vector(0.15, 0.0, 0.0)),
                                                          frame_id="grippoint")

            # Lift
            goal_bl = next_pose_framestamped.projectToFrame('hero/base_link', tf_buffer=self._robot.tf_buffer)

            rospy.loginfo('Start opening door')

            arm_goal_res = arm.send_goal(goal_bl, timeout=0.0)
            arm.wait_for_motion_done()
            if arm_goal_res:
                arm.send_gripper_goal('open')

            door_points = self._door.frame_points["value"]

            door_point_1_map = self._robot.tf_buffer.transform(door_points[0], "base_link")
            door_point_2_map = self._robot.tf_buffer.transform(door_points[1], "base_link")
            x1 = door_point_1_map.point.x
            y1 = door_point_1_map.point.y
            x2 = door_point_2_map.point.x
            y2 = door_point_2_map.point.y
            x = (x1 + x2) / 2.0
            y = (y1 + y2) / 2.0

            self._robot.base.force_drive(0.1, y/(x/0.1), 0, x/0.1)
            result = True

        elif self._door.direction["value"] == "inward":
            # ToDo: hier moet je het nieuwe x, y coord bepalen achter het middelpunt vd deur

            result = self._robot.base.force_drive(-0.1, 0, 0, 2.0)
            rospy.sleep(2.0)
            arm.send_gripper_goal('open')
            # Move back and to the left
            next_pose_framestamped = kdl_con.FrameStamped(kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0),
                                                                    kdl.Vector(-0.05, 0.0, 0.15)),
                                                          frame_id="grippoint")
            goal_bl = next_pose_framestamped.projectToFrame('hero/base_link',
                                                            tf_buffer=self._robot.tf_buffer)

            arm.send_goal(goal_bl, timeout=0.0)
            arm.wait_for_motion_done()
            # Move forward and twist
            next_pose_framestamped = kdl_con.FrameStamped(kdl.Frame(kdl.Rotation.RPY(0.0, 1.0, 0.0),
                                                                    kdl.Vector(0.1, 0.0, 0.0)),
                                                          frame_id="grippoint")
            goal_bl = next_pose_framestamped.projectToFrame('hero/base_link',
                                                            tf_buffer=self._robot.tf_buffer)

            arm.send_goal(goal_bl, timeout=0.0)
            arm.wait_for_motion_done()

            result = self._robot.base.force_drive(-0.1, 0, 0, 2.0)
            rospy.sleep(2.0)

            # ToDo: check if moving the arm is cleaner
            # next_x, next_y, next_z = tuple(map(operator.sub, current_pose[0], move_vector))
        else:
            rospy.logerr("Door direction ({}) not recognized.".format(self._door.handle.direction["value"]))
            arm.reset()
            self._robot.head.reset()
            return "done"

        if result:
            if self._door.hinge_direction["value"] == "left":
                th_vel = -0.2
            elif self._door.hinge_direction["value"] == "right":
                th_vel = 0.2
            else:
                arm.reset()
                self._robot.head.reset()
                return "done"
            self._robot.base.force_drive(0.0, 0.0, th_vel, 2.0)
            rospy.sleep(2.0)
            arm.reset()
            self._robot.head.reset()
            rospy.loginfo("Successfully executed action.")

            return "done"
        rospy.loginfo("Failed action.")
        arm.reset()
        self._robot.head.reset()
        return "done"


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

class Door(object):
    def __init__(self):
        """
        Door object that contains all information about the door
        """
        self._handle = Handle()
        self._hinge_direction = {"known": False, "value": None}
        self._frame_points = {"known": True, "value": []}
        self._direction = {"known": False, "value": None}

    @property
    def handle(self):
        return self._handle

    @property
    def hinge_direction(self):
        return self._hinge_direction

    @property
    def frame_points(self):
        return self._frame_points

    @property
    def direction(self):
        return self._direction

    @hinge_direction.setter
    def hinge_direction(self, hinge_direction):
        """
        :param hinge_direction: describes the direction by which to open the door
        :type hinge_direction: str (either "left" or "right")
        """
        self._hinge_direction["known"] = True
        self._hinge_direction["value"] = hinge_direction

    @frame_points.setter
    def frame_points(self, frame_points):
        """
        :param frame_points: describes the two bottom end points of the door frame
        :type frame_points: list of two PointStamped
        """
        self._frame_points["known"] = True
        self._frame_points["value"] = frame_points

    @direction.setter
    def direction(self, direction):
        """
        :param direction: describes the direction by which to open the door
        :type direction: str (either "outward" or "inward")
        """
        self._direction["known"] = True
        self._direction["value"] = direction


class Handle(object):
    def __init__(self):
        """
        Handle object
        """
        self._location = {"known": False, "value": PointStamped()}
        self._direction = {"known": False, "value": None}
        self._grasp_orientation = {"known": False, "value": None}

    @property
    def location(self):
        return self._location

    @property
    def direction(self):
        return self._direction

    @property
    def grasp_orientation(self):
        return self._grasp_orientation

    @location.setter
    def location(self, point):
        """
        :param point: point in 3d space that estimates the handle location
        :type point: PointStamped()
        """
        self._location["known"] = True
        self._location["value"] = point

    @direction.setter
    def direction(self, direction):
        """
        :param direction: describes the direction by which to unlatch the handle
        :type direction: str (either "up" or "down")
        """
        self._direction["known"] = True
        self._direction["value"] = direction

    @grasp_orientation.setter
    def grasp_orientation(self, orientation):
        """
        :param orientation: describes the orientation the gripper should have to grasp the handle
        :type orientation: str (either "horizontal" or "vertical")
        """
        self._grasp_orientation["known"] = True
        self._grasp_orientation["value"] = orientation
