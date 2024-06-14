# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.arm.gripper import GripperState
from robot_skills.robot import Robot
from robot_smach_states.navigation import NavigateToPose, ForceDrive
from robot_smach_states.manipulation import SetGripper, ArmToJointConfig
from robot_smach_states.util.designators import ArmDesignator


class SetArmPose(smach.State):
    """Set Arm Pose"""
    def __init__(self, robot, joint_positions):
        super().__init__(outcomes=["done", "failed"])

        self._robot = robot
        self._joints = joint_positions
        self._arm = self._robot.get_arm()

    def execute(self, userdate=None):
        # noinspection PyProtectedMember
        status = self._arm._arm._send_joint_trajectory([self._joints])
        if status:
            return "done"

        return "failed"


class MoveObstacles(smach.StateMachine):
    def __init__(self, robot: Robot, x: float, y: float, gdx: float = 0.0, gdy: float = 0.0):
        """
        MoveObstacles state machine: drives to an obstacles, grasps it and moves it out of the way

        :param robot: robot api object
        :param x: x component of obstacle position
        :param y: y component of obstacle position
        :param gdx: grasping pose x distance from obstacles
        :param gdy: grasping pose y distance from obstacles
        """
        super().__init__(outcomes=["succeeded", "failed", "abort"])
        rospy.loginfo(f"MoveObstacles: x: {x}, y: {y}, gdx: {gdx}, gdy: {gdy}")

        arm = ArmDesignator(robot)
        original_pose_frame = robot.base.get_location().frame

        op_x = original_pose_frame.p[0]
        op_y = original_pose_frame.p[1]
        op_theta = original_pose_frame.M.GetRPY()[2]
        obstacle_clearance_joint_positions = [0.3, -2.3, 0, 0.9, -1.57]

        with self:
            self.add(
                "DRIVE_TO_OBSTACLE",
                NavigateToPose(robot, x - gdx, y - gdy, 3.14),
                transitions={"arrived": "MOVE_TO_GRASPING_POSITION",
                             "unreachable": "FAIL_AT_ORIGINAL_POSE",
                             "goal_not_defined": "FAIL_AT_ORIGINAL_POSE"}
            )

            self.add(
                "MOVE_TO_GRASPING_POSITION",
                SetArmPose(robot, obstacle_clearance_joint_positions),
                transitions={"done": "OPEN_GRIPPER_FOR_GRASP_TOUCH",
                             "failed": "FAIL_AT_ORIGINAL_POSE"}
            )

            self.add(
                "OPEN_GRIPPER_FOR_GRASP_TOUCH",
                SetGripper(robot, arm, gripperstate=GripperState.OPEN),
                transitions={"succeeded": "GRASP_TOUCH_OBSTACLE",
                             "failed": "FAIL_AT_ORIGINAL_POSE"}
            )

            self.add(
                "GRASP_TOUCH_OBSTACLE",
                ForceDrive(robot, 0.14, 0, 0, 2),
                transitions={"done": "CLOSE_GRIPPER_FOR_GRASP_TOUCH"}
            )

            self.add(
                "CLOSE_GRIPPER_FOR_GRASP_TOUCH",
                SetGripper(robot, arm, gripperstate=GripperState.CLOSE),
                transitions={"succeeded": "MOVE_OBSTACLE",
                             "failed": "FAIL_AT_ORIGINAL_POSE"}
            )

            self.add(
                "MOVE_OBSTACLE",
                ForceDrive(robot, -0.05, 0.15, 0.05, 5),
                transitions={"done": "OPEN_GRIPPER_FOR_RELEASE"}
            )

            self.add(
                "OPEN_GRIPPER_FOR_RELEASE",
                SetGripper(robot, arm, gripperstate=GripperState.OPEN),
                transitions={"succeeded": "MOVE_AWAY_FROM_OBSTACLE",
                             "failed": "FAIL_AT_ORIGINAL_POSE"}
            )

            self.add(
                "MOVE_AWAY_FROM_OBSTACLE",
                ForceDrive(robot, -0.1, 0.0, 0.0, 2),
                transitions={"done": "RESET_ARM_POSE"}
            )

            self.add(
                "RESET_ARM_POSE",
                SetArmPose(robot, [0, 0, 0, 0, 0]),
                transitions={"done": "SUCCEED_AT_ORIGINAL_POSE",
                             "failed": "FAIL_AT_ORIGINAL_POSE"}
            )

            self.add(
                "FAIL_AT_ORIGINAL_POSE",
                NavigateToPose(robot, op_x, op_y, op_theta),
                transitions={"arrived": "failed",
                             "unreachable": "failed",
                             "goal_not_defined": "failed"}
            )

            self.add(
                "SUCCEED_AT_ORIGINAL_POSE",
                NavigateToPose(robot, op_x, op_y, op_theta),
                transitions={"arrived": "succeeded",
                             "unreachable": "failed",
                             "goal_not_defined": "failed"}
            )
