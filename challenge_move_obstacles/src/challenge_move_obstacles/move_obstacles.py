#!/usr/bin/env python

import rospy

import smach
from robot_smach_states.navigation import NavigateToPose, ForceDrive
from robot_smach_states.manipulation import SetGripper
from robot_skills.arm.gripper import GripperState


class SetArmPose(smach.State):
    """Set Arm Pose"""
    def __init__(self, robot, joint_positions):
        super().__init__(outcomes=["Done", "Failed"])

        self._robot = robot
        self._joints = joint_positions

        self._arm = self._robot.get_arm()

    def execute(self, userdate=None):
        status = self.arm._arm._send_joint_trajectory([self._joints])
        if status:
            return "Done"

        return "Failed"


class MoveObstacles(smach.StateMachine):
    """MoveObstacles StateMachine"""
    def __init__(self, robot: str, x: float, y: float, gdx: float = 0.0, gdy: float = 0.0):
        """
        Initialize MoveObstacles state machine

        :param robot: robot api object
        :param x: x component of obstacle position
        :param y: y component of obstacle position
        :param gdx: grasping pose x distance from obstacles
        :param gdy: grasping pose y distance from obstacles
        """
        super().__init__(outcomes=["succeeded", "failed", "abort"])
        self.robot = robot
        self.arm = self.robot.get_arm()
        self.original_pose_frame = self.robot.base.get_location().frame

        self.op_x = self.original_pose_frame.p[0]
        self.op_y = self.original_pose_frame.p[1]
        self.op_theta = self.original_pose_frame.M.GetRPY()[2]

        self.obstacle_clearance_joint_positions = [0.3, -2.3, 0, 0.9, -1.57]

        with self:
            self.add("DRIVE_TO_OBSTACLE",
                     NavigateToPose(self.robot, x - gdx, y - gdy, 0),
                     transitions={"arrived": "MOVE_TO_GRASPING_POSITION",
                                  "unreachable": "FAIL_AT_ORIGINAL_POSE",
                                  "goal_not_defined": "FAIL_AT_ORIGINAL_POSE"}
                     )

            self.add("MOVE_TO_GRASPING_POSITION",
                     SetArmPose(
                         self.robot, self.obstacle_clearance_joint_positions),
                     transitions={"Done": "OPEN_GRIPPER_FOR_GRASP_TOUCH",
                                  "Failed": "FAIL_AT_ORIGINAL_POSE"}
                     )

            self.add("OPEN_GRIPPER_FOR_GRASP_TOUCH",
                     SetGripper(self.robot, self.arm, gripperstate=GripperState.OPEN),
                     transitions={"succeeded": "GRASP_TOUCH_OBSTACLE",
                                  "failed": "FAIL_AT_ORIGINAL_POSE"}
                     )

            self.add("GRASP_TOUCH_OBSTACLE",
                     ForceDrive(self.robot, 0.14, 0, 0, 2),
                     transitions={"done": "CLOSE_GRIPPER_FOR_GRASP_TOUCH"}
                     )

            self.add("CLOSE_GRIPPER_FOR_GRASP_TOUCH",
                     SetGripper(self.robot, self.arm, gripperstate=GripperState.CLOSE),
                     transitions={"succeeded": "MOVE_OBSTACLE",
                                  "failed": "FAIL_AT_ORIGINAL_POSE"}
                     )

            self.add("MOVE_OBSTACLE",
                     ForceDrive(self.robot, 0.05, 0.1, 0.05, 2),
                     transitions={"done": "OPEN_GRIPPER_FOR_RELEASE"}
                     )

            self.add("OPEN_GRIPPER_FOR_RELEASE",
                     SetGripper(self.robot, self.arm, gripperstate=GripperState.OPEN),
                     transitions={"succeeded": "SUCCEED_AT_ORIGINAL_POSE",
                                  "failed": "FAIL_AT_ORIGINAL_POSE"}
                     )

            self.add("FAIL_AT_ORIGINAL_POSE",
                     NavigateToPose(self.robot, self.op_x,
                                    self.op_y, self.op_theta),
                     transitions={"arrived": "failed",
                                  "unreachable": "failed",
                                  "goal_not_defined": "failed"}
                     )

            self.add("SUCCEED_AT_ORIGINAL_POSE",
                     NavigateToPose(self.robot, self.op_x,
                                    self.op_y, self.op_theta),
                     transitions={"arrived": "succeeded",
                                  "unreachable": "failed",
                                  "goal_not_defined": "failed"}
                     )

