# ROS
import smach

# TU/e Robotics
from robot_skills.robot import Robot
from robot_skills.arm.gripper import GripperState
from robot_smach_states.human_interaction.human_interaction import Say
from robot_smach_states.navigation import NavigateToPose, ForceDrive, NavigateToWaypoint
from robot_smach_states.manipulation import SetGripper
from robot_smach_states.util.designators import ArmDesignator
from robot_smach_states.util.designators.ed_designators import EntityByIdDesignator


class SetArmPose(smach.State):
    """Set Arm Pose"""
    def __init__(self, robot, joint_positions):
        super().__init__(outcomes=["Done", "Failed"])

        self._robot = robot
        self._joints = joint_positions

        self._arm = self._robot.get_arm()

    def execute(self, userdate=None):
        status = self._arm._arm._send_joint_trajectory([self._joints])
        if status:
            return "Done"

        return "Failed"


class PushObject(smach.StateMachine):
    def __init__(self, robot: Robot, x: float, y: float, theta: float = 3.14, gdx: float = 0.8, gdy: float = 0.0,
                 final_waypoint='final_pose'):
        """
        State machine for pushing objects

        :param robot: robot API object
        :param x: x component of obstacle position
        :param y: y component of obstacle position
        :param theta: yaw component of obstacle position
        :param gdx: grasping pose x distance from obstacles
        :param gdy: grasping pose y distance from obstacles
        """
        super().__init__(outcomes=["succeeded", "failed"])

        self.robot = robot
        self.arm = ArmDesignator(self.robot)
        self.original_pose_frame = self.robot.base.get_location().frame

        self.op_x = self.original_pose_frame.p.x()
        self.op_y = self.original_pose_frame.p.y()
        self.op_theta = self.original_pose_frame.M.GetRPY()[2]
        self.waypoint = EntityByIdDesignator(self.robot, final_waypoint)

        # TODO: Retrieve joint positions using world model and MoveIt!
        self.obstacle_clearance_joint_positions = [0.3, -2.3, 0, 0.9, -1.57]

        with self:
            self.add("DRIVE_TO_OBSTACLE",
                     NavigateToPose(self.robot, x - gdx, y - gdy, theta),
                     transitions={"arrived": "SAY_PUSH_OBJECT",
                                  "unreachable": "FAIL_AT_ORIGINAL_POSE",
                                  "goal_not_defined": "FAIL_AT_ORIGINAL_POSE"}
                     )

            smach.StateMachine.add(
                "SAY_PUSH_OBJECT",
                Say(robot, "I do hope this table gets out of my way!"),
                transitions={"spoken": "MOVE_TO_GRASPING_POSITION"}
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

            # TODO: Replace hard coded values to calculation based upon grasping pose and obstacle position
            self.add("GRASP_TOUCH_OBSTACLE",
                     ForceDrive(self.robot, 0.14, 0, 0, 2),
                     transitions={"done": "CLOSE_GRIPPER_FOR_GRASP_TOUCH"}
                     )

            self.add("CLOSE_GRIPPER_FOR_GRASP_TOUCH",
                     SetGripper(self.robot, self.arm, gripperstate=GripperState.CLOSE),
                     transitions={"succeeded": "MOVE_OBSTACLE",
                                  "failed": "FAIL_AT_ORIGINAL_POSE"}
                     )

            # TODO: Replace hard coded values to calculation based on costmap
            self.add("MOVE_OBSTACLE",
                     ForceDrive(self.robot, -0.05, 0.15, 0.05, 5),
                     transitions={"done": "OPEN_GRIPPER_FOR_RELEASE"}
                     )

            self.add("OPEN_GRIPPER_FOR_RELEASE",
                     SetGripper(self.robot, self.arm, gripperstate=GripperState.OPEN),
                     transitions={"succeeded": "MOVE_AWAY_FROM_OBSTACLE",
                                  "failed": "FAIL_AT_ORIGINAL_POSE"}
                     )

            self.add("MOVE_AWAY_FROM_OBSTACLE",
                     ForceDrive(self.robot, -0.1, 0.0, 0.0, 2),
                     transitions={"done": "RESET_ARM_POSE"}
                     )

            self.add("RESET_ARM_POSE",
                     SetArmPose(self.robot, [0, 0, 0, 0, 0]),
                     transitions={"Done": "SUCCEED_AT_ORIGINAL_POSE",
                                  "Failed": "FAIL_AT_ORIGINAL_POSE"}
                     )

            self.add("FAIL_AT_ORIGINAL_POSE",
                     NavigateToPose(self.robot, self.op_x,
                                    self.op_y, self.op_theta),
                     transitions={"arrived": "failed",
                                  "unreachable": "failed",
                                  "goal_not_defined": "failed"}
                     )
            #
            # self.add("SUCCEED_AT_ORIGINAL_POSE",
            #          NavigateToPose(self.robot, self.op_x,
            #                         self.op_y, self.op_theta),
            #          transitions={"arrived": "succeeded",
            #                       "unreachable": "failed",
            #                       "goal_not_defined": "failed"}

            self.add("SUCCEED_AT_ORIGINAL_POSE",
                     NavigateToWaypoint(self.robot, self.waypoint),
                     transitions={"arrived": "succeeded",
                                  "unreachable": "failed",
                                  "goal_not_defined": "failed"}
                     )
