# System
import math

# Third party
import rospy

# TU/e Robotics
from robot_skills import api, base, ebutton, head, ears, lights, perception, picovoice, robot, speech, \
    topological_planner, torso, world_model_ed
from robot_skills.arm import arms, force_sensor, gripper, handover_detector, gripper_position_detector
from robot_skills.simulation import is_sim_mode, SimEButton, SimPicoVoice


class Hero(robot.Robot):
    """Hero"""

    def __init__(self, connection_timeout=robot.DEFAULT_CONNECTION_TIMEOUT):
        """
        Constructor

        :param connection_timeout: timeout to wait for ROS connections
        :type connection_timeout: Optional[float]
        """
        super().__init__(robot_name="hero", base_link_frame="base_link", connection_timeout=connection_timeout)

        self._ignored_parts = ["leftArm", "torso", "spindle", "head"]

        self.add_body_part('base', base.Base(self.robot_name, self.tf_buffer))

        arm_joint_names = rospy.get_param('/' + self.robot_name + '/skills/arm_center/joint_names')
        self.add_body_part('torso',
                           torso.Torso(self.robot_name, self.tf_buffer, self.get_joint_states, arm_joint_names))

        # add hero's arm
        hero_arm = arms.Arm(self.robot_name, self.tf_buffer, self.get_joint_states, "arm_center")
        hero_arm.add_part('force_sensor', force_sensor.ForceSensor(self.robot_name, self.tf_buffer,
                                                                   "/" + self.robot_name + "/wrist_wrench/raw"))
        hero_arm.add_part('gripper', gripper.ParrallelGripper(self.robot_name, self.tf_buffer, 'gripper'))
        hero_arm.add_part('handover_detector',
                          handover_detector.HandoverDetector(self.robot_name, self.tf_buffer, 'center'))

        hero_arm.add_part('gripper_position_detector',
                          gripper_position_detector.GripperPositionDetector(self.robot_name, self.tf_buffer,
                                                                            "/" + self.robot_name + "/joint_states"))
        self.add_arm_part('arm_center', hero_arm)

        self.add_body_part('head', head.Head(self.robot_name, self.tf_buffer))
        self.add_body_part('perception', perception.Perception(self.robot_name, self.tf_buffer,
                                                               "/hero/head_rgbd_sensor/rgb/image_raw",
                                                               "/hero/head_rgbd_sensor/project_2d_to_3d",
                                                               camera_base_ns='hero/head_rgbd_sensor'))

        # Human Robot Interaction
        self.add_body_part(
            'lights', lights.Lights(
                self.robot_name, self.tf_buffer, '/hero/command_status_led_rgb'
            )
        )
        self.add_body_part('speech', speech.Speech(self.robot_name, self.tf_buffer,
                                                   lambda: self.lights.set_color_rgba_msg(lights.SPEAKING),
                                                   lambda: self.lights.set_color_rgba_msg(lights.RESET)))
        self.add_body_part('hmi', api.Api(self.robot_name, self.tf_buffer,
                                          lambda: self.lights.set_color_rgba_msg(lights.LISTENING),
                                          lambda: self.lights.set_color_rgba_msg(lights.RESET)))
        self.add_body_part('ears', ears.Ears(self.robot_name, self.tf_buffer,
                                             lambda: self.lights.set_color_rgba_msg(lights.LISTENING),
                                             lambda: self.lights.set_color_rgba_msg(lights.RESET)))
        picovoice_class = SimPicoVoice if is_sim_mode() else picovoice.PicoVoice
        self.add_body_part("picovoice", picovoice_class(self.robot_name, self.tf_buffer))

        ebutton_class = SimEButton if is_sim_mode() else ebutton.EButton
        self.add_body_part('ebutton', ebutton_class(self.robot_name, self.tf_buffer, topic="/hero/runstop_button"))

        # Reasoning/world modeling
        self.add_body_part('ed', world_model_ed.ED(self.robot_name, self.tf_buffer))

        # Action planning
        # self.add_body_part(
        #    'topological_planner', topological_planner.TopologicalPlanner(self.robot_name, self.tf_buffer)
        # )

        # These don't work for HSR because (then) Toyota's diagnostics aggregator makes the robot go into error somehow
        for part in self.parts.values():
            part.unsubscribe_hardware_status()
            part._operational = True

        # verify joint goal required for posing
        assert 'arm_out_of_way' in self.parts['arm_center'].default_configurations, \
            "arm_out_of_way joint goal is not defined in {}_describtion skills.yaml".format(self.robot_name)
        # parameters for posing
        self.z_over = 0.4  # height the robot should look over the surface
        self.z_hh = 0.9  # height of the robots head at z_arm=0
        self.torso_to_arm_ratio = 2.0  # motion of arm/motion of torso

        self.configure()

    def move_to_inspect_pose(self, inspect_target):
        """
        This poses the robot for an inspect.

        :param inspect_target: kdl.Frame with the pose of the entity to be inspected.
        :return: boolean, false if something went wrong.
        """
        # calculate the arm_lift_link which must be sent
        z_head = inspect_target.z() + self.z_over

        # check whether moving the arm is necessary
        if z_head < 1.3:
            rospy.logdebug("Entity is low enough. we don't need to move the arm")
            return True

        # saturate the arm lift goal
        z_arm = (z_head - self.z_hh) * self.torso_to_arm_ratio
        z_arm = min(0.69, max(z_arm, 0.0))  # arm_lift_joint limit

        arm = self.get_arm(required_goals=['arm_out_of_way'])

        # noinspection PyProtectedMember
        pose = arm._arm.default_configurations['arm_out_of_way']
        pose[0] = z_arm
        # noinspection PyProtectedMember
        arm._arm._send_joint_trajectory([pose])

        self.base.turn_towards(inspect_target.x(), inspect_target.y(), "map", 1.57)
        arm.wait_for_motion_done()
        self.base.wait_for_motion_done()
        return True

    def move_to_hmi_pose(self):
        """
        This poses the robot for conversations.

        :return: None
        """
        arm = self.get_arm(required_goals=['arm_out_of_way'])

        rotation = 1.57
        rotation_speed = 1
        rotation_duration = rotation / rotation_speed
        arm.send_joint_goal('arm_out_of_way', 0.0)
        self.base.force_drive(0, 0, rotation_speed, rotation_duration)
        arm.wait_for_motion_done()

    def move_to_pregrasp_pose(self, arm, grasp_target):
        """
        This poses the robot for an inspect.

        :param arm: PublicArm to use for grasping the target, must have joint trajectory 'prepare_grasp'
        :param grasp_target: kdl.Frame with the pose of the entity to be grasp.
        :return: boolean, false if something went wrong.
        """
        # calculate the arm_lift_link which must be sent
        z_head = grasp_target.z() + self.z_over

        if z_head < 1.3:
            # we dont need to do stupid stuff
            arm.send_joint_trajectory('prepare_grasp')
            return True

        # saturate the arm lift goal
        z_arm = (z_head - self.z_hh) * self.torso_to_arm_ratio
        z_arm = min(0.69, max(z_arm, 0.0))

        # noinspection PyProtectedMember
        pose = arm._arm.default_trajectories['prepare_grasp']
        pose[1][0] = z_arm
        # noinspection PyProtectedMember
        arm._arm._send_joint_trajectory(pose)

        angle_offset = -math.atan2(arm.base_offset.y(), arm.base_offset.x())
        self.base.turn_towards(grasp_target.x(), grasp_target.y(), "map", angle_offset)
        arm.wait_for_motion_done()
        self.base.wait_for_motion_done()
        return True

    def go_to_driving_pose(self):
        arm = self.get_arm(required_goals=['carrying_pose'])
        arm.send_joint_goal('carrying_pose')
