from robot_skills import robot, api, arms, base, ebutton, head, ears, lights, perception, speech, torso, world_model_ed
from .simulation import is_sim_mode, SimEButton

import rospy


class Hero(robot.Robot):
    """docstring for Hero"""
    def __init__(self, wait_services=False):
        super(Hero, self).__init__(robot_name="hero", wait_services=wait_services)

        self._ignored_parts = ["leftArm", "torso", "spindle", "head"]

        self.add_body_part('base', base.Base(self.robot_name, self.tf_listener))
        self.add_body_part('torso', torso.Torso(self.robot_name, self.tf_listener, self.get_joint_states))

        self.add_arm_part(
            'leftArm',
            arms.ForceSensingArm(self.robot_name, self.tf_listener, self.get_joint_states, "left")
        )

        self.add_body_part('head', head.Head(self.robot_name, self.tf_listener))
        self.add_body_part('perception', perception.Perception(self.robot_name, self.tf_listener,
                                                               "/hero/head_rgbd_sensor/rgb/image_raw",
                                                               "/hero/head_rgbd_sensor/project_2d_to_3d",
                                                               camera_base_ns='hero/head_rgbd_sensor'))
        # self.add_body_part('ssl', ssl.SSL(self.robot_name, self.tf_listener))

        # Human Robot Interaction
        self.add_body_part('lights', lights.Lights(self.robot_name, self.tf_listener))
        self.add_body_part('speech', speech.Speech(self.robot_name, self.tf_listener,
                                                   lambda: self.lights.set_color_colorRGBA(lights.SPEAKING),
                                                   lambda: self.lights.set_color_colorRGBA(lights.RESET)))
        self.add_body_part('hmi', api.Api(self.robot_name, self.tf_listener,
                                          lambda: self.lights.set_color_colorRGBA(lights.LISTENING),
                                          lambda: self.lights.set_color_colorRGBA(lights.RESET)))
        self.add_body_part('ears', ears.Ears(self.robot_name, self.tf_listener,
                                             lambda: self.lights.set_color_colorRGBA(lights.LISTENING),
                                             lambda: self.lights.set_color_colorRGBA(lights.RESET)))

        ebutton_class = SimEButton if is_sim_mode() else ebutton.EButton
        self.add_body_part('ebutton', ebutton_class(self.robot_name, self.tf_listener, topic="/hero/runstop_button"))

        # Reasoning/world modeling
        self.add_body_part('ed', world_model_ed.ED(self.robot_name, self.tf_listener))

        #rename joint names
        self.parts['leftArm'].joint_names = self.parts['leftArm'].load_param('skills/arm/joint_names')

        # These don't work for HSR because (then) Toyota's diagnostics aggregator makes the robot go into error somehow
        for part in self.parts.itervalues():
            part.unsubscribe_hardware_status()
            part._operational = True

        # verify joint goal required for posing
        assert 'arm_out_of_way' in self.parts['leftArm'].default_configurations,\
            "arm_out_of_way joint goal is not defined in {}_describtion skills.yaml".format(self.robot_name)

        self.configure()

    def move_to_inspect_pose(self, inspect_target):
        """
        This poses the robot for an inspect.

        :param inspect_target: kdl.Frame with the pose of the entity to be inspected.
        :return: boolean, false if something went wrong.
        """
        # calculate the arm_lift_link which must be sent
        z_over = 0.4  # height the robot should look over the surface
        z_hh = 0.9  # height of the robots head at z_arm=0
        torso_to_arm_ratio = 2.0  # motion of arm/motion of torso
        z_head = inspect_target.z() + z_over

        # check whether moving the arm is necessary
        if z_head < 1.1:
            rospy.logdebug("Entity is low enough. we don't need to move the arm")
            return True

        # saturate the arm lift goal
        z_arm = (z_head - z_hh) * torso_to_arm_ratio
        z_arm = min(0.69, max(z_arm, 0.0))  # arm_lift_joint limit

        arm = self.parts['leftArm']

        # turn the robot
        rotation = 1.57
        rotation_speed = 1.0
        rotation_duration = rotation / rotation_speed

        pose = arm.default_configurations['arm_out_of_way']
        pose[0] = z_arm
        arm._send_joint_trajectory([pose])

        self.base.force_drive(0, 0, rotation_speed, rotation_duration)
        arm.wait_for_motion_done()
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
