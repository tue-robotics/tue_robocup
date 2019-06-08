from robot_skills import robot, api, arms, base, ebutton, head, ears, lights, perception, speech, torso, world_model_ed

import rospy


class Hero(robot.Robot):
    """docstring for Hero"""
    def __init__(self, wait_services=False):
        super(Hero, self).__init__(robot_name="hero", wait_services=wait_services)

        self._ignored_parts = ["leftArm", "torso", "spindle", "head"]

        self.add_body_part('base', base.Base(self.robot_name, self.tf_listener))
        self.add_body_part('torso', torso.Torso(self.robot_name, self.tf_listener))

        self.add_arm_part('leftArm', arms.Arm(self.robot_name, self.tf_listener, side="left"))

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

        self.add_body_part('ebutton', ebutton.EButton(self.robot_name, self.tf_listener))

        # Reasoning/world modeling
        self.add_body_part('ed', world_model_ed.ED(self.robot_name, self.tf_listener))

        #rename joint names
        self.parts['leftArm'].joint_names = self.parts['leftArm'].load_param('skills/arm/joint_names')

        # These don't work for HSR because (then) Toyota's diagnostics aggregator makes the robot go into error somehow
        for arm in self.arms.itervalues():
            arm.unsubscribe_hardware_status()
        for arm in self.arms.itervalues():
            arm._operational = True

        self.configure()

    def move_to_inspect_pose(self, inspect_target):
        # calculate the arm_lift_link which must be sent
        z_over = 0.3  # height the robot should look over the surface
        z_hh = 0.9  # height of the robots head at z_arm=0
        torso_to_arm_ratio = 2  # motion of arm/motion of torso
        z_head = inspect_target.z() + z_over

        # check whether moving the arm is necessary
        if z_head < 1.2:
            rospy.loginfo('Entity is low enough. we dont need to move the arm')
            return True

        # saturate the arm lift goal
        z_arm = (z_head - z_hh) * torso_to_arm_ratio
        if z_arm > 0.69:
            z_arm = 0.69
            rospy.logwarn('Warning: looking at excessively high surface')
        elif z_arm < 0.0:
            z_arm = 0.0
            rospy.logwarn('Surface is low enough, we dont need to move the arm.')

        arm = self.get_arm()

        # turn the robot
        rotation = 1.57
        rotation_speed = 1
        rotation_duration = rotation / rotation_speed
        if arm.has_joint_goal('arm_out_of_way'):
            pose = arm._arm.default_configurations['arm_out_of_way']
            pose[0] = z_arm
            arm._arm._send_joint_trajectory([pose])
        else:
            rospy.logwarn('Warning: robot does not have an "arm_out_of_way" joint goal')
            return False
        self.base.force_drive(0, 0, rotation_speed, rotation_duration)
        arm.wait_for_motion_done()
        return True


if __name__ == "__main__":
    rospy.init_node("hero")

    import doctest
    doctest.testmod()
