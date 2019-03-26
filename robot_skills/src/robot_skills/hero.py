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
                                                               "/hero/head_rgbd_sensor/project_2d_to_3d"))
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


if __name__ == "__main__":
    rospy.init_node("hero")

    import doctest
    doctest.testmod()
