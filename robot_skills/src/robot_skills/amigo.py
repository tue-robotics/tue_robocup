from robot_skills import robot, api, arms, base, ebutton, head, ears, lights, perception, speech, ssl, torso,\
    world_model_ed


class Amigo(robot.Robot):
    """docstring for Amigo"""
    def __init__(self, dontInclude=None, wait_services=False):
        if dontInclude is None:
            dontInclude = []
        super(Amigo, self).__init__(robot_name="amigo", wait_services=wait_services)

        self.add_body_part('base', base.Base(self.robot_name, self.tf_listener))
        self.add_body_part('torso', torso.Torso(self.robot_name, self.tf_listener))

        self.add_arm_part('leftArm', arms.Arm(self.robot_name, self.tf_listener, side="left"))
        self.add_arm_part('rightArm', arms.Arm(self.robot_name, self.tf_listener, side="right"))

        self.add_body_part('head', head.Head(self.robot_name, self.tf_listener))
        self.add_body_part('perception', perception.Perception(self.robot_name, self.tf_listener))
        self.add_body_part('ssl', ssl.SSL(self.robot_name, self.tf_listener))

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

        self.configure()
