# TU/e Robotics
from robot_skills import api, base, ears, ebutton, head, lights, perception, robot, sound_source_localisation, speech, \
    torso, world_model_ed
from robot_skills.simulation import is_sim_mode, SimEButton


class Sergio(robot.Robot):
    """
    Sergio
    """
    # noinspection PyUnresolvedReferences
    def __init__(self, wait_services=False):
        """
        Constructor

        :param wait_services: Not supported anymore by robot class
        """
        super(Sergio, self).__init__(robot_name="sergio", wait_services=wait_services)

        self._ignored_parts = ["leftArm", "rightArm", "torso", "spindle", "head"]

        self.add_body_part('base', base.Base(self.robot_name, self.tf_buffer))
        self.add_body_part('torso', torso.Torso(self.robot_name, self.tf_buffer, self.get_joint_states))

        # Add arms (replace the '[[arm_name]]' and '[[side_name]]' strings with actual arm names.)
        # self.add_arm_part('[[arm name]]', arms.Arm(self.robot_name, self.tf_buffer, side='[[side name]]'))

        self.add_body_part('head', head.Head(self.robot_name, self.tf_buffer))
        self.add_body_part('perception', perception.Perception(self.robot_name, self.tf_buffer))
        self.add_body_part('ssl', sound_source_localisation.SSL(self.robot_name, self.tf_buffer))

        # Human Robot Interaction
        self.add_body_part('lights', lights.TueLights(self.robot_name, self.tf_buffer))
        self.add_body_part('speech', speech.TueSpeech(self.robot_name, self.tf_buffer,
                                                      lambda: self.lights.set_color_rgba_msg(lights.SPEAKING),
                                                      lambda: self.lights.set_color_rgba_msg(lights.RESET)))
        self.add_body_part('hmi', api.Api(self.robot_name, self.tf_buffer,
                                          lambda: self.lights.set_color_rgba_msg(lights.LISTENING),
                                          lambda: self.lights.set_color_rgba_msg(lights.RESET)))
        self.add_body_part('ears', ears.Ears(self.robot_name, self.tf_buffer,
                                             lambda: self.lights.set_color_rgba_msg(lights.LISTENING),
                                             lambda: self.lights.set_color_rgba_msg(lights.RESET)))

        ebutton_class = SimEButton if is_sim_mode() else ebutton.EButton
        self.add_body_part('ebutton', ebutton_class(self.robot_name, self.tf_buffer))

        # Reasoning/world modeling
        self.add_body_part('ed', world_model_ed.ED(self.robot_name, self.tf_buffer))

        self.configure()

    def move_to_pregrasp_pose(self, arm, grasp_target):
        """
        This poses the robot for an inspect.

        :param arm: PublicArm with an available joint_trajectory 'prepare_grasp' to use for grasping the target
        :param grasp_target: kdl.Frame with the pose of the entity to be grasp.
        :return: boolean, false if something went wrong.
        """

        arm.send_joint_trajectory('prepare_grasp', timeout=0)
        return True
