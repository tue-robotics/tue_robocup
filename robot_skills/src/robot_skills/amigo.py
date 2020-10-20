from __future__ import print_function

from robot_skills import robot, api, arms, base, ebutton, head, ears, lights, perception, speech, \
    sound_source_localisation, torso, world_model_ed
from .simulation import is_sim_mode, SimEButton


class Amigo(robot.Robot):
    """
    Amigo
    """
    def __init__(self, dontInclude=None, wait_services=False):
        """
        Constructor

        :param dontInclude: Not supported anymore
        :param wait_services: Not supported anymore by robot class
        """
        if dontInclude is None:
            dontInclude = []
        super(Amigo, self).__init__(robot_name="amigo", wait_services=wait_services)

        self.add_body_part('base', base.Base(self.robot_name, self.tf_listener))
        self.add_body_part('torso', torso.Torso(self.robot_name, self.tf_listener, self.get_joint_states))

        self.add_arm_part('leftArm', arms.Arm(self.robot_name, self.tf_listener, self.get_joint_states, side="left"))
        self.add_arm_part('rightArm', arms.Arm(self.robot_name, self.tf_listener, self.get_joint_states, side="right"))

        self.add_body_part('head', head.Head(self.robot_name, self.tf_listener))
        self.add_body_part('perception', perception.Perception(self.robot_name, self.tf_listener))
        self.add_body_part('ssl', sound_source_localisation.SSL(self.robot_name, self.tf_listener))

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
        self.add_body_part('ebutton', ebutton_class(self.robot_name, self.tf_listener))

        # Reasoning/world modeling
        self.add_body_part('ed', world_model_ed.ED(self.robot_name, self.tf_listener))

        self.configure()

    def move_to_pregrasp_pose(self, arm, grasp_target):
        """
        This poses the robot for an inspect.

        :param arm: PublicArm with an available joint_trajectory 'prepare_grasp' to use for grasping the target
        :param grasp_target: kdl.Frame with the pose of the entity to be grasped.
        :return: boolean, false if something went wrong.
        """

        arm.send_joint_trajectory('prepare_grasp', timeout=0)
        return True
