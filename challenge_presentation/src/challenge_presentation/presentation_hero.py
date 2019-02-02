#!/usr/bin/env python

# ROS
import rospy
import smach
import sys
from functools import partial

# TU/e Robotics
from robot_smach_states.utility import Initialize
from robot_smach_states.util.startup import startup
from robot_skills import arms
from robot_skills.util.kdl_conversions import VectorStamped

class English(object):
    HI_MY_NAME_IS = "Hello, my name is Hero"
    IM_A_SERVICE_ROBOT = "I am one of the service robots of the Eindhoven University of Technology"
    MADE_BY_TOYOTA = "I am a Human Support Robot developed by the Toyota Motor Company."
    PURPOSE = "My purpose is to help people in domestic or care environments."
    IM_OMNIDIR = "I have an omnidirectional base instead of legs"
    EXPLAIN_BASE = "With this base, I can move in any direction and turn around whenever I want"
    ONE_ARM = "I have one arm. It can move along my extendable torso, which allows me to assist people in many ways."
    END_OF_ARM = "At the end of my arm, I have a gripper with which I can grasp objects."
    GRIPPER = "My gripper can be opened and closed when I need it to be."
    GRIPPER_CAMERA = "My gripper has its own camera, so I have perfect hand-eye coordination."
    TORSO = "My arm is mounted on a movable torso. This way, I can grasp higher and lower."
    CAMERA = "On my head, I have a 3D camera. I use this to detect and recognize objects and people."
    SCREEN = "My head also has a screen, which I can use to communicate with people around me."
    HEAD = "I can move my head up and down as well as sideways to look around me."
    LRF = "Furthermore, I have a laser range finder to help me to see where I am."
    LRF_LOCS = "The laser is mounted on top of my base."
    LRF_LOCS2 = "Look it's right there"
    MICROPHONE = "Finally, I have a microphone on my head so that I can hear what you are saying"
    END_OF_INTRO = "Thank you for your attention, I hope you enjoyed my presentation and have a nice day."

class Dutch(object):
    HI_MY_NAME_IS = "Hallo, mijn naam is Hero"
    IM_A_SERVICE_ROBOT = "Ik ben een van de zorgrobots van de Technische Universiteit Eindhoven"
    MADE_BY_TOYOTA = "Ik ben een hulp robot ontwikkeld door Toyota."
    PURPOSE = "Mijn doel is mensen helpen in huis- en zorgomgevingen"
    IM_OMNIDIR = "In plaats van benen heb ik een omni-directioneel onderstel"
    EXPLAIN_BASE = "Met dit onderstel kan ik alle kanten op bewegen en omdraaien wanneer ik wil"
    ONE_ARM = "Ik heb 1 arm. Deze kan ik via mij uitschuifbare nek bewegen, zo kan ik mensen met veel dingen helpen."
    END_OF_ARM = "Aan het eind van mijn arm zit 1 grijper waarmee ik dingen kan vastpakken."
    GRIPPER  = "Mijn grijper kan open en dicht wanneer ik dat wil"
    GRIPPER_CAMERA = "Mijn grijper heeft zijn eigen camera, ik heb dus perfecte hand-oog coordinatie."
    TORSO = "Mijn arm zit vast aan een beweegbare torso, zo kan ik de koekjes van de bovenste plank pakken."
    CAMERA = "Op mijn hoofd heb ik een 3D camera. Deze gebruik ik om objecten en mensen om me heen te herkennen."
    SCREEN = "Mijn hoofd heeft ook een scherm, waarmee ik met mensen kan communiceren."
    HEAD = "Ik kan mijn hoofd naar boven en beneden bewegen, en ook opzij!"
    LRF = "Verder heb ik 1 lezer afstandsmeter, waarmee ik beter kan zien waar ik ben." # laser = lezer :-)
    LRF_LOCS = "Deze lezer zit op mijn onderstel"
    LRF_LOCS2 = "Kijk daar zit ie"
    MICROPHONE = "Als laatste heb ik een microfoon waarmee ik kan horen wat mensen zeggen"
    END_OF_INTRO = "Bedankt voor uw aandacht, ik hoop dat je mijn presentatie leuk vond en ik wens je nog een fijne dag"


class Presentation(smach.State):
    """ Smach state to have the robot present itself to an audience as a demo """
    def __init__(self, robot, language='en'):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=["done", "preempted"])

        self.robot = robot

        trajectories = ["wave_front", "show_gripper", "point_to_laser"]
        self.arm = self.robot.get_arm(required_gripper_types=[arms.GripperTypes.GRASPING],
                                      required_trajectories=trajectories)

        self.language = language
        self.trans = {"en": English, "nl": Dutch}[language]
        if self.language == "nl":
            self.voice = "marjolijn"
        else:
            self.voice = "kyle"

    def execute(self, userdata=None):
        """ Execute function

        :param userdata:
        :return:
        """

        # List to store all functions
        function_list = []

        # Add all functions to function_list by using partial to prevent the function from executing before it is callled in the for loop

        # Introduction
        function_list.append(partial(self.robot.speech.speak, self.trans.HI_MY_NAME_IS, language=self.language,
                                     voice=self.voice, block=True))
        function_list.append(partial(self.robot.speech.speak, self.trans.IM_A_SERVICE_ROBOT, language=self.language,
                                     voice=self.voice, block=True))
        function_list.append(partial(self.robot.speech.speak, self.trans.MADE_BY_TOYOTA, language=self.language,
                                     voice=self.voice, block=True))
        function_list.append(partial(self.robot.speech.speak, self.trans.PURPOSE, language=self.language,
                                     voice=self.voice, block=True))

        # Base
        function_list.append(partial(self.robot.speech.speak, self.trans.IM_OMNIDIR, language=self.language,
                                     voice=self.voice, block=True))
        function_list.append(partial(self.robot.speech.speak, self.trans.EXPLAIN_BASE, language=self.language,
                                     voice=self.voice, block=False))
        function_list.append(partial(self.robot.base.force_drive, 0.1, 0, 0, 1.0))      # Forward
        function_list.append(partial(self.robot.base.force_drive, 0, 0.1, 0, 1.0))      # Left
        function_list.append(partial(self.robot.base.force_drive, -0.1, 0, 0, 1.0))     # Backwards
        function_list.append(partial(self.robot.base.force_drive, 0, -0.1, 0, 1.0))     # Right
        function_list.append(partial(self.robot.base.force_drive, 0, 0, 1.0, 6.28))     # Turn around

        # Arms
        function_list.append(partial(self.robot.speech.speak, self.trans.ONE_ARM, language=self.language,
                                     voice=self.voice, block=False))
        function_list.append(partial(self.arm.send_joint_trajectory, "wave_front"))
        function_list.append(partial(self.robot.speech.speak, self.trans.END_OF_ARM, language=self.language,
                                     voice=self.voice, block=True))
        function_list.append(partial(self.robot.speech.speak, self.trans.GRIPPER, language=self.language,
                                     voice=self.voice, block=False))
        function_list.append(partial(self.arm._send_joint_trajectory, [[0.01, 0.0, 0.0, -1.57, 0.0]]))
        function_list.append(partial(self.arm.send_gripper_goal, "open"))
        function_list.append(partial(self.arm.send_gripper_goal, "close"))
        function_list.append(partial(self.robot.speech.speak, self.trans.GRIPPER_CAMERA, language=self.language,
                                     voice=self.voice, block=True))
        function_list.append(partial(self.arm.reset))
        function_list.append(partial(self.arm.wait_for_motion_done))

        # Torso
        function_list.append(partial(self.robot.speech.speak, self.trans.TORSO, language=self.language,
                                     voice=self.voice, block=False))
        function_list.append(partial(self.robot.torso.medium))
        function_list.append(partial(self.robot.torso.wait_for_motion_done, 5.0))
        function_list.append(partial(self.robot.torso.reset))
        function_list.append(partial(self.robot.torso.wait_for_motion_done, 5.0))

        # RGBD Camera
        function_list.append(partial(self.robot.speech.speak, self.trans.CAMERA, language=self.language,
                                     voice=self.voice, block=True))
        function_list.append(partial(self.robot.speech.speak, self.trans.SCREEN, language=self.language,
                                     voice=self.voice, block=True))
        function_list.append(partial(self.robot.speech.speak, self.trans.HEAD, language=self.language,
                                     voice=self.voice, block=False))
        function_list.append(partial(self.robot.head.look_at_point, VectorStamped(1,1,1.75, frame_id="/hero/base_link"))) # Set nice path for moving head
        function_list.append(partial(self.robot.head.wait_for_motion_done))
        function_list.append(partial(self.robot.head.look_at_point, VectorStamped(1,-1,1.0, frame_id="/hero/base_link"))) # Set nice path for moving head
        function_list.append(partial(self.robot.head.wait_for_motion_done))
        function_list.append(partial(self.robot.head.look_at_point, VectorStamped(-1,0,1.0, frame_id="/hero/base_link"))) # Set nice path for moving head
        function_list.append(partial(self.robot.head.wait_for_motion_done))

        function_list.append(partial(self.robot.head.reset))
        function_list.append(partial(self.robot.head.wait_for_motion_done))

        # Lasers
        function_list.append(partial(self.robot.speech.speak, self.trans.LRF, language=self.language,
                                     voice=self.voice, block=True))
        function_list.append(partial(self.robot.speech.speak, self.trans.LRF_LOCS, language=self.language,
                                     voice=self.voice, block=False))
        function_list.append(partial(self.arm.send_joint_trajectory, "point_to_laser", 20.0)) # Maybe takes too long
        function_list.append(partial(self.arm.wait_for_motion_done))
        function_list.append(partial(self.robot.speech.speak, self.trans.LRF_LOCS2, language=self.language,
                                     voice=self.voice, block=True))
        function_list.append(partial(self.arm.reset))


        # Microphone
        function_list.append(partial(self.robot.speech.speak, self.trans.MICROPHONE, language=self.language,
                                     voice=self.voice, block=True))

        # Final
        function_list.append(partial(self.robot.speech.speak, self.trans.END_OF_INTRO, language=self.language,
                                     voice=self.voice, block=True))

        # Loop through all functions while checking for preempt between every function
        # When preempt, reset all parts and wait for the motion to be done before preempting
        for function in function_list:
            function()
            if self.preempt_requested():
                self.robot.speech.speak("Sorry, but I have to stop my introduction")
                self.arm.reset()
                self.arm.send_gripper_goal("close")
                self.robot.torso.reset()
                self.robot.head.reset()
                self.arm.wait_for_motion_done()
                self.robot.torso.wait_for_motion_done()
                self.robot.head.wait_for_motion_done()
                return 'preempted'

        return "done"


class PresentationMachineHero(smach.StateMachine):
    def __init__(self, robot, language='nl'):
            """ Contains the Initialize state and the Presentation state
            :param robot: Robot to use
            :return:
            """
            smach.StateMachine.__init__(self, outcomes=["done", "aborted", "preempted"])

            with self:
                smach.StateMachine.add("INITIALIZE", Initialize(robot=robot),
                                       transitions={"initialized": "PRESENT",
                                                    "abort": "aborted"})

                smach.StateMachine.add("PRESENT", Presentation(robot=robot, language=language),
                                       transitions={"done": "done", "preempted": "preempted"})

def setup_statemachine(robot):
        sm = smach.StateMachine(outcomes=["done", "aborted", "preempted"])
        with sm:
                smach.StateMachine.add("INITIALIZE", Initialize(robot=robot),
                                       transitions={"initialized": "PRESENT",
                                                    "abort": "aborted"})

                smach.StateMachine.add("PRESENT", Presentation(robot=robot, language="en"),
                                       transitions={"done": "done", "preempted": "preempted"})
                return sm

if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "Please provide robot name as argument."
        exit(1)

    rospy.init_node('test_presentation')
    startup(setup_statemachine, robot_name=robot_name)

