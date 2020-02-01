# ROS
import smach
from functools import partial

# TU/e Robotics
from robot_smach_states.utility import Initialize
import robot_skills.arms as arms

class English(object):
    HI_MY_NAME_IS = "Hello, my name is {}"
    IM_A_SERVICE_ROBOT = "I am one of the service robots of the Eindhoven University of Technology"
    PURPOSE = "My purpose is to help people in domestic or care environments."
    IM_OMNIDIR = "I have an omnidirectional base instead of legs"
    EXPLAIN_BASE = "With this base, I can instantly move in any direction and turn around whenever I want"
    TWO_ARMS = "I have two arms. These have the dimensions and degrees of freedom of human arms"
    HUMAN_ARMS = "This makes me capable of moving my arms just like you would move your arms."
    END_OF_ARMS = "At the end of my arms, I have two grippers with which I can grasp objects"
    GRIPPERS = "My grippers can be opened and closed when I need to."
    TORSO = "My arms are mounted on a moveable torso. This way, I can grasp higher and lower"
    HEAD = "As a head, I have a 3D camera. I use this to detect and recognize objects and people"
    CAMERA = "My 3D camera is mounted on top of my torso and I can move my camera just like a human head."
    TWO_LRFs = "Furthermore, I have two laser range finders to help me to see where I am"
    LRF_LOCS = "One laser is mounted on my torso and the other one is at the bottom of my base"
    MICROPHONE = "Finally, I have a microphone on my head so that I can hear what you are saying"
    END_OF_INTRO = "Thank you for your attention, I hope that you like what you see and have a nice day."

class Dutch(object):
    HI_MY_NAME_IS = "Hallo, mijn naam is {}"
    IM_A_SERVICE_ROBOT = "Ik ben een van de zorgrobots van de Technische Universiteit Eindhoven"
    PURPOSE = "Mijn doel is mensen te helpen in huis- en zorgomgevingen"
    IM_OMNIDIR = "In plaats van benen heb ik een omni-directioneel onderstel"
    EXPLAIN_BASE = "Met dit onderstel kan ik direct alle kanten op bewegen en omdraaien wanneer ik wil"
    TWO_ARMS = "Ik heb ook twee armen. Deze hebben dezelfde afmetingen en bewegingsmogelijkheden als mensenarmen"
    HUMAN_ARMS = "Dit stelt mij in staat om mijn armen net zo te bewegen als jij de jouwe"
    END_OF_ARMS = "Aan het eind van mijn armen zitten grijpers waarmee ik dingen kan vastpakken"
    GRIPPERS = "Mijn grijpers kunnen open en dicht als ik dat wil"
    TORSO = "Mijn armen zitten aan een beweegbare torso, zodat ik hoger en lager kan pakken"
    HEAD = "Als hoofd heb ik een 3D camera. Dat gebruik ik om mensen en dingen te herkennen"
    CAMERA = "Mijn hoofd zit bovenop mijn torso en ik kan rondkijken net als een mens"
    TWO_LRFs = "Verder heb ik twee lezer afstandsmeters, waarmee ik beter kan zien waar ik ben" # laser = lezer :-)
    LRF_LOCS = "1 lezer zit op mijn torso en de andere onderaan mijn onderstel"
    MICROPHONE = "Als laatste heb ik een microfoon waarmee ik kan horen wat mensen zeggen"
    END_OF_INTRO = "Bedankt voor uw aandacht, ik hoop dat je het leuk vond en ik wens je nog een fijne dag"


class Presentation(smach.State):
    """ Smach state to have the robot present itself to an audience as a demo """
    def __init__(self, robot, language='en'):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=["done", "preempted"])

        self.robot = robot

        left_trajectories = ["wave_front", "show_gripper", "point_to_laser"]
        self.left_arm = self.robot.get_arm(required_gripper_types=[arms.GripperTypes.GRASPING],
                                           required_trajectories=left_trajectories,
                                           required_arm_name='left')

        right_trajectories = ["show_gripper", "point_to_kinect"]
        self.right_arm = self.robot.get_arm(required_gripper_types=[arms.GripperTypes.GRASPING],
                                             required_trajectories=right_trajectories,
                                             required_arm_name='right')

        self.language = language
        self.trans = {"en":English, "nl":Dutch}[language]
        if self.language=="nl":
            self.voice="marjolijn"
        else:
            self.voice="kyle"

    def execute(self, userdata=None):
        """ Execute function

        :param userdata:
        :return:
        """

        # List to store all functions
        function_list = []

        # Add all functions to function_list by using partial to prevent the function from executing before it is callled in the for loop

        # Introduction
        function_list.append(partial(self.robot.speech.speak, self.trans.HI_MY_NAME_IS.format(self.robot.robot_name), language=self.language, voice=self.voice, block=True))
        function_list.append(partial(self.robot.speech.speak, self.trans.IM_A_SERVICE_ROBOT, language=self.language, voice=self.voice, block=True))
        function_list.append(partial(self.robot.speech.speak, self.trans.PURPOSE, language=self.language, voice=self.voice, block=True))

        # Base
        function_list.append(partial(self.robot.speech.speak, self.trans.IM_OMNIDIR, language=self.language, voice=self.voice, block=True ))
        function_list.append(partial(self.robot.speech.speak, self.trans.EXPLAIN_BASE, language=self.language, voice=self.voice, block=False))
        function_list.append(partial(self.robot.base.force_drive, 0.1, 0, 0, 1.0))      # Forward
        function_list.append(partial(self.robot.base.force_drive, 0, 0.1, 0, 1.0))      # Left
        function_list.append(partial(self.robot.base.force_drive, -0.1, 0, 0, 1.0))     # Backwards
        function_list.append(partial(self.robot.base.force_drive, 0, -0.1, 0, 1.0))     # Right
        function_list.append(partial(self.robot.base.force_drive, 0, 0, 1.0, 6.28))     # Turn around

        # Arms
        if self.robot.robot_name == "amigo":
            function_list.append(partial(self.robot.speech.speak, self.trans.TWO_ARMS, language=self.language, voice=self.voice, block=False))
            function_list.append(partial(self.robot.speech.speak, self.trans.HUMAN_ARMS, language=self.language, voice=self.voice, block=False))
            if self.left_arm is not None:
                function_list.append(partial(self.left_arm.send_joint_trajectory, "wave_front"))
            function_list.append(partial(self.robot.speech.speak, self.trans.END_OF_ARMS, language=self.language, voice=self.voice, block=False))
            function_list.append(partial(self.robot.speech.speak, self.trans.GRIPPERS, language=self.language, voice=self.voice, block=False))
            if self.left_arm is not None:
                function_list.append(partial(self.left_arm.send_joint_trajectory, "show_gripper"))
            if self.right_arm is not None:
                function_list.append(partial(self.right_arm.send_joint_trajectory, "show_gripper"))
            if self.left_arm is not None:
                function_list.append(partial(self.left_arm.send_gripper_goal, "open"))
                function_list.append(partial(self.left_arm.send_gripper_goal, "close"))
            if self.right_arm is not None:
                function_list.append(partial(self.right_arm.send_gripper_goal, "open"))
                function_list.append(partial(self.right_arm.send_gripper_goal, "close"))
            if self.left_arm is not None:
                function_list.append(partial(self.left_arm.reset))
            if self.right_arm is not None:
                function_list.append(partial(self.right_arm.reset))
                function_list.append(partial(self.right_arm.wait_for_motion_done))

        # Torso
        if self.robot.robot_name == "amigo":
            function_list.append(partial(self.robot.speech.speak, self.trans.TORSO, language=self.language, voice=self.voice, block=False))
            function_list.append(partial(self.robot.torso.medium))
            function_list.append(partial(self.robot.torso.wait_for_motion_done, 5.0))
            function_list.append(partial(self.robot.torso.reset))
            function_list.append(partial(self.robot.torso.wait_for_motion_done, 5.0))

        # Kinect
        function_list.append(partial(self.robot.speech.speak, self.trans.HEAD, language=self.language, voice=self.voice, block=False))
        #Removed this part to increase the tempo of the presentation
        # function_list.append(partial(self.right_arm.send_joint_trajectory, "point_to_kinect"))
        function_list.append(partial(self.robot.speech.speak, self.trans.CAMERA, language=self.language, voice=self.voice, block=False))
        function_list.append(partial(self.robot.head.look_at_hand, "right"))
        function_list.append(partial(self.robot.head.wait_for_motion_done))
        function_list.append(partial(self.robot.head.look_at_hand, "left"))
        function_list.append(partial(self.robot.head.wait_for_motion_done))
        function_list.append(partial(self.robot.head.reset))
        function_list.append(partial(self.robot.head.wait_for_motion_done))

        # Lasers
        function_list.append(partial(self.robot.speech.speak, self.trans.TWO_LRFs, language=self.language, voice=self.voice, block=True))
        function_list.append(partial(self.robot.speech.speak, self.trans.LRF_LOCS, language=self.language, voice=self.voice, block=False))
        #Removed this part to increase the tempo of the presentation
        #if self.left_arm is not None:
        #    function_list.append(partial(self.left_arm.send_joint_trajectory, "point_to_laser"))

        # Microphone
        function_list.append(partial(self.robot.speech.speak, self.trans.MICROPHONE, language=self.language, voice=self.voice, block=True))

        # Final
        function_list.append(partial(self.robot.speech.speak, self.trans.END_OF_INTRO, language=self.language, voice=self.voice, block=True))

        # Loop through all functions while checking for preempt between every function
        # When preempt, reset all parts and wait for the motion to be done before preempting
        for function in function_list:
            function()
            if self.preempt_requested():
                self.robot.speech.speak("Sorry, but I have to stop my introduction")
                if self.left_arm is not None:
                    self.left_arm.reset()
                if self.right_arm is not None:
                    self.right_arm.reset()
                if self.left_arm is not None:
                    self.left_arm.send_gripper_goal("close")
                if self.right_arm is not None:
                    self.right_arm.send_gripper_goal("close")
                self.robot.torso.reset()
                self.robot.head.reset()

                if self.left_arm is not None:
                    self.left_arm.wait_for_motion_done()
                if self.right_arm is not None:
                    self.right_arm.wait_for_motion_done()
                self.robot.torso.wait_for_motion_done()
                self.robot.head.wait_for_motion_done()
                return 'preempted'

        return "done"


class PresentationMachine(smach.StateMachine):
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
