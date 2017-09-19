# ROS
import smach

# TU/e Robotics
from robot_smach_states.utility import Initialize


class Presentation(smach.State):
    """ Smach state to have the robot present itself to an audience as a demo """
    def __init__(self, robot):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=["done"])

        self.robot = robot

    def execute(self, userdata=None):
        """ Execute function

        :param userdata:
        :return:
        """
        # ToDo: here we can have the robot do awesome stuff
        # self.robot.speech.speak("Yeehah")
        # Introduction
        self.robot.speech.speak("Hi, my name is {}".format(self.robot.robot_name), block=True)
        self.robot.speech.speak("I am one of the care robots of the Eindhoven University of Technology", block=True)
        self.robot.speech.speak("My purpose is to help people in domestic or care environments.", block=True)

        # Base
        self.robot.speech.speak("I have a omnidirectionaly base", block=True)
        self.robot.speech.speak("With this, I can instantly move in any direction", block=False)
        self.robot.base.force_drive(0.1, 0, 0, 1.0)  # Forward
        self.robot.base.force_drive(0, 0.1, 0, 1.0)  # Left
        self.robot.base.force_drive(-0.1, 0, 0, 1.0)  # Backwards
        self.robot.base.force_drive(0, -0.1, 0, 1.0)  # Right
        self.robot.base.force_drive(0, 0, 1.0, 6.28)  # Turn around

        # Arms
        self.robot.speech.speak("I have two arms. These have the dimensions and degrees of freedom of human arms",
                                block=False)
        self.robot.speech.speak("With these, I can grasp objects")
        # ToDo: replace
        self.robot.leftArm._send_joint_trajectory([[-0.1, 1.4, 0.5, 1.6, 0, 0, 0],
                                                   [-0.1, 1.4, -0.5, 1.6, 0, 0, 0],
                                                   [-0.1, 1.4, 0.5, 1.6, 0, 0, 0],
                                                   [-0.1, -0.2, 0.2, 0.8, 0.0, 0.0, 0.0]])

        # Torso
        self.robot.speech.speak("My arms are mounted on a moveable torso. This way, I can grasp higher and lower",
                                block=False)
        self.robot.torso.low()
        self.robot.torso.wait_for_motion_done(5.0)
        self.robot.torso.reset()

        # Kinect
        self.robot.speech.speak("As a head, I have a 3D camera. I use this to detect and recognize objects and people",
                                block=False)
        # ToDo: replace
        self.robot.rightArm._send_joint_trajectory([[-0.1, 1.4, 0.5, 1.6, 0, 0, 0],
                                                   [-0.1, -0.2, 0.2, 0.8, 0.0, 0.0, 0.0]])

        # Lasers
        self.robot.speech.speak("Furthermore, I have two laser range finders to help me to see where I am", block=False)
        # ToDo: replace
        self.robot.leftArm._send_joint_trajectory([[-1.2, 0.3, 1.0, 2.2, 0, 0.5, 0],
                                                   [-0.1, -0.2, 0.2, 0.8, 0.0, 0.0, 0.0]])

        # Microphone
        self.robot.speech.speak("Finally, I have a microphone on my head so that I can hear what you're saying")

        # ToDo: uitsmijter

        return "done"


class PresentationMachine(smach.StateMachine):
    def __init__(self, robot):
            """ Contains the Initialize state and the Presentation state
            :param robot: Robot to use
            :return:
            """
            smach.StateMachine.__init__(self, outcomes=["done", "aborted"])

            with self:
                smach.StateMachine.add("INITIALIZE", Initialize(robot=robot),
                                       transitions={"initialized": "PRESENT",
                                                    "abort": "aborted"})

                smach.StateMachine.add("PRESENT", Presentation(robot=robot),
                                       transitions={"done": "done"})
