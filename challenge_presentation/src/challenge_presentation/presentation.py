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
        # Introduction
        self.robot.speech.speak("Hi, my name is {}".format(self.robot.robot_name), block=True)
        self.robot.speech.speak("I am one of the care robots of the Eindhoven University of Technology", block=True)
        self.robot.speech.speak("My purpose is to help people in domestic or care environments.", block=True)

        # Base
        self.robot.speech.speak("I have a omnidirectionaly base", block=True)
        self.robot.speech.speak("With this, I can instantly move in any direction and turn around whenever I want", block=False)
        self.robot.base.force_drive(0.1, 0, 0, 1.0)  # Forward
        self.robot.base.force_drive(0, 0.1, 0, 1.0)  # Left
        self.robot.base.force_drive(-0.1, 0, 0, 1.0)  # Backwards
        self.robot.base.force_drive(0, -0.1, 0, 1.0)  # Right
        self.robot.base.force_drive(0, 0, 1.0, 6.28)  # Turn around

        # Arms
        self.robot.speech.speak("I have two arms. These have the dimensions and degrees of freedom of human arms",
                                block=False)
        self.robot.speech.speak("So I can move my arms like you would move your arms", block=False)
        self.robot.leftArm.send_joint_trajectory("wave_front")
        self.robot.speech.speak("At the end of my arms, I have two grippers with which I can grasp objects", block=False)
        self.robot.speech.speak("My grippers can be opened and closed when I need to.", block=False)
        self.robot.leftArm._send_joint_trajectory([[0, 0, 0, 1.7, 0, 0, 0]])
        self.robot.rightArm._send_joint_trajectory([[0, 0, 0, 1.7, 0, 0, 0]])
        self.robot.leftArm.send_gripper_goal("open")
        self.robot.leftArm.send_gripper_goal("close")
        self.robot.rightArm.send_gripper_goal("open")
        self.robot.rightArm.send_gripper_goal("close")
        # Torso
        self.robot.speech.speak("My arms are mounted on a moveable torso. This way, I can grasp higher and lower",
                                block=False)
        self.robot.torso.low()
        self.robot.torso.wait_for_motion_done(5.0)
        self.robot.torso.reset()
        self.robot.torso.wait_for_motion_done(5.0)

        # Kinect
        self.robot.speech.speak("As a head, I have a 3D camera. I use this to detect and recognize objects and people",
                                block=False)
        self.robot.rightArm.send_joint_trajectory("point_to_kinect")
        self.robot.speech.speak("My 3D camera is mounted on top of my torso and I can move my camera just like a human head.", block=False)

        # Lasers
        self.robot.speech.speak("Furthermore, I have two laser range finders to help me to see where I am", block=True)
        self.robot.speech.speak("One laser is mounted on my torso and the other one is at the bottom of my base",
                                block=False)
        self.robot.leftArm.send_joint_trajectory("point_to_laser")

        # Microphone
        self.robot.speech.speak("Finally, I have a microphone on my head so that I can hear what you're saying")

        # Final
        self.robot.speech.speak("This was my introduction, I hope that you like what you see and please enjoy the presentation.")

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
