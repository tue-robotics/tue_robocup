# ROS
import smach


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
        self.robot.speech.speak("Yeehah")
        return "done"
