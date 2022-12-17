import rospy
import smach

from robot_smach_states.util.designators import check_type


class ShowImage(smach.State):
    """
    This state allows images to be shown on the robot display
    """
    def __init__(self, robot, filename, duration=5.0):
        """
        Constructor

        :param robot: the robot object
        :param filename: string or designator describing the path to the image (absolute path incl. file extension)
        :param duration: Duration for which the image should be shown in seconds
        """
        super().__init__(outcomes=["succeeded", "failed"])

        self._robot = robot
        check_type(filename, str)
        self._filename = filename
        self._duration = duration

    def execute(self, ud=None):
        filename = self._filename.resolve() if hasattr(self._filename, "resolve") else self._filename
        if filename is None:
            rospy.logerr(f"Could not resolve a filename from {self._filename}")
            return "failed"

        self._robot.hmi.show_image(filename, self._duration)
        return "succeeded"
