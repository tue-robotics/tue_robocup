# ROS
import os
import rospy
import smach


class ShowImageState(smach.State):
    """
    This state allows images to be shown on the hero display the package and the path to the image in the package
    """

    def __init__(self, robot, image_filename, seconds=5.0):
        """

        :param robot: the robot object
        :param image_filename: string describing the path to the image (absolute path incl. file extension)
        """
        super(ShowImageState, self).__init__(outcomes=['succeeded', 'failed'])
        self._filename = image_filename
        self._robot = robot
        self._seconds = seconds

    def execute(self, ud=None):
        if not os.path.exists(self._filename):
            rospy.logerr("Failed to display image: image does not exist")
            return 'failed'
        self._robot.hmi.show_image(self._filename, self._seconds)
        return 'succeeded'
