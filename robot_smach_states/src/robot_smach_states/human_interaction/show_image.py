#ROS
import os
import rospkg
import smach


class ShowImageState(smach.State):
    """
    This state allows images to be shown on the hero display the package and the path to the image in the package
    """

    def __init__(self, robot, package_name, path_to_image_in_package, seconds=5):
        """

        :param robot: the robot object
        :param package_name: string describing the package in which the image is located
        :param path_to_image_in_package: string describing the path to the image in the package specified before
        """
        super(ShowImageState, self).__init__(outcomes=['succeeded', 'failed'])
        self._path = os.path.join(rospkg.RosPack().get_path(package_name), path_to_image_in_package)
        self._robot = robot
        self._seconds = seconds
        if not os.path.exists(self._path):
            raise ValueError("Image path {} does not exist".format(self._path))

    def execute(self, ud=None):
        if not self._path:
            return 'failed'
        self._robot.hmi.show_image(self._path, self._seconds)
        return 'succeeded'
