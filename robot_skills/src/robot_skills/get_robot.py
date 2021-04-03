# System
import sys

# ROS
import rospy

# Robot skills
from .robot import Robot

# ToDo: create a decent registration here
ROBOTS = {}
try:
    from amigo_skills import Amigo
    ROBOTS["amigo"] = Amigo
except ImportError:
    pass
try:
    from hero_skills import Hero
    ROBOTS["hero"] = Hero
except ImportError:
    pass
try:
    from sergio_skills import Sergio
    ROBOTS["sergio"] = Sergio
except ImportError:
    pass

from .mockbot import Mockbot
ROBOTS["mockbot"] = Mockbot


def get_robot_from_argv(index, default_robot_name="hero"):
    """
    Construct a robot from the name given in the command-line or from the default robot name.

    :param index: Index in the command-line arguments where a robot name may be available.
    :param default_robot_name: Name of the robot to use if the command line did not contain a name.
    :return: The constructed robot.
    :raise: RunTimeError if no robot could be created.
    """
    if len(sys.argv) > index:
        robot_name = sys.argv[index]
    else:
        robot_name = default_robot_name

    return get_robot(robot_name)


def get_robot(name):
    # type: (str) -> Robot
    """
    Constructs a robot (api) object based on the provided name

    :param name: (str) robot name
    :return: (Robot)
    :raises: RuntimeError
    """
    rospy.loginfo("Constructing robot {}".format(name))
    robot_class = ROBOTS.get(name.lower())
    if robot_class is not None:
        return robot_class()
    else:
        error_msg = "Cannot construct robot '{}'\n".format(name)
        error_msg += "Available robots:\n\t{}\n".format("\n\t".join(list(ROBOTS.keys())))
        error_msg += "To install, try: 'tue-get install {}_skills'".format(name)
        rospy.logerr(error_msg)
        raise RuntimeError(error_msg)
