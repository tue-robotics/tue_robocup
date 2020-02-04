# System
import sys

# ROS
import rospy

# Robot skills
from .robot import Robot
from .amigo import Amigo
from .hero import Hero
from .mockbot import Mockbot
from .sergio import Sergio


ROBOTS = {
    "amigo": Amigo,
    "hero": Hero,
    "mockbot": Mockbot,
    "sergio": Sergio
}


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
        raise RuntimeError("Don't know which robot to construct with name {}".format(name))
