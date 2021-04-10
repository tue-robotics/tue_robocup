# System
import importlib
import os.path
import sys

# ROS
import rospy
import rospkg

# Robot skills
from .robot import Robot


ROBOTS = {}


def load_robots():
    """
    Register the robots, which are export via package.xml 'export' tag
    """
    rospack = rospkg.RosPack()
    to_check = rospack.get_depends_on('robot_skills', implicit=False)
    to_check.append("robot_skills")  # also check for robots in robot_skills

    for pkg in to_check:
        m = rospack.get_manifest(pkg)
        robots = m.get_export('robot_skills', 'robot')
        if not robots:
            continue
        for robot in robots:
            try:
                # import the specified plugin module
                robot_splitted = robot.split('.')
                robot_mod = ".".join(robot_splitted[:-1])
                robot_name = robot_splitted[-1]

                mod = importlib.import_module(robot_mod)
                robot_class = getattr(mod, robot_name)

                ROBOTS[robot_name.lower()] = robot_class

                rospy.logdebug("Loaded robot: {} to {}".format(robot, robot_name.lower))

            except Exception as e:
                rospy.logerr("Unable to load robot '{}' from package '{}'. Exception thrown: '{}'".format(robot,
                                                                                                          pkg,
                                                                                                          e))


# Register the robots
load_robots()


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
