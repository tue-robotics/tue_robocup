# ROS
import rospy

def load_param(robot_name: str, param_name: str, default=None):
    """
    Loads a parameter from the parameter server, namespaced by robot name

    :param robot_name: Name of the robot being used.
    :param param_name: Parameter name.
    :param default: Default value if parameter is unavailable.
    :return: Parameter value (loaded or default).
    """
    if default is None:
        return rospy.get_param('/' + robot_name + '/' + param_name)
    else:
        return rospy.get_param('/' + robot_name + '/' + param_name, default)
