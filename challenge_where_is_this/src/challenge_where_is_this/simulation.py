# System
import os

# ROS
import rospy
import std_srvs.srv

IS_SIMULATING = False
OPERATOR_AVAILABLE = True


def _toggle_operator(_):
    """
    Toggles if the operator is following

    :param _:
    :return:
    """
    global OPERATOR_AVAILABLE
    OPERATOR_AVAILABLE = not OPERATOR_AVAILABLE
    rospy.loginfo("Operator available: {}".format(OPERATOR_AVAILABLE))
    return std_srvs.srv.EmptyResponse()


def mock_detect_operator(robot, distance=1.0, radius=0.5):  # noinspection
    """
    Mocks the 'detect operator' method. Only returns if the mocked operator is available

    :param robot: -
    :param distance: -
    :param radius: -
    :return: (bool)
    """
    assert IS_SIMULATING, "This 'detect_operator' mock should *only* be used *in simulation*"
    return OPERATOR_AVAILABLE


if os.getenv("ROBOT_REAL", "false").lower() != "true":
    rospy.Service("toggle_operator", std_srvs.srv.Empty, _toggle_operator)
    rospy.loginfo('\n\nCall:\nrosservice call /toggle_operator "{}"\n\nto toggle operator availability\n'.format('{}'))
    IS_SIMULATING = True
