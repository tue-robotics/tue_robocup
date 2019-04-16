# ROS
import rospy

# Robot skills
from .amigo import Amigo
from .hero import Hero
# from .mockbot import Mockbot  # ToDo: enable after merging Loys MockBot update
from .sergio import Sergio


def get_robot(name):
    """
    Constructs a robot (api) object based on the provided name
    :param name: (str) robot name
    :return: (Robot)
    :raises: RuntimeError
    """
    rospy.loginfo("Constructing robot {}".format(name))
    name = name.lower()
    if name == "amigo":
        return Amigo()
    elif name == "hero":
        return Hero()
    # elif name == "mockbot":
    #     return Mockbot()
    elif name == "sergio":
        return Sergio()
    else:
        raise RuntimeError("Don't know which robot to construct with name {}".format(name))
