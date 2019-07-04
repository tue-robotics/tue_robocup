# System
import os


def is_sim_mode():
    """
    Determines whether we are simulating or working with the real robot

    :return: (bool)
    """
    return os.environ.get("ROBOT_REAL", "false").lower() == "false"
