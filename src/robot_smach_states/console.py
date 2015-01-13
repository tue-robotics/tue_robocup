#!/usr/bin/ipython -i

# ROS
import roslib; roslib.load_manifest('robot_console')
import rospy
import sys

# Robot interfaces
from robot_skills import robot, amigo, sergio

# State machines
import robot_smach_states as state_machine

def loadRobot(robot_name):
    if robot_name == "amigo":
        globals()[robot_name] = amigo.Amigo(wait_services=False)
    elif robot_name == "sergio":
        globals()[robot_name] = sergio.Sergio(wait_services=False)
    else:
        return None

    print """\033[92m
    Succesfully loaded robot '%s'\033[1;m"""%robot_name

if __name__ == "__main__":
    rospy.init_node("robot_console", anonymous=True)

    print """
    \033[1;33m-------------------------------------------------------------------
    | TU/e Robot Console - version 0.1                                |
    |                                                                 |
    | use loadRobot([robot_name]) to load robot ; available robots:   |
    |    - amigo                                                      |
    |    - sergio                                                     |
    |                                                                 |
    | Robots can be automatically loaded with arguments.              |
    -------------------------------------------------------------------\033[1;m"""

    for arg in sys.argv:
        loadRobot(arg)

    print """\033[92m
    Succesfully loaded statemachines  as 'state_machine'\033[1;m"""