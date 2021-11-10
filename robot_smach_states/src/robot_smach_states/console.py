#!/usr/bin/ipython -i

"""Robot console

Usage:
  robot-console <robot>...
  robot-console <robot>...

Examples:
  robot-console amigo
  robot-console sergio
  robot-console amigo sergio
"""

from __future__ import absolute_import, print_function

# System
from docopt import docopt, DocoptExit
import traceback

# ROS
import rospy

# TU/e Robotics
from robot_skills import get_robot

# Load convenient data types (DO NOT REMOVE)
# noinspection PyUnresolvedReferences
import PyKDL as kdl
# noinspection PyUnresolvedReferences
from pykdl_ros import FrameStamped, VectorStamped

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'


def load_robot(robot_name,):

    try:
        print(bcolors.OKBLUE + '\tloading {}'.format(robot_name) + bcolors.ENDC)
        robot = get_robot(robot_name)

        # register as global
        globals()[robot_name] = robot
        print(bcolors.OKGREEN+'\tSuccesfully loaded "{}"'.format(robot_name) + bcolors.ENDC)
    except Exception:
        msg = '\n"{}" Could not be found.\n'.format(robot_name)
        print(bcolors.WARNING + msg + bcolors.ENDC)
        traceback.print_exc()


if __name__ == '__main__':
    try:
        arguments = docopt(__doc__)
        rospy.init_node("robot_console", anonymous=True)

        print("""
        \033[1;33m-------------------------------------------------------------------
        | TU/e Robot Console - version 0.1                                |
        -------------------------------------------------------------------\033[1;m""")

        print('\n\tTo load state machines or designators, enter:\n')
        print('\t"import robot_skills.util.msg_constructors as msgs"')
        print('\t"import robot_smach_states.util.designators as ds"')
        print('\t"import robot_smach_states as states"\n')

        for robot in arguments['<robot>']:
            load_robot(robot)

    except DocoptExit as e:
        print(bcolors.FAIL + str(e) + bcolors.ENDC)
        exit()  # quit ipython
