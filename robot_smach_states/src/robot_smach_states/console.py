#!/usr/bin/ipython -i

"""Robot console

Usage:
  robot-console <robot>... [options]
  robot-console [options] <robot>...

Options:
  -h --help                     Show this screen.
  -t TIMEOUT --timeout=TIMEOUT  Timeout for ROS connections [Default: {}].

Examples:
  robot-console amigo
  robot-console sergio
  robot-console amigo sergio
"""

from __future__ import absolute_import, print_function

# System
import sys

from docopt import docopt, DocoptExit
import traceback

# ROS
import rospy

# TU/e Robotics
from robot_skills.robot import DEFAULT_CONNECTION_TIMEOUT
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


def load_robot(robot_name, *args, **kwargs):

    try:
        print(bcolors.OKBLUE + '\tloading {}'.format(robot_name) + bcolors.ENDC)
        robot = get_robot(robot_name, *args, **kwargs)

        # register as global
        globals()[robot_name] = robot
        print(bcolors.OKGREEN+'\tSuccessfully loaded "{}"'.format(robot_name) + bcolors.ENDC)
    except Exception:
        msg = '\n"{}" Could not be found.\n'.format(robot_name)
        print(bcolors.WARNING + msg + bcolors.ENDC)
        traceback.print_exc()


if __name__ == "__main__":
    __doc__ = __doc__.format(DEFAULT_CONNECTION_TIMEOUT)
    try:
        arguments = docopt(__doc__)
        if arguments["--help"]:
            print(__doc__)
            sys.exit(0)
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
            load_robot(robot, connection_timeout=arguments["--timeout"])

    except DocoptExit as e:
        print(bcolors.FAIL + str(e) + bcolors.ENDC)
        sys.exit(1)  # quit ipython
