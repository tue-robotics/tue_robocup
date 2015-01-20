#!/usr/bin/ipython -i

"""Robot console

Usage:
  robot-console <robot>...
  robot-console <robot>... [--part=<parts>]

Examples:
  robot-console amigo
  robot-console sergio --part=arms,head
  robot-console amigo sergio
"""

import sys
import importlib
import rospy
from docopt import docopt, DocoptExit

# load state machines
import robot_smach_states as state_machine

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

def load_robot(robot_name, parts=None):
    try:
        module = importlib.import_module('robot_skills.' + robot_name)
        robot = module.__getattribute__(robot_name.title())(wait_services=False)
    except (ImportError, AttributeError) as e:
        msg = '\nrobot "%s" could not be found!!!\n' % robot_name
        msg += '\nException:\n' + str(e) + '\n'
        raise DocoptExit(msg)

    # register as global
    globals()[robot_name] = robot

    print bcolors.OKGREEN+'\tSuccesfully loaded robot "%s"' % robot_name +bcolors.ENDC

if __name__ == '__main__':
    try:
        arguments = docopt(__doc__)
        rospy.init_node("robot_console", anonymous=True)

        print """
        \033[1;33m-------------------------------------------------------------------
        | TU/e Robot Console - version 0.1                                |
        -------------------------------------------------------------------\033[1;m"""

        print bcolors.OKGREEN+'\tSuccesfully loaded statemachines as "state_machine"'+bcolors.ENDC

        parts = arguments['--part']
        for robot in arguments['<robot>']:
            load_robot(robot, parts=parts)

    except DocoptExit as e:
        print bcolors.FAIL + str(e) + bcolors.ENDC
        exit() # quit ipython
