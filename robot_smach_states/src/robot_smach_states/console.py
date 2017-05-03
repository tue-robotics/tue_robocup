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
import traceback

import rospy
from docopt import docopt, DocoptExit

# load state machines
import robot_smach_states as state_machine
import robot_smach_states.util.designators as ds
import robot_skills.util.msg_constructors as msgs

# Load convenient data types (DO NOT REMOVE)
import PyKDL as kdl
from robot_skills.util.kdl_conversions import frame_stamped


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

def load_robot(robot_name, parts=None):
    modules = [robot_name] if not parts else parts

    loaded = []
    for name in modules:
        try:
            print bcolors.OKBLUE + '\tloading %s' % name + bcolors.ENDC
            module = importlib.import_module('robot_skills.' + name)
            constructor = module.__getattribute__(name.title())

            if parts:
                instance = constructor(robot_name, wait_services=True)
            else:
                instance = constructor(wait_services=False)

            loaded.append(instance)
            # register as global
            globals()[name] = instance
            print bcolors.OKGREEN+'\tSuccesfully loaded "%s"' % name +bcolors.ENDC
        except (ImportError, AttributeError, TypeError) as e:
            msg = '\n"%s" could not be found!!!\n' % name
            print bcolors.WARNING + msg + bcolors.ENDC
            traceback.print_exc()

    if not len(loaded):
        raise DocoptExit("error: no robots or parts loaded")

if __name__ == '__main__':
    try:
        arguments = docopt(__doc__)
        rospy.init_node("robot_console", anonymous=True)

        print """
        \033[1;33m-------------------------------------------------------------------
        | TU/e Robot Console - version 0.1                                |
        -------------------------------------------------------------------\033[1;m"""

        print bcolors.OKGREEN+'\tSuccesfully loaded statemachines as "state_machine"'+bcolors.ENDC
        print bcolors.OKGREEN+'\tSuccesfully loaded designators as "ds"'+bcolors.ENDC

        parts = arguments['--part']
        parts = parts.split(',') if parts else None
        for robot in arguments['<robot>']:
            load_robot(robot, parts=parts)

    except DocoptExit as e:
        print bcolors.FAIL + str(e) + bcolors.ENDC
        exit() # quit ipython
