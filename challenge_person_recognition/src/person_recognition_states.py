#! /usr/bin/env python
import rospy

import smach

from robot_skills.amigo import Amigo
from robot_smach_states import *

from robot_skills.reasoner  import Conjunction, Compound, Disjunction, Constant
from robot_smach_states.util.startup import startup
import robot_skills.util.msg_constructors as msgs
import robot_skills.util.transformations as transformations
from robot_smach_states.designators.designator import Designator, VariableDesignator, EdEntityByQueryDesignator

from pein_srvs.srv import SetObjects
from ed.srv import SimpleQuery, SimpleQueryRequest

from robot_smach_states.utility_states import Initialize
from robot_smach_states.human_interaction import Say

from robot_smach_states import Grab

import inspect

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


OUT_PREFIX = bcolors.WARNING + "[CHALLENGE BASIC FUNCTIONALITIES] " + bcolors.ENDC

# ----------------------------------------------------------------------------------------------------

class WaitForOperator(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['success', 'failed'])
        self.robot = robot

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "WaitForOperator" + bcolors.ENDC

        # Return sucess only when a person is seen in front of the robot, or time-out after a minute

        return 'success'


# ----------------------------------------------------------------------------------------------------

class LearnPerson(smach.State):
    def __init__(self, robot, name):
        smach.State.__init__(self,outcomes=['success', 'failed'])
        self.robot = robot
        self.name = name

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "LearnPerson" + bcolors.ENDC

        # Do a service call to initate learning
        print OUT_PREFIX + "Learning with name " + self.name

        return 'success'


# ----------------------------------------------------------------------------------------------------

class FindCrowd(smach.State):
    def __init__(self, robot, designator):
        smach.State.__init__(self,outcomes=['success', 'failed'])
        self.robot = robot
        self.designator = designator

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "FindCrowd" + bcolors.ENDC

        # Get location of people and determine if the proximity makes them a crowd

        return 'success'

# ----------------------------------------------------------------------------------------------------

class DescribeOperator(smach.State):
    def __init__(self, robot, designator):
        smach.State.__init__(self,outcomes=['success', 'failed'])
        self.robot = robot
        self.designator = designator

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "DescribeOperator" + bcolors.ENDC

        # Get information about the operator using the Designator

        return 'success'

# ----------------------------------------------------------------------------------------------------

class DescribeCrowd(smach.State):
    def __init__(self, robot, designator):
        smach.State.__init__(self,outcomes=['success', 'failed'])
        self.robot = robot
        self.designator = designator

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "DescribeCrowd" + bcolors.ENDC

        # Get information about the crowd using the Designator

        return 'success'

# ----------------------------------------------------------------------------------------------------
