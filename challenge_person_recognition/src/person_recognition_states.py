#! /usr/bin/env python
import roslib; 
import rospy
import smach
import subprocess
import inspect
import random

from robot_skills.amigo import Amigo
from robot_smach_states import *

from robot_smach_states.util.designators import DesignatorResolvementError

# from robot_skills.reasoner  import Conjunction, Compound, Disjunction, Constant
# from robot_smach_states.util.startup import startup
# import robot_skills.util.msg_constructors as msgs
# import robot_skills.util.transformations as transformations
# from robot_smach_states.util.designators import Designator, VariableDesignator

# from pein_srvs.srv import SetObjects
# from ed.srv import SimpleQuery, SimpleQueryRequest

# from robot_smach_states.utility import Initialize
# from robot_smach_states.human_interaction import Say

# from robot_smach_states import Grab




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

        subprocess.Popen("rosservice call /face_learning/learn " + self.name)

        return 'success'

# ----------------------------------------------------------------------------------------------------

class LookAtPersonInFront(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['done'])
        self.robot = robot

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "LookAtPersonInFront" + bcolors.ENDC

        # get location of the person and make sure the camera points to the head and body
        #self.robot.head.lookAtStandingPerson()
	self.robot.head.set_pan_tilt(0,0)

        return 'done'

# ----------------------------------------------------------------------------------------------------

class CancelHeadGoals(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['done'])
        self.robot = robot

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "CancelHeadGoals" + bcolors.ENDC

        self.robot.head.cancelGoal()

        return 'done'


# ----------------------------------------------------------------------------------------------------

class FindCrowd(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['success', 'failed'])
        self.robot = robot

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "FindCrowd" + bcolors.ENDC

        foundCrowd = False
        foundHuman = False

        # turn head to one side to start the swipping the room
        self.robot.head.set_pan_tilt(pan=-1.1, tilt=0.0, timeout=3.0)
        # turn head to the center
        self.robot.head.set_pan_tilt(pan=0.0, pan_vel=0.1, tilt=0.0, timeout=3.0)
        # turn head to the other side
        self.robot.head.set_pan_tilt(pan=1.1, pan_vel=0.1, tilt=0.0, timeout=5.0)

        # query world model
        crowdDesignator = EdEntityDesignator(self.robot, id="crowd")
        humanDesignator = EdEntityDesignator(self.robot, id="human")

        try:
            result = crowdDesignator.resolve()
        except DesignatorResolvementError:
            foundCrowd = False
            pass

        if result:
            foundCrowd = True
        else:
            foundCrowd = False

        if not foundCrowd:
            try:
                result = humanDesignator.resolve()
            except DesignatorResolvementError:
                pass

            if result:
                foundCrowd = True
            else:
                foundCrowd = False


        if foundCrowd or foundHuman:
            return 'success'
        else:
            return 'failed'

# ----------------------------------------------------------------------------------------------------

class DescribeOperator(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['success', 'failed'])
        self.robot = robot

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

class PointAtOperator(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.robot = robot

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "PointAtOperator" + bcolors.ENDC

        # Get information about the operator and point at the location

        return 'success'

# ----------------------------------------------------------------------------------------------------
