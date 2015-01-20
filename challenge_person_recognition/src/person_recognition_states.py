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

from robot_smach_states.manip.grab import Grab

import inspect


# ----------------------------------------------------------------------------------------------------

class Test(smach.State):
    """Sets a VariableDesignator to an Entity that matches some criteria."""
    def __init__(self, robot, designator):
        smach.State.__init__(self,outcomes=['done', 'failed'])
        self.robot = robot
        self.designator = designator

    def execute(self, userdata):


        return 'done'
