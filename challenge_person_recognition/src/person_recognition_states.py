#! /usr/bin/env python
import roslib; 
import rospy
import ipdb;
import smach
import subprocess
import inspect
import random
import ed_perception.msg
import actionlib
import actionlib_msgs

from smach_ros import SimpleActionState

from robot_skills.amigo import Amigo
from robot_smach_states import *

from robot_smach_states.util.designators import DesignatorResolvementError

from ed_perception.msg import FaceLearningGoal, FaceLearningResult

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

    def execute(self):
        print OUT_PREFIX + bcolors.WARNING + "WaitForOperator" + bcolors.ENDC

        # Return sucess only when a person is seen in front of the robot, or time-out after a minute

        return 'success'


# ----------------------------------------------------------------------------------------------------

class LookAtPersonInFront(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "LookAtPersonInFront" + bcolors.ENDC

        # get location of the person and make sure the camera points to the head and body
        #self.robot.head.lookAtStandingPerson()
        self.robot.spindle.high()
        self.robot.head.set_pan_tilt(0, -0.2)

        return 'done'

# ----------------------------------------------------------------------------------------------------

class CancelHeadGoals(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['done'])
        self.robot = robot

    def execute(self):
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
        
        # create designators
        crowdDesignator = EdEntityDesignator(self.robot, id="crowd")
        humanDesignator = EdEntityDesignator(self.robot, id="human")

        print "starting scan"
        # turn head to one side to start the swipping the room
        self.robot.head.set_pan_tilt(pan=-1.1, tilt=0.0, timeout=3.0)
        rospy.sleep(3)

        print "going for the center"
        # turn head to the center
        self.robot.head.set_pan_tilt(pan=0.0, pan_vel=0.1, tilt=0.0, timeout=3.0)
        rospy.sleep(3)

        print "going for the side"
        # turn head to the other side
        self.robot.head.set_pan_tilt(pan=1.1, pan_vel=0.1, tilt=0.0, timeout=5.0)
        rospy.sleep(3)

        result = None
        
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

    def execute(self):
        print OUT_PREFIX + bcolors.WARNING + "DescribeOperator" + bcolors.ENDC

        # Get information about the operator using the Designator

        return 'success'

# ----------------------------------------------------------------------------------------------------

class DescribeCrowd(smach.State):
    def __init__(self, robot, designator):
        smach.State.__init__(self,outcomes=['success', 'failed'])
        self.robot = robot
        self.designator = designator

    def execute(self):
        print OUT_PREFIX + bcolors.WARNING + "DescribeCrowd" + bcolors.ENDC

        # Get information about the crowd using the Designator

        return 'success'

# ----------------------------------------------------------------------------------------------------

class PointAtOperator(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.robot = robot

    def execute(self):
        print OUT_PREFIX + bcolors.WARNING + "PointAtOperator" + bcolors.ENDC

        # Get information about the operator and point at the location

        return 'success'


# ----------------------------------------------------------------------------------------------------


# Ask the persons name
class AskPersonName(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['succeded', 'failed'],
                                output_keys=['personName_out'])

        self.robot = robot

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "AskPersonName" + bcolors.ENDC

        name = "Mr. Operator"
        userdata.personName_out = name

        self.robot.speech.speak("I shall call you " + name + "!", mood='excited', block=False)

        return 'succeded'

# ----------------------------------------------------------------------------------------------------


# SimpleAction state machine to learn a new face
class LearnPerson(smach.StateMachine):
    # tutorial  for SimpleActionState here http://wiki.ros.org/smach/Tutorials/SimpleActionState
    def __init__(self, robot):
        smach.StateMachine.__init__(self, 
                                    outcomes=['succeded_learning', 'failed_learning'],
                                    input_keys=['personName_in'])
        self.robot = robot

        with self:

            def learn_result_cb(userdata, status, result):
                print OUT_PREFIX + bcolors.WARNING + "learn_result_cb" + bcolors.ENDC

                # ipdb.set_trace()

                # test the result and parse the message
                if status == actionlib.GoalStatus.SUCCEEDED:
                    if result.result_info == "Learning complete":
                        print "Face learning complete! result: " + result.result_info
                        return 'succeeded'
                    else:
                        return 'aborted'
                else:
                    print "Face learning aborted! result: " + result.result_info
                    return 'aborted'

            # Create Simple Action Client
            smach.StateMachine.add( 'LEARN_PERSON',
                                    SimpleActionState('/amigo/ed/face_recognition/learn_face',
                                                    ed_perception.msg.FaceLearningAction,
                                                    result_cb = learn_result_cb,
                                                    goal_slots = ['person_name'],
                                                    output_keys=['result_info_output']),
                                                    # result_slots=['result_info']),
                                    transitions={   'succeeded':'succeded_learning',
                                                    'aborted': 'failed_learning',
                                                    'preempted': 'failed_learning'},
                                    remapping={     'person_name':'personName_in'})