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

from robot_smach_states.util.designators import DesignatorResolvementError, EdEntityDesignator, AttrDesignator

from ed_perception.msg import FaceLearningGoal, FaceLearningResult


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
        smach.State.__init__(self, outcomes=['succeded', 'failed'])
        self.robot = robot

    def execute(self, robot):
        print OUT_PREFIX + bcolors.WARNING + "LookAtPersonInFront" + bcolors.ENDC

        # create designator
        humanDesignator = EdEntityDesignator(self.robot, type="human")
        dataDesignator = AttrDesignator(humanDesignator, 'data')

        # set robots pose
        self.robot.spindle.high()
        self.robot.head.set_pan_tilt(0, -0.2)

        # initialize result
        result = None

        # try to resolve the designator
        try:
            result = humanDesignator.resolve()
        except DesignatorResolvementError:
            pass

        # if there is a person in front, try to look at the face
        if result:
            print "Got a result"
    
            # try to resolve the designator
            try:
                entityData = dataDesignator.resolve()
            except DesignatorResolvementError:
                pass            

            # print entityData
            # TODO: look at the person's face, and slightly down

            # setLookAtGoal(self, point_stamped, end_time=0, pan_vel=0.2, tilt_vel=0.2, wait_for_setpoint=False):
            # send_goal(self, point_stamped, timeout=4.0, keep_tracking=False, min_pan=0, max_pan=0, min_tilt=0, max_tilt=0, pan_vel=0, tilt_vel=0):
            
            # center_point: 
            #   x: 1.56504058838
            #   y: 4.42022323608
            #   z: 1.07198965549

            # self.robot.head.send_goal(point_stamped = ) #keep_tracking=False
            
            # import ipdb; ipdb.set_trace()

            """
            Example of the output:
            {'type': 'human', 
            'perception_result': {
                'face_recognizer': {'fisher': {'score': 1435.45, 'name': 'luis'}, 'lbph': {'score': 46.8702, 'name': 'luis'}, 'score': 0, 'label': None}, 
                'human_contour_matcher': {'deviation': 2348.98, 'score': 0, 'error': 2191.49, 'stance': 'side_left', 'label': 'human_shape'}, 
                'face_detector': {'score': 1, 'faces_front': [{'y': 182, 'width': 195, 'height': 195, 'x': 318}], 'label': 'face'}, 
                'type_aggregator': {'score': 1, 'type': 'human'}, 'histogram': [{'amount': 0.910328, 'type': 'chewing_gum_white'}, {'amount': 0.149846, 'type': 'coffee_pads'}, {'amount': 0.456642, 'type': 'cola'}, {'amount': 0.357733, 'type': 'cup'}, {'amount': 0.173658, 'type': 'deodorant'}, {'amount': 0.796962, 'type': 'fanta'}, {'amount': 0.154168, 'type': 'tea'}], 'size_matcher': {'score': 1, 'label': 'large_size', 'size': {'width': 1.36362, 'height': 0.673354}}, 
                'odu_finder': None, 
                'color_matcher': {'colors': [{'name': 'black', 'value': 0.0292143}, {'name': 'blue', 'value': 0.0013802}, {'name': 'brown', 'value': 0.0357389}, {'name': 'green', 'value': 0.00255129}, {'name': 'grey', 'value': 0.0462578}, {'name': 'orange', 'value': 4.18244e-05}, {'name': 'pink', 'value': 0.00280223}, {'name': 'purple', 'value': 0.0041197}, {'name': 'red', 'value': 0.000543717}, {'name': 'white', 'value': 0.876785}, {'name': 'yellow', 'value': 0.000564629}], 'hypothesis': [{'score': 0.910328, 'name': 'chewing_gum_white'}, {'score': 0.149846, 'name': 'coffee_pads'}, {'score': 0.456642, 'name': 'cola'}, {'score': 0.357733, 'name': 'cup'}, {'score': 0.173658, 'name': 'deodorant'}, {'score': 0.796962, 'name': 'fanta'}, {'score': 0.154168, 'name': 'tea'}]}}}
            """

        else:
            print "Could not find anyone in front of the robot. It will just look in front and up."
            return 'failed'

        return 'succeded'

# ----------------------------------------------------------------------------------------------------

class CancelHeadGoals(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['done'])
        self.robot = robot

    def execute(self, robot):
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

        self.robot.spindle.high()

        foundCrowd = False
        foundHuman = False
        
        # create designators
        crowdDesignator = EdEntityDesignator(self.robot, type="crowd")
        humanDesignator = EdEntityDesignator(self.robot, type="human")

        print "starting scan"
        # turn head to one side to start the swipping the room
        # self.robot.head.set_pan_tilt(pan=-1.1, tilt=-0.2, timeout=3.0)
        self.robot.head.setPanTiltGoal(pan=-1.1, tilt=-0.2)
        # self.robot.head.wait()
        rospy.sleep(3)

        print "center position"
        # turn head to the center
        # self.robot.head.set_pan_tilt(pan=0.0, pan_vel=0.1, tilt=-0.2, timeout=3.0)
        self.robot.head.setPanTiltGoal(pan=0.0, pan_vel=0.1, tilt=-0.2)
        # self.robot.head.wait()
        rospy.sleep(3)

        print "side position"
        # turn head to the other side
        # self.robot.head.set_pan_tilt(pan=1.1, pan_vel=0.1, tilt=-0.2, timeout=5.0)
        self.robot.head.setPanTiltGoal(pan=1.1, pan_vel=0.1, tilt=-0.2)
        # self.robot.head.wait()
        rospy.sleep(2)

        
        print "canceling goal"
        self.robot.head.cancelGoal()

        result = None
        
        try:
            result = crowdDesignator.resolve()
        except DesignatorResolvementError:
            foundCrowd = False
            pass

        if result:
            print "Hey i found a crowd"
            foundCrowd = True
        else:
            print "Didnt find a crowd"
            foundCrowd = False

        if not foundCrowd:
            result = None
            try:
                result = humanDesignator.resolve()
            except DesignatorResolvementError:
                pass

            if result:
                print "Hey i found a human"
                foundCrowd = True
            else:
                print "Didnt find anything"
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

    def execute(self, robot):
        print OUT_PREFIX + bcolors.WARNING + "DescribeOperator" + bcolors.ENDC

        # Get information about the operator using the Designator

        return 'success'


# ----------------------------------------------------------------------------------------------------


class DescribeCrowd(smach.State):
    def __init__(self, robot, designator):
        smach.State.__init__(self,outcomes=['success', 'failed'])
        self.robot = robot
        self.designator = designator

    def execute(self, robot):
        print OUT_PREFIX + bcolors.WARNING + "DescribeCrowd" + bcolors.ENDC

        # Get information about the crowd using the Designator

        return 'success'


# ----------------------------------------------------------------------------------------------------


class PointAtOperator(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.robot = robot

    def execute(self, robot):
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