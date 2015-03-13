#! /usr/bin/env python
import roslib; 
import rospy
import smach
import subprocess
import inspect
import random
import ed_perception.msg
import actionlib
import actionlib_msgs
import robot_skills.util.msg_constructors as msgs

from smach_ros import SimpleActionState
from robot_skills.amigo import Amigo
from robot_smach_states.util.designators import DesignatorResolvementError, EdEntityDesignator, AttrDesignator
from ed_perception.msg import FaceLearningGoal, FaceLearningResult


# import ipdb; ipdb.set_trace()

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


OUT_PREFIX = bcolors.WARNING + "[CHALLENGE PERSON RECOGNITION] " + bcolors.ENDC

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

        # initialize variables
        foundFace = False
        result = None
        entityData = None
        faces_front = None

        # set robots pose
        self.robot.spindle.high()
        self.robot.head.set_pan_tilt(0, -0.2)
        self.robot.head.set_pan_tilt(0, -0.2)   # doing this twice because once does not work, bug?
        rospy.sleep(1)    # to give time to the head to go to place and the perception to get results

        # create designator
        humanDesignator = EdEntityDesignator(self.robot, type="human")
        dataDesignator = AttrDesignator(humanDesignator, 'data')

        # try to resolve the designator
        try:
            result = humanDesignator.resolve()
        except DesignatorResolvementError:
            print OUT_PREFIX + "Could not resolve humanDesignator"
            pass

        # if there is a person in front, try to look at the face
        if result:
            print "Got a result"
    
            # resolve the data designator
            try:
                entityData = dataDesignator.resolve()
            except DesignatorResolvementError:
                print OUT_PREFIX + "Could not resolve dataDesignator"
                pass

            # extract information from data
            if entityData:
                try:
                    # get information on the first face found (cant guarantee its the closest in case there are many)
                    faces_front = entityData["perception_result"]["face_detector"]["faces_front"][0]
                except KeyError, ke:
                    print OUT_PREFIX + "KeyError faces_front: " + ke
                    pass
                except IndexError, ke:
                    print OUT_PREFIX + "IndexError faces_front: " + ke
                    pass

                if faces_front:
                    # TODO: look at the person's face, and slightly down
                    headGoal = msgs.PointStamped(x=faces_front["map_x"], y=faces_front["map_y"], z=faces_front["map_z"], frame_id="/map")
                    # import ipdb; ipdb.set_trace()

                    print OUT_PREFIX + "Sending head goal to (" + str(headGoal.point.x) + ", " + str(headGoal.point.y) + ", " + str(headGoal.point.z) + ")"

                    self.robot.head.send_goal(point_stamped = headGoal)

                    foundFace == True            
                else:
                    print OUT_PREFIX + "Could not find anyone in front of the robot. It will just look in front and up."
                    return 'failed'

        
        if foundFace == True:
            return 'succeded'
        else:
            print OUT_PREFIX + "Could not find anyone in front of the robot. It will just look in front and up."
            return 'failed'


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
        smach.State.__init__(   self,
                                outcomes=['success', 'failed'],
                                output_keys=['locations_out'])
        self.robot = robot

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "FindCrowd" + bcolors.ENDC

        self.robot.spindle.high()

        foundCrowd = False
        foundHuman = False
        crowds = None
        humans = None
        locationsToVistit = []
        
        # create designators
        crowdDesignator = EdEntityDesignator(self.robot, type="crowd")
        humanDesignator = EdEntityDesignator(self.robot, type="human")

        # "scan" the room with the head
        print "starting scan"
        # turn head to one side to start the swipping the room
        # self.robot.head.set_pan_tilt(pan=-1.1, tilt=-0.2, timeout=3.0)
        self.robot.head.setPanTiltGoal(pan=-1.1, tilt=-0.2)
        self.robot.head.setPanTiltGoal(pan=-1.1, tilt=-0.2)
        # self.robot.head.wait()
        rospy.sleep(2)

        print "center position"
        # turn head to the center
        # self.robot.head.set_pan_tilt(pan=0.0, pan_vel=0.1, tilt=-0.2, timeout=3.0)
        self.robot.head.setPanTiltGoal(pan=0.0, pan_vel=0.1, tilt=-0.2)
        self.robot.head.setPanTiltGoal(pan=0.0, pan_vel=0.1, tilt=-0.2)
        # self.robot.head.wait()
        rospy.sleep(2)

        print "side position"
        # turn head to the other side
        # self.robot.head.set_pan_tilt(pan=1.1, pan_vel=0.1, tilt=-0.2, timeout=5.0)
        self.robot.head.setPanTiltGoal(pan=1.1, pan_vel=0.1, tilt=-0.2)
        self.robot.head.setPanTiltGoal(pan=1.1, pan_vel=0.1, tilt=-0.2)
        # self.robot.head.wait()
        rospy.sleep(1)

        print "canceling goal"
        self.robot.head.cancelGoal()


        # interpret results
        try:
            crowds = crowdDesignator.resolve()
        except DesignatorResolvementError:
            foundCrowd = False
            pass

        if crowds:
            print OUT_PREFIX + "Found a crowd"
            # print crowds

            # locationsToVistit.extend(crowds)
            foundCrowd = True
        else:
            print OUT_PREFIX + "Didnt find a crowd"
            foundCrowd = False

        try:
            humans = humanDesignator.resolve()
        except DesignatorResolvementError:
            pass

        if humans:
            print OUT_PREFIX + "Found a human"
            # print humans

            # locationsToVistit.extend(humans)
            foundHuman = True
        else:
            print OUT_PREFIX + "Didnt find no one"
            foundHuman = False


        print OUT_PREFIX + "To visit:"
        print locationsToVistit

        

        if foundCrowd or foundHuman:
            import ipdb; ipdb.set_trace()
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
        smach.State.__init__(self, outcomes=['succeded', 'failed'])
        self.robot = robot

    def execute(self, robot):
        print OUT_PREFIX + bcolors.WARNING + "PointAtOperator" + bcolors.ENDC

        # Get information about the operator and point at the location

        return 'succeded'


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

        self.robot.speech.speak("What is your name?", block=True)

        self.robot.speech.speak("I shall call you " + name + "!", block=False)

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
        self.service_name = "/" + robot.robot_name + "/ed/face_recognition/learn_face"

        with self:

            # Callback when a result is received
            def learn_result_cb(userdata, status, result):
                print OUT_PREFIX + bcolors.WARNING + "learn_result_cb" + bcolors.ENDC

                print "Received result from the learning service"

                # test the result and parse the message
                if status == actionlib.GoalStatus.SUCCEEDED:
                    if result.result_info == "Learning complete":
                        print "Face learning complete! result: " + result.result_info
                        # self.robot.speech.speak("Learning complete.", block=False)

                        return 'succeeded'
                    else:
                        return 'aborted'
                else:
                    print "Face learning aborted! result: " + result.result_info
                    return 'aborted'

            # Callback when a result is sent
            def learn_goal_cb(userdata, goal):
                print OUT_PREFIX + bcolors.WARNING + "goal_result_cb" + bcolors.ENDC
                # import ipdb; ipdb.set_trace()

                self.robot.speech.speak("Please look at me while I learn your face", block=True)

                learn_goal = FaceLearningGoal()
                learn_goal.person_name = userdata.person_name_goal

                print "Goal sent to the learning service, with name '" + learn_goal.person_name + "'"

                return learn_goal

            # Create Simple Action Client
            smach.StateMachine.add( 'LEARN_PERSON',
                                    SimpleActionState(  self.service_name,
                                                        ed_perception.msg.FaceLearningAction,
                                                        result_cb = learn_result_cb,
                                                        goal_cb = learn_goal_cb,            # create a goal inside the callback
                                                        input_keys=['person_name_goal'],
                                                        output_keys=['result_info_out']),
                                                        # goal_slots = ['person_name_goal'],# or create it here directly
                                    transitions={   'succeeded':'succeded_learning',
                                                    'aborted': 'failed_learning',
                                                    'preempted': 'failed_learning'},
                                    remapping={     'person_name_goal':'personName_in'})


# ----------------------------------------------------------------------------------------------------


# Ask the persons name
class VisitLocations(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['succeded', 'failed'],
                                input_keys=['locations_in'])

        self.robot = robot

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "VisitLocations" + bcolors.ENDC

        print "Locations still available to visit: "
        import ipdb; ipdb.set_trace()

        return 'succeded'