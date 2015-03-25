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
import geometry_msgs.msg as gm

from collections import namedtuple
from smach_ros import SimpleActionState
from robot_skills.amigo import Amigo
from robot_smach_states.util.designators import DesignatorResolvementError, EdEntityDesignator, AttrDesignator, VariableDesignator, Designator
from ed_perception.msg import FaceLearningGoal, FaceLearningResult
from ed.msg import EntityInfo


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


class Location(object):
    def __init__(self, point_stamped, visited, attempts):
        self.point_stamped = point_stamped
        self.visited = visited
        self.attempts = attempts

class FaceAnalysed(object):
    def __init__(self, point_stamped, name, score):
        self.point_stamped = point_stamped
        self.name = name
        self.score = score


# ----------------------------------------------------------------------------------------------------


class PointDesignator(Designator):
    """ Returns a more or less hardcoded designator"""
    def __init__(self, point_stamped=None):
        super(Designator, self).__init__()
        
        self.entity = EntityInfo()

        if not point_stamped == None:
            self.entity.pose.position = point_stamped.point

    def setPoint(self, point_stamped):
        self.entity.pose.position = point_stamped.point
    
    def resolve(self):
        return self.entity


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
    def __init__(self, robot, lookDown = False):
        smach.State.__init__(self, outcomes=['succeded', 'failed'])
        self.robot = robot
        self.lookDown = lookDown

    def execute(self, robot):
        print OUT_PREFIX + bcolors.WARNING + "LookAtPersonInFront" + bcolors.ENDC

        # initialize variables
        foundFace = False
        result = None
        entityData = None
        faces_front = None
        desgnResult = None

        # set robots pose
        self.robot.spindle.high()
        
        # look front, 2 meters high
        # self.robot.head.look_at_point(point_stamped=msgs.PointStamped(3, 0, 2,"amigo/base_link"), end_time=0, timeout=4)

        self.robot.head.look_at_standing_person()


        # create designator
        humanDesignator = EdEntityDesignator(self.robot, type="human")
        dataDesignator = AttrDesignator(humanDesignator, 'data')

        # try to resolve the designator
        try:
            desgnResult = humanDesignator.resolve()
        except DesignatorResolvementError:
            print OUT_PREFIX + "Could not resolve humanDesignator"
            pass

        # if no person was seen at 2 meters high, look down, because the person might be sitting
        if desgnResult == None and self.lookDown == True:
            # look front, 2 meters high
            self.robot.head.look_at_point(point_stamped=msgs.PointStamped(3, 0, 0,"amigo/base_link"), end_time=0, timeout=4)
            # rospy.sleep(2)    # to give time to the head to go to place and the perception to get results            

        # if there is a person in front, try to look at the face
        if desgnResult:
            print OUT_PREFIX + "Found a human!"
    
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
                    headGoal = msgs.PointStamped(x=faces_front["map_x"], y=faces_front["map_y"], z=faces_front["map_z"], frame_id="/map")
                    # import ipdb; ipdb.set_trace()

                    print OUT_PREFIX + "Sending head goal to (" + str(headGoal.point.x) + ", " + str(headGoal.point.y) + ", " + str(headGoal.point.z) + ")"
                    self.robot.head.look_at_point(point_stamped=headGoal, end_time=0, timeout=4)

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

        self.robot.head.cancel_goal()

        return 'done'


# ----------------------------------------------------------------------------------------------------


class FindCrowd(smach.State):
    def __init__(self, robot, locations):
        smach.State.__init__(   self, outcomes=['succeded', 'failed'])
        self.robot = robot
        self.locations = locations

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
        # look at 3 meters front, 5 meters right and 2 meters high
        self.robot.head.look_at_point(point_stamped=msgs.PointStamped(3,-5,2,"amigo/base_link"), end_time=0, timeout=4)
        # self.robot.head.wait()
        # rospy.sleep(2)  


        # look at 3 meters front, 5 meters left and 2 meters high
        self.robot.head.look_at_point(point_stamped=msgs.PointStamped(3,5,2,"amigo/base_link"), end_time=0, timeout=4)
        # rospy.sleep(3)

        #  clear head goals
        self.robot.head.cancelGoal()

        # resolve crowds designator
        try:
            crowds = crowdDesignator.resolve()
        except DesignatorResolvementError:
            foundCrowd = False
            pass

        if crowds:
            # print OUT_PREFIX + "Found a crowd"
            foundCrowd = True
        else:
            print OUT_PREFIX + "Didnt find a crowd"
            foundCrowd = False

        # resolve humans designator
        try:
            humans = humanDesignator.resolve()
        except DesignatorResolvementError:
            pass

        if humans:
            # print OUT_PREFIX + "Found a human"
            foundHuman = True
        else:
            print OUT_PREFIX + "Didn't find humans"
            foundHuman = False

        # if no one was found, return failed, else add locations to list
        if not foundCrowd and not foundHuman:
            print OUT_PREFIX + "Could not find anyone in the room"
            return 'failed'
        else:
        # if a person or crowd was found, add their positions to the list

            print OUT_PREFIX + "Current list of locations: " + str(self.locations.resolve())
            # import ipdb; ipdb.set_trace()

            # resolve crowd locations
            if foundCrowd:
                try:
                    dataDesignator = AttrDesignator(crowdDesignator, 'center_point')
                    entityData = dataDesignator.resolve()
                    print OUT_PREFIX + "Crowd found at:\n" + str(entityData)
                except DesignatorResolvementError:
                    print OUT_PREFIX + "Could not resolve dataDesignator"
                    pass

                # add point to locations to visit
                try:
                    self.locations.current += [Location(point_stamped = msgs.PointStamped(x=entityData.x, y=entityData.y, z=entityData.z, frame_id="/map"),
                                                        visited = False,
                                                        attempts = 0)]
                except KeyError, ke:
                    print OUT_PREFIX + "KeyError faces_front: " + ke
                    pass
                except IndexError, ke:
                    print OUT_PREFIX + "IndexError faces_front: " + ke
                    pass

            # resolve human locations
            if foundHuman:
                # resolve the data designator
                try:
                    dataDesignator = AttrDesignator(humanDesignator, 'center_point')
                    entityData = dataDesignator.resolve()
                    print OUT_PREFIX + "Human found at:\n" + str(entityData)
                except DesignatorResolvementError:
                    print OUT_PREFIX + "Could not resolve dataDesignator"
                    pass

                # add point to locations to visit
                try:
                    self.locations.current += [Location(point_stamped = msgs.PointStamped(x=entityData.x, y=entityData.y, z=entityData.z, frame_id="/map"),
                                                        visited = False, 
                                                        attempts = 0)]
                except KeyError, ke:
                    print OUT_PREFIX + "KeyError faces_front: " + ke
                    pass
                except IndexError, ke:
                    print OUT_PREFIX + "IndexError faces_front: " + ke
                    pass
            
            # TODO: Filter the list so there are no dupliacte locations
            print OUT_PREFIX + "Updated locations to visit:" + str(self.locations.resolve())

            return 'succeded'


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


class GoToOperator(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['unreachable', 'goal_not_defined', 'arrived'])

        self.robot = robot

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "GoToOperator" + bcolors.ENDC

        return 'arrived'


# ----------------------------------------------------------------------------------------------------


class AnalyzePerson(smach.State):
    def __init__(self, robot, facesAnalysed):
        smach.State.__init__(self, outcomes=['succeded', 'failed'])

        self.robot = robot
        self.facesAnalysed = facesAnalysed

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "AnalyzePerson" + bcolors.ENDC
        
        # initialize variables
        analysed = False
        result = None
        entityData = None
        recognition_label = None
        recognition_score = None
        humanDesgnResult = None
        crowdDesgnResult = None

        # create designator
        crowdDesignator = EdEntityDesignator(self.robot, type="crowd")
        humanDesignator = EdEntityDesignator(self.robot, type="human")
        dataDesignator = AttrDesignator(humanDesignator, 'data')
        crowdDataDesignator = AttrDesignator(crowdDesignator, 'data')

        # try to resolve the human designator
        try:
            humanDesgnResult = humanDesignator.resolve()            
        except DesignatorResolvementError:
            print OUT_PREFIX + "Could not resolve humanDesignator. No humans found"
            pass


        # try to resolve the crowd designator
        try:
            crowdDesgnResult = crowdDesignator.resolve()
        except DesignatorResolvementError:
            print OUT_PREFIX + "Could not resolve crowdDesignator. No crowds found"
            pass

        # If there is no result it might mean that the person is sitting down, so look down
        if not desgnResult:
            # set robots pose
            self.robot.spindle.high()
            self.robot.head.look_at_standing_person()
            # rospy.sleep(1)    # to give time to the head to go to place and the perception to get results            



        import ipdb; ipdb.set_trace()

        # if a person was found, save the results from face recognition
        if humanDesgnResult :
            print "Human designator got a result. Trying to resolve Data designator."
    
            # resolve the data designator
            try:
                entityData = dataDesignator.resolve()
                print "Entity data (human): " + str(entityData)

                # crowdData = crowdDataDesignator.resolve()
                # print "Entity data (crowd): " + str(entityData)
            except DesignatorResolvementError:
                print OUT_PREFIX + "Could not resolve dataDesignator"
                pass

            # extract name, score and face location
            if not entityData == None:
                try:
                    # print "Entity data: " + str(entityData)
                    # get information on the first face found (cant guarantee its the closest in case there are many)
                    # import ipdb; ipdb.set_trace()
                    recognition_label = entityData["perception_result"]["face_recognizer"]["label"]
                    recognition_score = entityData["perception_result"]["face_recognizer"]["score"]

                    # get location
                    faces_front = entityData["perception_result"]["face_detector"]["faces_front"][0]

                    # initialize label as empty string
                    if recognition_score == 0:
                        recognition_label = ""
                        print OUT_PREFIX + "Unrecognized person"

                except KeyError, ke:
                    print OUT_PREFIX + "KeyError recognition_label/recognition_score: " + ke
                    pass
                except IndexError, ke:
                    print OUT_PREFIX + "IndexError recognition_label/recognition_score: " + ke
                    pass

                #  if information is valid
                if not recognition_label == None and not recognition_score == None:
                    print OUT_PREFIX + "Recognition result: '" + str(recognition_label) + "' (score: " + str(recognition_score) + ")"

                    analysed == True
                    # import ipdb; ipdb.set_trace()
                    self.facesAnalysed.current += [(FaceAnalysed(point_stamped = msgs.PointStamped(x=faces_front["map_x"], y=faces_front["map_y"], z=faces_front["map_z"], frame_id="/map"),
                                                         name = recognition_label, 
                                                         score = recognition_score))]
                else:
                    print OUT_PREFIX + "Label and/or score empty: '" + str(recognition_label) + "', " + str(recognition_score) + "'"

            else:
                print OUT_PREFIX + "Attribute Designator returned empty"


        # cancel the head goal from the LookAtPersonInFront state
        # self.robot.head.cancelGoal()
        # self.robot.spindle.medium()

        if analysed:
            print OUT_PREFIX + "Faces Analysed so far: " + self.facesAnalysed
            return 'succeded'
        else:
            print OUT_PREFIX + "Could not find anyone in front of the robot"
            return 'failed'


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


class GetNextLocation(smach.State):
    """
    From a list of locations to visit, choose the next one and go there
    """
    def __init__(self, robot, locations, nextLocation):
        smach.State.__init__(   self, outcomes=['done', 'visited_all'])

        self.robot = robot
        self.locations = locations
        self.nextLocation = nextLocation

    def execute(self, userdata):
        print OUT_PREFIX + bcolors.WARNING + "GetNextLocation" + bcolors.ENDC

        availableLocations = self.locations.resolve()
        
        # if there are locations not visited yet, choose one
        if availableLocations:
            
            print OUT_PREFIX + str(len(availableLocations)) + " locations still available to visit"
            
            chosenLocation = None
            # TODO: CHOOSE CLOSEST LOCATION
            for loc in availableLocations:

                if loc.visited == False:
                    chosenLocation = loc.point_stamped
                    loc.attempts +=1
                    break

            if not chosenLocation:
                print OUT_PREFIX + 'All locations are marked as visited'
                return 'visited_all'
            else:
                self.nextLocation.current.setPoint(point_stamped = chosenLocation)
                print OUT_PREFIX + "Next location: " + str(chosenLocation)

                return 'done'
            
        # If there are not more loactions to visit then report back to the operator
        else:

            # TODO: CONFIRM EVERY LOCATION WAS VISIT OR RETRY UNREACHABLE LOCATIONS

            print OUT_PREFIX + "Visited all locations"
            return 'visited_all'
