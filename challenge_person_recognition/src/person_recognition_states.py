#! /usr/bin/env python
import roslib;
import rospy
import smach
import subprocess
import inspect
import random
import ed_perception.msg
import robot_skills.util.msg_constructors as msgs
# import geometry_msgs.msg as gm
import math
from smach_ros import SimpleActionState
from robot_smach_states.util.designators import *
from robot_smach_states.human_interaction.human_interaction import HearOptionsExtra
from ed.msg import EntityInfo
from dragonfly_speech_recognition.srv import GetSpeechResponse
from robocup_knowledge import load_knowledge

# ----------------------------------------------------------------------------------------------------

common_knowledge = load_knowledge("common")
challenge_knowledge = load_knowledge("challenge_person_recognition")


def printOk(sentence):
    challenge_knowledge.printOk(sentence)

def printError(sentence):
    challenge_knowledge.printError(sentence)
    
def printWarning(sentence):
    challenge_knowledge.printWarning(sentence)


class Location(object):
    def __init__(self, point_stamped, visited, attempts):
        self.point_stamped = point_stamped
        self.visited = visited
        self.attempts = attempts

class FaceAnalysed(object):
    def __init__(self, point_stamped, name, score, pose=challenge_knowledge.Pose.Standing, gender=challenge_knowledge.Gender.Male, inMainCrowd=True):
        self.point_stamped = point_stamped
        self.name = name
        self.score = score
        self.pose = pose
        self.gender = gender
        self.inMainCrowd = inMainCrowd

class PointDesignator(Designator):
    """ Returns a more or less hardcoded designator"""
    def __init__(self, point_stamped=None):
        super(PointDesignator, self).__init__(resolve_type=EntityInfo)

        self.entity = EntityInfo()

        if not point_stamped == None:
            self.entity.pose.position = point_stamped.point

    def setPoint(self, point_stamped):
        self.entity.pose.position = point_stamped.point

    def resolve(self):
        return self.entity


def points_distance(p1, p2):
    printOk("points_distance")

    deltaX = p2[0] - p1[0]
    deltaY = p2[1] - p1[1]
    deltaZ = p2[2] - p1[2]

    distance = math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ)
    printOk("Distance ({0},{1},{2}) -> ({3},{4},{5}) = {6}".format(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], distance))

    return distance


# ----------------------------------------------------------------------------------------------------


class LookAtPersonInFront(smach.State):
    def __init__(self, robot, lookDown = False):
        smach.State.__init__(self, outcomes=['succeded', 'failed'])
        self.robot = robot
        self.lookDown = lookDown

    def execute(self, robot):
        printOk("LookAtPersonInFront")

        # initialize variables
        foundFace = False
        result = None
        entityData = None
        faces_front = None
        desgnResult = None

        # create designators
        humanDesignator = EdEntityDesignator(self.robot, type="human")
        dataDesignator = AttrDesignator(humanDesignator, 'data')
        centerDesignator = AttrDesignator(humanDesignator, 'center_point')

        # set robots pose
        # self.robot.spindle.high()
        self.robot.head.cancel_goal()

        # look front, 2 meters high
        # self.robot.head.look_at_standing_person(timeout=4)
        self.robot.head.look_at_point(point_stamped=msgs.PointStamped(3, 0, 1.5,self.robot.robot_name+"/base_link"), end_time=0, timeout=4)
        rospy.sleep(3)  # give time for the percetion algorithms to add the entity

        # try to resolve the designator
        desgnResult = humanDesignator.resolve()
        if not desgnResult:
            printOk("Could not find a human while looking up")
            pass

        # if no person was seen at 2 meters high, look down, because the person might be sitting
        if desgnResult == None and self.lookDown == True:
            # look front, 2 meters high
            self.robot.head.look_at_point(point_stamped=msgs.PointStamped(3, 0, 0,self.robot.robot_name+"/base_link"), end_time=0, timeout=4)
            rospy.sleep(3)    # give time for the percetion algorithms to add the entity

            # try to resolve the designator
            desgnResult = humanDesignator.resolve()
            if not desgnResult:
                printOk("Could not find a human while looking down")
                pass

        # if there is a person in front, try to look at the face
        if not desgnResult == None:
            printOk("Designator resolved a Human!")

            # resolve the data designator
            entityData = dataDesignator.resolve()
            if not entityData:
                printOk("Could not resolve dataDesignator")
                pass

            # extract information from data
            if not entityData == None:
                faces_front = None
                try:
                    # import ipdb; ipdb.set_trace()
                    # get information on the first face found (cant guarantee its the closest in case there are many)
                    faces_front = entityData["perception_result"]["face_detector"]["faces_front"][0]
                except KeyError, ke:
                    printOk("KeyError faces_front: " + str(ke))
                    pass
                except IndexError, ke:
                    printOk("IndexError faces_front: " + str(ke))
                    pass

                if faces_front:
                    headGoal = msgs.PointStamped(x=faces_front["map_x"], y=faces_front["map_y"], z=faces_front["map_z"], frame_id="/map")

                    printOk("Sending head goal to (" + str(headGoal.point.x) + ", " + str(headGoal.point.y) + ", " + str(headGoal.point.z) + ")")
                    self.robot.head.look_at_point(point_stamped=headGoal, end_time=0, timeout=4)

                    foundFace == True            
                else:
                    printOk("Could not find anyone in front of the robot. It will just look in front and up.")
                    return 'failed'

        if foundFace == True:
            return 'succeded'
        else:
            printOk("Could not find anyone in front of the robot. It will just look in front and up.")
            return 'failed'


# ----------------------------------------------------------------------------------------------------

class CancelHeadGoals(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['done'])
        self.robot = robot

    def execute(self, robot):
        printOk("CancelHeadGoals")

        self.robot.head.cancel_goal()

        return 'done'


# ----------------------------------------------------------------------------------------------------


class FindCrowd(smach.State):
    def __init__(self, robot, locations):
        smach.State.__init__(   self, outcomes=['succeded', 'failed'])
        self.robot = robot
        self.locations = locations

    def execute(self, userdata):
        printOk("FindCrowd")
        
        foundFace = False
        centerPointRes = None
        humanDesignatorRes = None
        entityDataRes = None
        

        # create designators
        humanDesignator = EdEntityCollectionDesignator(self.robot, criteriafuncs=[lambda entity: entity.type in ["crowd", "human"]])
        centerPointDes = AttrDesignator(humanDesignator, 'center_point')

        self.robot.head.cancel_goal()

        # "scan" the room with the head
        # look at 3 meters front, 5 meters right and 2 meters high
        self.robot.head.look_at_point(point_stamped=msgs.PointStamped(3,-3,2,self.robot.robot_name+"/base_link"), end_time=0, timeout=8)
        rospy.sleep(2)

        self.robot.head.look_at_point(point_stamped=msgs.PointStamped(3,0,2,self.robot.robot_name+"/base_link"), end_time=0, timeout=8)
        rospy.sleep(3)

        # look at 3 meters front, 5 meters left and 2 meters high
        self.robot.head.look_at_point(point_stamped=msgs.PointStamped(3,3,2,self.robot.robot_name+"/base_link"), end_time=0, timeout=8)
        rospy.sleep(2)

        # extra sleep to give more time to process
        rospy.sleep(2)

        self.robot.head.cancel_goal()

        # resolve designator
        humanDesignatorRes = humanDesignator.resolve()
        if not humanDesignatorRes:
            printOk("Could not resolve humanDesignator")
            return 'failed'


        # if humanDesignatorRes is not empty
        if not humanDesignatorRes == None:
            printOk("Iterating through the {0} humans found".format(len(humanDesignatorRes)))

            for humanEntity in humanDesignatorRes:
                faceList = None
                try:
                    faceList = humanEntity.data['perception_result']['face_detector']['faces_front']
                    printOk("Found {0} faces in this entity".format(len(faceList)))
                except KeyError, ke:
                    # import ipdb; ipdb.set_trace()
                    printOk("Could not resolve humanEntity.data[...]:" + str(ke))
                    pass

                # faceList = [human['perception_result']['face_detector']['faces_front'] for human in humanDesignatorRes]

                if not faceList == None:
                    for face in faceList:

                        alreadyExists = False
                        for loc in self.locations.current:
                            p1 = (face["map_x"], face["map_y"], face["map_z"])
                            p2 = (loc.point_stamped.point.x, loc.point_stamped.point.y, loc.point_stamped.point.z)

                            if points_distance(p1=p1, p2=p2) < 0.1:
                                alreadyExists = True
                                break

                        if not alreadyExists:
                            self.locations.current += [Location(point_stamped = msgs.PointStamped(x=face["map_x"], y=face["map_y"], z=face["map_z"], frame_id="/map"),
                                                                visited = False,
                                                                attempts = 0)]

                            printOk("Added face location to the list: ({0}, {1}, {2})".format(face["map_x"], face["map_y"], face["map_z"]))
                        else:
                            printOk("Location already exists in the list")

                else:
                    printWarning("Designator resolved but no faces where found")

                    # resolve data from entity
                    # try:
                    #     # centerPointRes = humanEntity.center_point
                    #     centerPointRes = None
                    # except KeyError, ke:
                    #     printOk("Could not resolve humanEntity.center_point" + str(ke))
                    #     pass

                    # if not centerPointRes == None:
                    #     alreadyExists = False
                    #     for loc in self.locations.current:
                    #         p1 = (centerPointRes.x, centerPointRes.y, centerPointRes.z)
                    #         p2 = (loc.point_stamped.point.x, loc.point_stamped.point.y, loc.point_stamped.point.z)

                    #         if points_distance(p1=p1, p2=p2) < 0.1:
                    #             alreadyExists = True
                    #             break

                    #     if not alreadyExists:
                    #         self.locations.current += [Location(point_stamped = msgs.PointStamped(x=centerPointRes.x, y=centerPointRes.y, z=centerPointRes.z, frame_id="/map"),
                    #                                             visited = False,
                    #                                             attempts = 0)]

                    #         printOk("Added center_point to the list: ({0}, {1}, {2})".format(centerPointRes.x, centerPointRes.y, centerPointRes.z))
                    #     else:
                    #         printOk("Location closeby already exists in the list")


            printOk("Face location list has " + str(len(self.locations.current)) + " entries")

            # if the number of found faces is not enough, continue searching
            if len(self.locations.current) >= challenge_knowledge.min_faces_found:
                return 'succeded'
            else:
                return 'failed'                

        else:
            printWarning("Could not find anyone in the room.")

        return 'failed'


# ----------------------------------------------------------------------------------------------------


class DescribePeople(smach.State):
    def __init__(self, robot, facesAnalyzedDes):
        smach.State.__init__(   self, 
                                input_keys=['operatorIdx_in'],
                                outcomes=['done'])
        self.robot = robot
        self.facesAnalyzedDes = facesAnalyzedDes

    def execute(self, userdata):
        printOk("DescribePeople")

        numberMale = 0
        numberFemale = 0
        faceList = None

        # try to resolve the crowd designator
        faceList = self.facesAnalyzedDes.resolve()
        if not faceList:
            printFail("Could not resolve facesAnalyzedDes")
            pass

        if not faceList == None:
            # Compute crowd details

            for face in faceList:
                printOk("Name: {0}, Score: {1}, Location: ({2},{3},{4}), Pose: {5}, Gender: {6}, Main crowd: {7}".format(
                    str(face.name),
                    str(face.score),
                    str(face.point_stamped.point.x), str(face.point_stamped.point.y), str(face.point_stamped.point.z),
                    str(face.pose),
                    str(face.gender),
                    str(face.inMainCrowd)))

                if face.inMainCrowd == True:
                    if face.gender == challenge_knowledge.Gender.Male:
                        numberMale += 1
                    else:
                        numberFemale += 1

            self.robot.speech.speak("I counted {0} persons in this crowd. {1} males and {2} females.".format(
                str(numberMale + numberFemale),
                numberMale if numberMale > 0 else "no",
                numberFemale if numberFemale > 0 else "no"),
                block=False)

            try:
                if userdata.operatorIdx_in == None:
                    printFail("Operator index from the list is invalid.")
                    self.robot.speech.speak("I could not find my operator among the people I searched for.", block=False)
                else:
                    self.robot.speech.speak("My operator is a {gender}, and {pronoun} is {pose}.".format(
                        gender = "man" if faceList[userdata.operatorIdx_in].gender == challenge_knowledge.Gender.Male else "woman",
                        pronoun = "he" if faceList[userdata.operatorIdx_in].gender == challenge_knowledge.Gender.Male else "she",
                        pose =  "standing up" if faceList[userdata.operatorIdx_in].pose == challenge_knowledge.Pose.Standing else "sitting down"),
                        block=True)
            except KeyError, ke:
                    printOk("KeyError userdata.operatorIdx_in:" + str(ke))
                    pass

        else:
            self.robot.speech.speak("I could not find any faces during the challenge.",  block=False)

        return 'done'



# ----------------------------------------------------------------------------------------------------


class PointAtOperator(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot

    def execute(self, robot):
        printOk("PointAtOperator")

        # Get information about the operator and point at the location
        self.robot.rightArm.send_goal(0.5, -0.2, 0.9, 0, 0, 0, 60)

        return 'succeeded'


# ----------------------------------------------------------------------------------------------------


# Ask the person's name
class AskPersonName(smach.State):
    def __init__(self, robot, operatorNameDes):
        smach.State.__init__(   self, 
                                outcomes=['succeded', 'failed'])

        self.robot = robot
        self.operatorNameDes = operatorNameDes

    def execute(self, userdata):
        printOk("AskPersonName")

        self.robot.speech.speak("What is your name?", block=False)

        spec = Designator("((<prefix> <name>)|<name>)")

        choices = Designator({"name"  : common_knowledge.names,
                              "prefix": ["My name is", "I'm called", "I am"]})

        answer = VariableDesignator(resolve_type=GetSpeechResponse)

        state = HearOptionsExtra(self.robot, spec, choices, answer)
        outcome = state.execute()

        if not outcome == "heard":
            name = "Mister Operator"
            self.operatorNameDes.current = name

            printWarning("Speech recognition outcome was not successful (outcome: '{0}'). Using default name '{1}'".format(str(outcome), self.operatorNameDes.resolve()))
            return 'failed'
        else:
            try:
                name = answer.resolve().choices["name"]
                self.operatorNameDes.current = name

                printOk("Result received from speech recognition is '" + name + "'")
            except KeyError, ke:
                printOk("KeyError resolving the name heard: " + str(ke))
                pass

        return 'succeded'


# ----------------------------------------------------------------------------------------------------


# Confirm person's name
class ConfirmPersonName(smach.State):
    def __init__(self, robot, operatorNameDes):
        smach.State.__init__(   self, 
                                outcomes=['correct', 'incorrect'])

        self.robot = robot

    def execute(self, userdata):
        printOk("ConfirmPersonName")

        self.operatorNameDes.current = name

        self.robot.speech.speak("I heard " + name + ", is that correct?", block=False)


        spec = Designator("answer")

        choices = Designator({"anser": ["yes", "correct", "right", "that is correct"]})

        answer = VariableDesignator(resolve_type=GetSpeechResponse)

        state = HearOptionsExtra(self.robot, spec, choices, answer)
        outcome = state.execute()

        if not outcome == "heard":
            name = "Mister Operator Fallback"
            self.operatorNameDes.current = name

            printWarning("Speech recognition outcome was not successful (outcome: " + str(outcome) + ")")
            return 'incorrect'
        else:
            self.robot.speech.speak("Ok!", block=False)

        return 'correct'
        

# ----------------------------------------------------------------------------------------------------


class GetOperatorLocation(smach.State):
    def __init__(self, robot, facesAnalysedDes, operatorNameDes, operatorLocationDes):
        smach.State.__init__(   self, 
                                output_keys=['operatorIdx_out'],
                                outcomes=['succeeded', 'failed'])

        self.robot = robot
        self.facesAnalysedDes = facesAnalysedDes
        self.operatorLocationDes = operatorLocationDes
        self.operatorNameDes = operatorNameDes

    def execute(self, userdata):
        printOk("GetOperatorLocation")

        lowest_score = 1    # scores are between 0 and 1
        chosenOperator = False
        faceList = None

        # try to resolve the crowd designator
        faceList = self.facesAnalysedDes.resolve()
        if not faceList:
            printOk("Could not resolve faces analysed")
            return 'failed'

        # import ipdb; ipdb.set_trace()

        if not faceList == None:
            for idx, face in enumerate(faceList):
                printOk("Name: {0}, Score: {1}, Location: ({2},{3},{4})".format(
                    str(face.name),
                    str(face.score),
                    str(face.point_stamped.point.x), str(face.point_stamped.point.y), str(face.point_stamped.point.z)))

                # score of 0 is for unidentifies people
                if face.score > 0 and face.score < lowest_score and self.operatorNameDes.resolve() == face.name:
                    lowest_score = face.score
                    operatorIdx = idx
                    self.operatorLocationDes.current.setPoint(point_stamped = msgs.PointStamped(x=face.point_stamped.point.x, y=face.point_stamped.point.y, z=face.point_stamped.point.z, frame_id="/map"))
                    chosenOperator = True

            # If no operator was choosen, select a random one
            if not chosenOperator:
                # operatorIdx = 0
                operatorIdx = random.randint(0, len(faceList)-1)
                chosenOperator = True
                printWarn("Could not choose an operator. Selecting random index: " + operatorIdx)

            if chosenOperator:
                printOk("Operator is: {0} ({1}), Location: ({2},{3},{4})".format(
                    str(faceList[operatorIdx].name),
                    str(faceList[operatorIdx].score),
                    str(faceList[operatorIdx].point_stamped.point.x), str(faceList[operatorIdx].point_stamped.point.y), str(faceList[operatorIdx].point_stamped.point.z)))

                # Operators face location
                # p1 = (faceList[operatorIdx].point_stamped.point.x, faceList[operatorIdx].point_stamped.point.y, faceList[operatorIdx].point_stamped.point.z)

                # # Update who belongs to the main crowd, close to the operator
                # for face in faceList:
                #     p2 = (face.point_stamped.point.x, face.point_stamped.point.y, face.point_stamped.point.z)

                #     if points_distance(p1=p1, p2=p2) < 5.0:
                #         face.inMainCrowd = True

                userdata.operatorIdx_out = operatorIdx
                return 'succeeded'
            else:
                userdata.operatorIdx_out = 0

                printFail("Could not choose an operator from the list! Selecting the first.")
                return 'failed'
        else:
            printFail("No faces were analysed!")
            return 'failed'


# ----------------------------------------------------------------------------------------------------


class AnalysePerson(smach.State):
    def __init__(self, robot, facesAnalysedDes):
        smach.State.__init__(self, outcomes=['succeded', 'failed'])

        self.robot = robot
        self.facesAnalysedDes = facesAnalysedDes

    def execute(self, userdata):
        printOk("AnalysePerson")

        # initialize variables
        entityDataList = []
        recognition_label = None
        recognition_score = None
        humanDesignatorRes = None

        # create designators
        humanDesignator = EdEntityCollectionDesignator(self.robot, criteriafuncs=[lambda entity: entity.type in ["crowd", "human"]])
        humanDataDesignator = AttrDesignator(humanDesignator, 'data')

        humanDesignatorRes = humanDesignator.resolve()
        if not humanDesignatorRes:
            printOk("Could not resolve humanDesgnResult.")
            pass

        if not humanDesignatorRes == None:
            printOk("Iterating through the {0} humans found".format(len(humanDesignatorRes)))

            for humanEntity in humanDesignatorRes:

                try:
                    # import ipdb; ipdb.set_trace()
                    # faceList = [human.data['perception_result']['face_recognizer']['face'] for human in humanDesignatorRes][0]
                    faceList = humanEntity.data['perception_result']['face_recognizer']['face']

                    # faceList = humanEntity.data['perception_result']['face_recognizer']['face']
                    printOk("Found {0} faces in this entity".format(len(faceList)))
                    printOk(str(faceList))

                    for faceInfo in faceList:

                        recognition_label = faceInfo['label']
                        recognition_score = faceInfo['score']
                        face_idx = faceInfo['index']

                        # initialize label as empty string if the recogniton didnt conclude anything
                        if recognition_score == 0:
                            recognition_label = ""
                            printOk("Unrecognized person")


                        # if entityData['type'] == "crowd":
                            #  get the corresponding location of the face
                        for face_detector_loc in humanEntity.data['perception_result']['face_detector']['faces_front']:
                            if face_idx == face_detector_loc['index']:
                                # get location
                                face_loc = face_detector_loc
                                break

                        # elif entityData['type'] == "human":
                        #     face_loc = entityData['perception_result']['face_detector']['faces_front'][0]
                        # else:
                        #     print OUT_PREFIX + bcolors.FAIL + "Uknown entity type. Unable to get face location" + bcolors.ENDC

                        #  test if a face in this location is already present in the list
                        sameFace = False
                        for face in self.facesAnalysedDes.current:
                            p1 = (face_loc["map_x"], face_loc["map_y"], face_loc["map_z"])
                            p2 = (face.point_stamped.point.x,face.point_stamped.point.y, face.point_stamped.point.z)

                            if points_distance(p1=p1, p2=p2) < 0.1:
                                sameFace = True
                                break

                        if sameFace:
                            # TODO: COMPARE SCORES FROM THE FACE RECOGNITION, IF ITS BETTER, REPLACE
                            printOk("Face already present in the list. List size: " + str(len(self.facesAnalysedDes.current)))

                        #  if information is valid, add it to the list of analysed faces
                        # if not recognition_label == None and not recognition_score == None and not sameFace:
                        if not sameFace:

                            # "predict" pose in a hacky way
                            if face_loc["map_z"] > 0.8:
                                pose = challenge_knowledge.Pose.Standing
                            else:
                                pose = challenge_knowledge.Pose.Sitting_down

                            #  "predict" gender, in a hacky way
                            if recognition_label[:-1] == 'a':
                                personGender = challenge_knowledge.Gender.Female
                            else:
                                personGender = challenge_knowledge.Gender.Male

                            printOk("Adding face to list: '{0}' (score:{1}, pose: {2}) @ ({3},{4},{5})".format(
                                str(recognition_label),
                                str(recognition_score),
                                "standing up" if pose == challenge_knowledge.Pose.Standing else "sitting down",
                                face_loc["map_x"], face_loc["map_y"], face_loc["map_z"]))

                            self.facesAnalysedDes.current += [(FaceAnalysed(point_stamped = msgs.PointStamped(x=face_loc["map_x"], y=face_loc["map_y"], z=face_loc["map_z"], frame_id="/map"),
                                                                            name = recognition_label, 
                                                                            score = recognition_score,
                                                                            pose = pose,
                                                                            gender = personGender))]
                        else:
                            printWarning("Did not add face to the list")

                except KeyError, ke:
                    printError("KeyError faceList:" + str(ke))
                    pass

            return 'succeded'
        else:
            printOk("Could not find anyone in front of the robot.")
            return 'failed'


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
        printOk("GetNextLocation")

        availableLocations = self.locations.resolve()

        # if there are locations not visited yet, choose one
        if availableLocations:
            printOk(str(len(availableLocations)) + " locations in the list")

            chosenLocation = None
            # TODO: CHOOSE CLOSEST LOCATION
            for loc in availableLocations:

                if loc.visited == False:
                    if loc.attempts < 5:
                        chosenLocation = loc.point_stamped
                        loc.attempts += 1
                        break
                    else:
                        printOk('Tried to visit this location several times and it never worked. Ignoring it')

            if not chosenLocation:
                printOk('All locations are marked as visited')
                return 'visited_all'
            else:
                self.nextLocation.current.setPoint(point_stamped = chosenLocation)
                printOk("Next location: " + str(chosenLocation.point))

                return 'done'

        # If there are not more loactions to visit then report back to the operator
        else:
            printOk("Visited all locations")
            return 'visited_all'


# ----------------------------------------------------------------------------------------------------


class ResetSearch(smach.State):
    """
    Reset variables related to the operator searching
    """
    def __init__(self, robot, locations):
        smach.State.__init__(   self, outcomes=['done'])

        self.robot = robot
        self.locations = locations
        
    def execute(self, userdata):
        printOk("ResetSearch")

        # Reset locations to visit
        self.locations.current = []
        
        return 'done'
