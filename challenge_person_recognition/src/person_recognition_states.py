#! /usr/bin/env python
import roslib;
import rospy
import smach
import subprocess
import inspect
import random
import ed_perception.msg
import robot_skills.util.msg_constructors as msgs
import math
from smach_ros import SimpleActionState
from robot_smach_states.util.designators import *
from robot_smach_states.human_interaction.human_interaction import HearOptionsExtra
from ed.msg import EntityInfo
from dragonfly_speech_recognition.srv import GetSpeechResponse
from robocup_knowledge import load_knowledge
from robot_skills.util import transformations

# ----------------------------------------------------------------------------------------------------

common_knowledge = load_knowledge("common")
challenge_knowledge = load_knowledge("challenge_person_recognition")
OBJECT_TYPES = challenge_knowledge.object_types

# rename calls to make them shorter and colorfull
def printOk(sentence):
    challenge_knowledge.printOk(sentence)

def printError(sentence):
    challenge_knowledge.printError(sentence)

def printWarning(sentence):
    challenge_knowledge.printWarning(sentence)


# ----------------------------------------------------------------------------------------------------


class Location(object):
    """ class to store location of faces """
    def __init__(self, point_stamped, visited, attempts):
        self.point_stamped = point_stamped
        self.visited = visited
        self.attempts = attempts


# ----------------------------------------------------------------------------------------------------


class FaceAnalysed(object):
    """ class to store characteristics of a face """
    def __init__(   self, 
                    point_stamped, 
                    name, 
                    score, 
                    pose = challenge_knowledge.Pose.Standing, 
                    gender = challenge_knowledge.Gender.Male, 
                    inMainCrowd = True, 
                    orderedPosition = "",
                    operator = False):
        self.point_stamped = point_stamped
        self.name = name
        self.score = score
        self.pose = pose
        self.gender = gender
        self.inMainCrowd = inMainCrowd
        self.orderedPosition = orderedPosition
        self.operator = operator

    def __repr__(self):
        return "Name: {0}, Score: {1}, Location: ({2},{3},{4}), Pose: {5}, Gender: {6}, Main crowd: {7}, Position: {8}".format(
                    str(self.name),
                    str(self.score),
                    str(self.point_stamped.point.x), str(self.point_stamped.point.y), str(self.point_stamped.point.z),
                    str(self.pose),
                    str(self.gender),
                    str(self.inMainCrowd),
                    str(self.orderedPosition))


# ----------------------------------------------------------------------------------------------------


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


# ----------------------------------------------------------------------------------------------------


def points_distance(p1, p2):
    # printOk("points_distance")

    deltaX = p2[0] - p1[0]
    deltaY = p2[1] - p1[1]
    deltaZ = p2[2] - p1[2]

    distance = math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ)
    # printOk("Distance ({0},{1},{2}) -> ({3},{4},{5}) = {6}".format(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], distance))

    return distance


# ----------------------------------------------------------------------------------------------------


def scanForHuman(robot):
    printOk("scanForHuman")

    entity_list = []

    ''' Enable kinect segmentation plugin (only one image frame) '''
    entities_found = robot.ed.segment_kinect(max_sensor_range=3)

    printOk("Found {0} entities".format(len(entities_found)))

    ''' Get all entities that are returned by the segmentation and are on top of the shelf '''
    id_list = []                
    for entity_id in entities_found:
        ''' get the entity from the ID '''
        entity = robot.ed.get_entity(entity_id)

        if entity:
            id_list.append(entity.id)

    ''' Try to classify the entities '''
    entity_types = robot.ed.classify(ids=id_list, types=OBJECT_TYPES)

    ''' Check all entities that were flagged to see if they have received a 'type' it_label
        if so: recite them and lock them '''
    for i in range(0, len(id_list)):
        e_id = id_list[i]
        e_type = entity_types[i]

        if e_type:
            if e_type == "human":
                entity = robot.ed.get_entity(e_id)
                # entity.data

                printOk("Entity with type " + e_type + " added to the list (id " + e_id + ")")
                # robot.ed.update_entity(id=e_id, flags=[{"add": "locked"}])

                entity_list = entity_list + [entity]

            else:
                printOk("Entity with type '" + e_type + "' ignored")


    # import ipdb; ipdb.set_trace()

    return entity_list


# ----------------------------------------------------------------------------------------------------


class WaitForPerson(smach.State):
    def __init__(self, robot, attempts = 1, sleep_interval = 1):
        smach.State.__init__(self, outcomes=['succeded', 'failed'])
        self.robot = robot
        self.attempts = attempts
        self.sleep_interval = sleep_interval

    def execute(self, userdata=None):
        printOk("WaitForPerson")

        counter = 0
        desgnResult = None

        while counter < self.attempts:
            print "WaitForPerson: waiting {0}/{1}".format(counter, self.attempts)

            desgnResult = scanForHuman(self.robot)
            if desgnResult:
                printOk("Found a human!")
                return 'succeded'

            counter += 1
            rospy.sleep(self.sleep_interval)

        return 'failed'


# ----------------------------------------------------------------------------------------------------


class LookAtPersonInFront(smach.State):
    def __init__(self, robot, lookDown = False):
        smach.State.__init__(self, outcomes=['succeded', 'failed'])
        self.robot = robot
        self.lookDown = lookDown

    def execute(self, userdata=None):
        printOk("LookAtPersonInFront")

        # initialize variables
        foundFace = False
        result = None
        entityData = None
        faces_front = None
        desgnResult = None

        self.robot.head.cancel_goal()

        # look front, 2 meters high
        self.robot.head.look_at_point(point_stamped=msgs.PointStamped(3, 0, 1.5,self.robot.robot_name+"/base_link"), end_time=0, timeout=4)
        rospy.sleep(1)  # give time for the percetion algorithms to add the entity

        desgnResult = scanForHuman(self.robot)
        if not desgnResult:
            printOk("Could not find a human while looking up")

        # if no person was seen at 2 meters high, look down, because the person might be sitting
        if not desgnResult and self.lookDown == True:
            # look front, 2 meters high
            self.robot.head.look_at_point(point_stamped=msgs.PointStamped(3, 0, 0,self.robot.robot_name+"/base_link"), end_time=0, timeout=4)

            # try to resolve the designator
            desgnResult = scanForHuman(self.robot)
            if not desgnResult:
                printWarning("Could not find a human while looking down")
                pass

        # if there is a person in front, try to look at the face
        if desgnResult:
            printOk("Designator resolved a Human!")

            # extract information from data
            faces_front = None
            try:
                # import ipdb; ipdb.set_trace()
                # get information on the first face found (cant guarantee its the closest in case there are many)
                faces_front = desgnResult[0].data["perception_result"]["face_detector"]["faces_front"][0]
            except KeyError, ke:
                printWarning("KeyError faces_front: " + str(ke))
                pass
            except IndexError, ke:
                printWarning("IndexError faces_front: " + str(ke))
                pass
            except TypeError, ke:
                printWarning("TypeError faces_front: " + str(ke))
                pass

            if faces_front:
                headGoal = msgs.PointStamped(x=faces_front["map_x"], y=faces_front["map_y"], z=faces_front["map_z"], frame_id="/map")

                printOk("Sending head goal to (" + str(headGoal.point.x) + ", " + str(headGoal.point.y) + ", " + str(headGoal.point.z) + ")")
                self.robot.head.look_at_point(point_stamped=headGoal, end_time=0, timeout=4)

                foundFace == True            
            else:
                printWarning("Found a human but no faces.")
                foundFace == False 

        if foundFace == True:
            return 'succeded'
        else:
            printWarning("Could not find anyone in front of the robot. It will just look forward.")
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


        faces_locations = []

        # points where the head will look at
        head_points_stamped = [ msgs.PointStamped(3,-3,1, self.robot.robot_name + "/base_link"),
                                msgs.PointStamped(3,0,1, self.robot.robot_name + "/base_link"),
                                msgs.PointStamped(3,3,1, self.robot.robot_name + "/base_link")]

        # clear head goals
        self.robot.head.cancel_goal()

        # find human entities while turning the head
        for head_point in head_points_stamped:
            self.robot.head.look_at_point(point_stamped=head_point, end_time=0, timeout=8)
            
            ''' sleep to prevent head movement while scanning for humans'''
            rospy.sleep(1)

            entity_list = scanForHuman(self.robot)
            if not entity_list:
                printWarning("Did not find any humans")
                pass
            else:
                # import ipdb; ipdb.set_trace()
                faces_locations = faces_locations + entity_list
                printOk("Found {0} humans. Adding to list, now with {1} (before filtering)".format(len(entity_list), len(faces_locations)))


        # clear head gloas
        self.robot.head.cancel_goal()

        # if humanDesignatorRes is not empty
        if faces_locations:
            printOk("Iterating through the {0} faces found".format(len(faces_locations)))

            for humanEntity in faces_locations:
                faceList = []
                try:
                    # faces_front = desgnResult[0].data["perception_result"]["face_detector"]["faces_front"][0]
                    faceList = humanEntity.data['perception_result']['face_detector']['faces_front']
                    printOk("Found {0} faces in this entity".format(len(faceList)))
                except KeyError, ke:
                    printError("Could not resolve humanEntity.data[...]:" + str(ke))
                    pass
                except TypeError, te:
                    printError("Could not resolve humanEntity.data[...]:" + str(te))
                    # import ipdb; ipdb.set_trace()
                    pass

                ''' iterate through all the faces in this entity '''
                if faceList:
                    for face in faceList:

                        ''' check if there is already a face in this location in the list, to avoid duplicates '''
                        alreadyExists = False
                        for loc in self.locations.current:
                            p1 = (face["map_x"], face["map_y"], face["map_z"])
                            p2 = (loc.point_stamped.point.x, loc.point_stamped.point.y, loc.point_stamped.point.z)

                            if points_distance(p1=p1, p2=p2) < challenge_knowledge.face_proximity_treshold:
                                printOk ("Too close to another face: " + str(points_distance(p1=p1, p2=p2)))
                                alreadyExists = True
                                break

                        if not alreadyExists:
                            self.locations.current += [Location(point_stamped = msgs.PointStamped(x=face["map_x"], y=face["map_y"], z=face["map_z"], frame_id="/map"),
                                                                visited = False,
                                                                attempts = 0)]

                            printOk("\tAdded face location to the list: ({0}, {1}, {2})".format(face["map_x"], face["map_y"], face["map_z"]))
                        else:
                            printOk("Face location already exists in the list")

                else:
                    printWarning("Designator resolved but no faces where found")


            printOk("Found " + str(len(self.locations.current)) + " faces")

            ''' if the number of found faces is not enough, continue searching '''
            if len(self.locations.current) >= challenge_knowledge.min_faces_found:
                return 'succeded'
            else:
                return 'failed'                

        else:
            printWarning("Could not find anyone in the room.")

        return 'failed'


# ----------------------------------------------------------------------------------------------------

class DescribePeople(smach.State):
    def __init__(self, robot, facesAnalyzedDes, operatorNameDes):
        smach.State.__init__(   self, 
                                input_keys=['operatorIdx_in'],
                                outcomes=['done'])
        self.robot = robot
        self.facesAnalyzedDes = facesAnalyzedDes
        self.operatorNameDes = operatorNameDes
        self.first_time = True
        self.numberMale = 0
        self.numberFemale = 0
        self.operatorIdx = 0

    def execute(self, userdata):
        printOk("DescribePeople")

        faceList = None
        position = 1

        # try to resolve the crowd designator
        faceList = self.facesAnalyzedDes.resolve()
        if not faceList:
            printFail("Could not resolve facesAnalyzedDes")
            pass

        if not faceList == None:

            # there's a bug somewhere, if the function is ran twice it presents a different descriptio, so... fixed for now
            if self.first_time == True:
                self.first_time = False
                # import ipdb; ipdb.set_trace()

                # convert map frame to base_link frame
                for face in faceList:
                    in_map = msgs.PointStamped(point=face.point_stamped.point, frame_id="/map")
                    in_base_link = transformations.tf_transform(in_map, "/map", "/"+self.robot.robot_name+"/base_link", self.robot.tf_listener)
                    face.point_stamped.point.x = in_base_link.x
                    face.point_stamped.point.y = in_base_link.y
                    face.point_stamped.point.z = in_base_link.z

                # order list by face Y location wrt base_link
                faceList.sort(key=lambda k: k.point_stamped.point.y)

                # Compute crowd and operator details
                for idx, face in enumerate(faceList):
                    printOk("Name: {0}, Score: {1}, Location: ({2},{3},{4}), Pose: {5}, Gender: {6}, Main crowd: {7}, Position: {8}, Operator: {9}".format(
                        str(face.name),
                        str(face.score),
                        str(face.point_stamped.point.x), str(face.point_stamped.point.y), str(face.point_stamped.point.z),
                        str(face.pose),
                        str(face.gender),
                        str(face.inMainCrowd),
                        str(face.orderedPosition),
                        str(face.operator)))

                    # count number of males and females
                    if face.inMainCrowd == True:
                        if face.gender == challenge_knowledge.Gender.Male:
                            self.numberMale += 1
                        else:
                            self.numberFemale += 1

                    # translate position number to word
                    if idx + 1 == 1 : face.orderedPosition = "first"
                    if idx + 1 == 2 : face.orderedPosition = "second"
                    if idx + 1 == 3 : face.orderedPosition = "third"
                    if idx + 1 == 4 : face.orderedPosition = "fourth"
                    if idx + 1 == 5 : face.orderedPosition = "fifth"
                    if idx + 1 == 6 : face.orderedPosition = "sixth"
                    if idx + 1 == 7 : face.orderedPosition = "seventh"
                    if idx + 1 == 8 : face.orderedPosition = "eighth"
                    if idx + 1 == 9 : face.orderedPosition = "ninth"
                    if idx + 1 == 10 : face.orderedPosition = "tenth"

                    if face.operator == True:
                        self.operatorIdx = idx


            self.robot.speech.speak("I counted {0} persons in this crowd. {1} males and {2} females.".format(
                str(self.numberMale + self.numberFemale),
                self.numberMale if self.numberMale > 0 else "no",
                self.numberFemale if self.numberFemale > 0 else "no"),
                block=False)

            try:
                # just to be safe, test this...
                if self.operatorIdx < 0 or self.operatorIdx >= len(faceList):
                    printFail("Operator index from the list is invalid. (" + self.operatorIdx + "). Reseting to 0")
                    self.operatorIdx = 0
                
                self.robot.speech.speak("My operator is a {gender}, and {pronoun} is {pose}. {name} is the {order} person in the crowd, starting from my right.".format(
                    name = self.operatorNameDes.resolve(),
                    order = faceList[self.operatorIdx].orderedPosition,
                    gender = "man" if faceList[self.operatorIdx].gender == challenge_knowledge.Gender.Male else "woman",
                    pronoun = "he" if faceList[self.operatorIdx].gender == challenge_knowledge.Gender.Male else "she",
                    pose =  "standing up" if faceList[self.operatorIdx].pose == challenge_knowledge.Pose.Standing else "sitting down"), block=True)

            except KeyError, ke:
                    printOk("KeyError self.operatorIdx:" + str(ke))
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

        is_writeable(operatorNameDes)
        self.operatorNameDes = operatorNameDes

    def execute(self, userdata):
        printOk("AskPersonName")

        self.robot.speech.speak("What is your name?", block=True)

        spec = Designator("((<prefix> <name>)|<name>)")

        choices = Designator({"name"  : common_knowledge.names,
                              "prefix": ["My name is", "I'm called", "I am"]})

        answer = VariableDesignator(resolve_type=GetSpeechResponse)

        state = HearOptionsExtra(self.robot, spec, choices, writeable(answer))
        outcome = state.execute()

        if not outcome == "heard":
            name = "Mister Operator"
            self.operatorNameDes.write(name)

            printWarning("Speech recognition outcome was not successful (outcome: '{0}'). Using default name '{1}'".format(str(outcome), self.operatorNameDes.resolve()))
            return 'failed'
        else:
            try:
                name = answer.resolve().choices["name"]
                self.operatorNameDes.write(name)

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

        self.operatorNameDes.write(name)

        self.robot.speech.speak("I heard " + name + ", is that correct?", block=False)


        spec = Designator("answer")

        choices = Designator({"anser": ["yes", "correct", "right", "that is correct"]})

        answer = VariableDesignator(resolve_type=GetSpeechResponse)

        state = HearOptionsExtra(self.robot, spec, choices, answer)
        outcome = state.execute()

        if not outcome == "heard":
            name = "Mister Operator Fallback"
            self.operatorNameDes.write(name)

            printWarning("Speech recognition outcome was not successful (outcome: " + str(outcome) + ")")
            return 'incorrect'
        else:
            self.robot.speech.speak("Ok!", block=False)

        return 'correct'


# ----------------------------------------------------------------------------------------------------


class ChooseOperator(smach.State):
    def __init__(self, robot, facesAnalysedDes, operatorNameDes, operatorLocationDes):
        smach.State.__init__(   self, 
                                output_keys=['operatorIdx_out'],
                                outcomes=['succeeded', 'failed'])

        self.robot = robot
        self.facesAnalysedDes = facesAnalysedDes
        self.operatorLocationDes = operatorLocationDes
        self.operatorNameDes = operatorNameDes

    def execute(self, userdata):
        printOk("ChooseOperator")

        lowest_score = 1    # scores are between 0 and 1
        chosenOperator = False
        faceList = None

        faceList = self.facesAnalysedDes.resolve()
        if not faceList:
            printOk("Could not resolve faces analysed")
            return 'failed'

        # import ipdb; ipdb.set_trace()

        if not faceList == None:            
            ''' iterate through all the faces recognized and choose the best one as operator '''
            for idx, face in enumerate(faceList):
                printOk("\tName: {0}, Score: {1}, Location: ({2},{3},{4})".format(
                    str(face.name),
                    str(face.score),
                    str(face.point_stamped.point.x), str(face.point_stamped.point.y), str(face.point_stamped.point.z)))

                # Low scores are better. Score=0 means face not identified
                if face.score > 0 and face.score < lowest_score and self.operatorNameDes.resolve() == face.name:
                    lowest_score = face.score
                    operatorIdx = idx
                    self.operatorLocationDes.current.setPoint(point_stamped = msgs.PointStamped(x=face.point_stamped.point.x, 
                                                                                                y=face.point_stamped.point.y, 
                                                                                                z=face.point_stamped.point.z, 
                                                                                                frame_id="/map"))
                    chosenOperator = True

            ''' if for some reason the operator could not be choosen, just select the first one in the list ''' 
            if not chosenOperator:
                printWarning("Could not choose an operator, i was looking for " + self.operatorNameDes.resolve() + ". Selecting the first one in the list!")
                operatorIdx = 0
                self.operatorLocationDes.current.setPoint(point_stamped = msgs.PointStamped(x=faceList[operatorIdx].point_stamped.point.x, 
                                                                                            y=faceList[operatorIdx].point_stamped.point.y, 
                                                                                            z=faceList[operatorIdx].point_stamped.point.z, 
                                                                                            frame_id="/map"))

                
                # rand_op_idx = random.randint(0, len(faceList)-1)
                # printWarning("Could not choose an operator! Selecting random index: " + str(rand_op_idx))
                # If no operator was choosen, select a random one
                # userdata.operatorIdx_out = rand_op_idx

            ''' Updated list and userdata'''
            userdata.operatorIdx_out = operatorIdx
            faceList[operatorIdx].operator = True

            printOk("Operator is: {0} ({1}), Location: ({2},{3},{4})".format(
                str(faceList[operatorIdx].name),
                str(faceList[operatorIdx].score),
                str(faceList[operatorIdx].point_stamped.point.x), str(faceList[operatorIdx].point_stamped.point.y), str(faceList[operatorIdx].point_stamped.point.z)))

            return 'succeeded'

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
        # humanDesignator = EdEntityCollectionDesignator(self.robot, criteriafuncs=[lambda entity: entity.type in ["crowd", "human"]])
        # humanDataDesignator = AttrDesignator(humanDesignator, 'data')

        # humanDesignatorRes = humanDesignator.resolve()
        # if not humanDesignatorRes:
        #     printOk("Could not resolve humanDesgnResult.")
        #     pass

        
        humanDesignatorRes = scanForHuman(self.robot)
        if humanDesignatorRes:
            printOk("Iterating through the {0} humans found".format(len(humanDesignatorRes)))

            for humanEntity in humanDesignatorRes:

                try:
                    # import ipdb; ipdb.set_trace()
                    faceList = humanEntity.data['perception_result']['face_recognizer']['face']

                    printOk("Found {0} faces in this entity".format(len(faceList)))
                    printOk("\t" + str(faceList))

                    for faceInfo in faceList:

                        recognition_label = faceInfo['label']
                        recognition_score = faceInfo['score']
                        face_idx = faceInfo['index']

                        # initialize label as empty string if the recogniton didnt conclude anything
                        if recognition_score == 0:
                            recognition_label = "unknown"
                            printOk("Unrecognized face")

                        #  get the location of the face being currently analyzed
                        for face_detector_loc in humanEntity.data['perception_result']['face_detector']['faces_front']:
                            if face_idx == face_detector_loc['index']:
                                # get location
                                face_loc = face_detector_loc
                                break

                        # temporary, for debug output
                        shortest = 90

                        #  test if a face in this location is already present in the list
                        sameFace = False
                        p1 = (face_loc["map_x"], face_loc["map_y"], face_loc["map_z"])

                        for face in self.facesAnalysedDes.current:
                            p2 = (face.point_stamped.point.x,face.point_stamped.point.y, face.point_stamped.point.z)
                            
                            # just for testing
                            if points_distance(p1=p1, p2=p2) < shortest:
                                shortest = points_distance(p1=p1, p2=p2)

                            if points_distance(p1=p1, p2=p2) < challenge_knowledge.face_proximity_treshold:
                                printOk ("Too close to another face: " + str(points_distance(p1=p1, p2=p2)))
                                sameFace = True
                                break


                        #  if information is valid, add it to the list of analysed faces
                        if not sameFace:

                            printOk("Shortest distance to another face was: " + str(shortest))

                            printOk("Height of face: " + str(face_loc["map_z"]))

                            # "predict" pose in a hacky way
                            if face_loc["map_z"] > challenge_knowledge.sitting_height_treshold:
                                pose = challenge_knowledge.Pose.Standing
                            else:
                                pose = challenge_knowledge.Pose.Sitting_down

                            # TODO: match against the name list fiven by the knowledge
                            #  "predict" gender, in a hacky way. Names finished with A are female
                            
                            personGender = random.randint(0, 1)
                            # if recognition_label[-1:] == 'a':
                            #     personGender = challenge_knowledge.Gender.Female
                            # else:
                            #     personGender = challenge_knowledge.Gender.Male

                            printOk("\tAdding face to list: '{0}' (score:{1}, pose: {2}) @ ({3},{4},{5})".format(
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
                            printWarning("There is a face in this location already on the list.  List size: " + str(len(self.facesAnalysedDes.current)))

                except KeyError, ke:
                    printError("KeyError faceList:" + str(ke))
                    pass

            return 'succeded'
        else:
            printWarning("Could not find anyone in front of the robot.")
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
        self.locations.write([])
        
        return 'done'


# ----------------------------------------------------------------------------------------------------


class InitializeWorldModel(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.ed.configure_kinect_segmentation(continuous=False)
        self.robot.ed.configure_perception(continuous=False)
        self.robot.ed.disable_plugins(plugin_names=["laser_integration"])
        self.robot.ed.reset()

        return "done"


# ----------------------------------------------------------------------------------------------------

class TogglePerceptionMode(smach.State):

    def __init__(self, robot, toggle_mode=False):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot
        self.toggle_mode = toggle_mode

    def execute(self, userdata=None):
        printOk("TogglePerceptionMode")
        printOk("Continuos mode = " + str(self.toggle_mode))

        self.robot.ed.configure_kinect_segmentation(continuous=self.toggle_mode)
        self.robot.ed.configure_perception(continuous=self.toggle_mode)
        return "done"


# ----------------------------------------------------------------------------------------------------


class ResetEd(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):
        printOk("ResetEd")

        self.robot.ed.reset()

        return "done"
