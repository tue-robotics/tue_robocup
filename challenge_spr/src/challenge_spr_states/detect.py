#!/usr/bin/env python

import sys
import time
import smach
import math
import random
import rospy

import robot_smach_states as states
import robot_smach_states.util.designators as ds
import robot_skills.util.msg_constructors as msgs

from image_recognition_msgs.msg import FaceProperties
from robot_smach_states.util.startup import startup
from robot_skills.util.kdl_conversions import VectorStamped
from robot_skills.util.image_operations import img_recognitions_to_rois, img_cutout
from robocup_knowledge import load_knowledge
from tue_msgs.msg import People

timeout = 10


class DetectCrowd(smach.State):
    """
    Detect a crowd and describe it based on individual detections
    """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], output_keys=['crowd_data'])
        self.robot = robot
        self._people_sub = rospy.Subscriber(robot.robot_name + '/persons', People, self.people_cb, queue_size=1)
        self.people_received = People()
        self.triggered = False

    def people_cb(self, persons):
        if self.triggered and persons.people:
            if len(persons.people) > len(self.people_received.people):
                rospy.logdebug('Received %d persons in the people cb', len(persons.people))
                self.people_received = persons

    def execute(self, userdata=None):
        self.triggered = True
        tries = 3
        detections = self.recognize(tries)

        self.triggered = False
        crowd_data = self.describe_crowd(detections)
        userdata.crowd_data = crowd_data

        return 'succeeded'

    def recognize(self, tries=3):
        """
        Recognize people. Takes multiple images and takes the try with the most faces

        :param tries: number of tries
        :type tries: int
        :return: list of face properties
        :rtype: list[image_recognition_msgs/FaceProperties]
        """
        number_of_people = 0
        best_image = None
        best_detection = None

        sentences = ["You are all looking great today!            Keep looking in my camera!",
                     "I like it when everybody is staring at me; being in the center of attention!"]

        for i in range(0, tries):
            self.robot.speech.speak(sentences[i % (tries - 1)], block=False)
            self.robot.head.look_at_point(VectorStamped(6, 0, 0, self.robot.robot_name + "/base_link"))
            self.robot.head.wait_for_motion_done()
            rospy.sleep(1)

            image = self.robot.perception.get_image()
            face_rois = self.robot.perception.detect_faces(image=image)

            if len(face_rois) > number_of_people:
                number_of_people = len(face_rois)
                best_image = image
                best_detection = face_rois

        if best_image:  # at least one face detected
            faces = img_cutout(best_image, img_recognitions_to_rois(best_detection))

            rospy.loginfo('Processing faces...')
            return self.robot.perception.get_face_properties(faces=faces)
        else:
            rospy.logerr("No faces detected during all {} tries".format(tries))
            return []

    def describe_crowd(self, detections):
        """
        Conversion from individual face properties to crowd properties

        :param detections: list of face properties
        :type detections: list[image_recognition_msgs/FaceProperties]
        :return: crowd properties
        :rtype: dict
        """
        waving_persons = []
        rarm_persons = []
        larm_persons = []
        laying_persons = []
        sitting_persons = []
        lpointing_persons = []
        rpointing_persons = []
        num_males = 0
        num_men = 0
        num_boys = 0
        num_females = 0
        num_women = 0
        num_girls = 0
        num_elders = 0
        num_waving = 0
        num_rarm = 0
        num_larm = 0
        num_lpointing = 0
        num_rpointing = 0
        num_laying = 0
        num_sitting = 0

        for person in self.people_received.people:
                if {'RWave'}.intersection(set(person.tags)):
                    rarm_persons.append(person)
                elif {'LWave'}.intersection(set(person.tags)):
                    larm_persons.append(person)
                if {'RPointing'}.intersection(set(person.tags)):
                    rpointing_persons.append(person)
                elif {'LPointing'}.intersection(set(person.tags)):
                    lpointing_persons.append(person)
                if {'RLaying', 'LLaying'}.intersection(set(person.tags)):
                    laying_persons.append(person)
                elif {'RSitting', 'LSitting'}.intersection(set(person.tags)):
                    sitting_persons.append(person)

        num_rarm = len(rarm_persons)
        num_larm = len(larm_persons)
        num_waving = num_rarm + num_larm
        num_rpointing = len(rpointing_persons)
        num_lpointing = len(lpointing_persons)
        num_laying = len(laying_persons)
        num_sitting = len(sitting_persons)

        if not all(detections):
            rospy.loginfo('making a random guess for %d people', len(detections))
            if len(detections) > 2:
                num_males = 1 + random.randrange(len(detections) - 2)
            else:
                num_males = 1
            num_females = len(detections) - num_males
        else:
            for d in detections:
                if d.gender == FaceProperties.MALE:
                    if d.age < 18:
                        num_boys += 1
                        num_males += 1
                    elif d.age > 60:
                        num_elders += 1
                        num_males += 1
                    else:
                        num_men += 1
                        num_males += 1
                else:
                    if d.age < 18:
                        num_girls += 1
                        num_females += 1
                    elif d.age > 60:
                        num_elders += 1
                        num_females += 1
                    else:
                        num_women += 1
                        num_females += 1

        self.robot.speech.speak("There are %d males and %d females in the crowd" % (num_males, num_females))

        return {
            "males": num_males,
            "men": num_men,
            "boys": num_boys,
            "females": num_females,
            "women": num_women,
            "girls": num_girls,
            "children": num_boys + num_girls,
            "adults": num_men + num_women,
            "elders": num_elders,
            "crowd_size": num_females + num_males,
            "waiving": num_waving,
            "raising_left": num_larm,
            "raising_right": num_rarm,
            "pointing_left": num_lpointing,
            "pointing_right": num_rpointing,
            "laying": num_laying,
            "sitting": num_sitting
        }


# Standalone testing -----------------------------------------------------------------

class TestDetectCrowd(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'DETECT_CROWD',
                                                'abort': 'Aborted'})

            smach.StateMachine.add("DETECT_CROWD",
                                   DetectCrowd(robot),
                                   transitions={'succeeded': 'Done',
                                                'failed': 'Aborted'})


if __name__ == "__main__":
    rospy.init_node('speech_person_recognition_exec')

    startup(TestDetectCrowd, challenge_name="challenge_spr")
