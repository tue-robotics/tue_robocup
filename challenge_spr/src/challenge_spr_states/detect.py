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

timeout = 10


class DetectCrowd(smach.State):
    """
    Detect a crowd and describe it based on individual detections
    """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], output_keys=['crowd_data'])
        self.robot = robot

    def execute(self, userdata=None):
        tries = 3
        detections = self.recognize(tries)

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
            self.robot.head.look_at_point(VectorStamped(100, 0, 1.5, self.robot.robot_name + "/base_link"))
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
        num_males = 0
        num_females = 0
        num_women = 0
        num_girls = 0
        num_men = 0
        num_boys = 0
        num_elders = 0

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
                    elif d.age > 60:
                        num_elders += 1
                    else:
                        num_men += 1
                else:
                    if d.age < 18:
                        num_girls += 1
                    elif d.age > 60:
                        num_elders += 1
                    else:
                        num_women += 1

            num_males = num_boys + num_men
            num_females = num_girls + num_women

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
            "crowd_size": num_females + num_males + num_elders
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
