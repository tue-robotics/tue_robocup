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

from cv_bridge import CvBridge, CvBridgeError
from openface_ros.face_recognizer import FaceRecognizer
# from skybiometry_ros import Skybiometry
from image_recognition_msgs.msg import FaceProperties
from image_recognition_msgs.srv import GetFaceProperties
from robot_smach_states.util.startup import startup
from robot_skills.util.kdl_conversions import VectorStamped
from robocup_knowledge import load_knowledge

timeout = 10


align_path = '~/openface/models/dlib/shape_predictor_68_face_landmarks.dat'
net_path = '~/openface/models/openface/nn4.small2.v1.t7'


# key = '69efefc20c7f42d8af1f2646ce6742ec'
# secret = '5fab420ca6cf4ff28e7780efcffadb6c'


class DetectCrowd(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], output_keys=['crowd_data'])
        self.robot = robot
        self._bridge = CvBridge()
        self._face_recognizer = FaceRecognizer(align_path, net_path)
        # self._skybiometry = Skybiometry(key, secret)

    def execute(self, userdata):
        tries = 3
        detections = self.recognize(tries)

        crowd_data = self.describe_crowd(detections)
        userdata.crowd_data = crowd_data

        return 'succeeded'

    def recognize(self, tries):
        number_of_people = 0
        best_detection = None

        sentences = ["You are all looking great today!            Keep looking in my camera!",
                     "I like it when everybody is staring at me; being in the center of attention!"]

        for i in range(0, tries):
            self.robot.speech.speak(sentences[i % (tries - 1)], block=False)
            image = self.get_shot()
            face_rois = self.get_faces(image)

            if len(face_rois) > number_of_people:
                number_of_people = len(face_rois)
                best_image = image
                best_detection = face_rois

        imgs = []
        if best_detection:
            for face_recognition in best_detection:
                cv2_img = best_image[face_recognition.roi.y_offset:face_recognition.roi.y_offset + face_recognition.roi.height,
                                 face_recognition.roi.x_offset:face_recognition.roi.x_offset + face_recognition.roi.width]
                imgmsg = self._bridge.cv2_to_imgmsg(cv2_img, 'bgr8')
                imgs.append(imgmsg)

        rospy.loginfo('Calling Skybiometry...')

        try:
            # face_properties = self._skybiometry.get_face_properties(imgs, timeout)
            get_face_properties = rospy.ServiceProxy('/get_face_properties', GetFaceProperties)
            face_properties_response = get_face_properties(imgs)
            face_properties = face_properties_response.properties_array
        except Exception as e:
            rospy.logerr(str(e))
            self.robot.speech.speak('API call failed, is there internet?')
            return [None]*number_of_people

        face_log = '\n - '.join([''] + [repr(s) for s in face_properties])
        rospy.loginfo('face_properties:%s', face_log)
        return face_properties


    def get_shot(self):
        z = 1.5
        self.robot.head.look_at_point(VectorStamped(100, 0, z, self.robot.robot_name + "/base_link"))
        self.robot.head.wait_for_motion_done()
        time.sleep(1)

        image =self.robot.head.get_image()
        return self._bridge.imgmsg_to_cv2(image, 'bgr8')

    def get_faces(self, image):

        faces = self._face_recognizer.recognize(image)

        rospy.loginfo("Faces: %s", faces)

        return faces


    def describe_crowd(self, detections):
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
                        num_boys +=1
                    elif d.age > 60:
                        num_elders +=1
                    else:
                        num_men += 1
                else:
                    if d.age < 18:
                        num_girls +=1
                    elif d.age > 60:
                        num_elders +=1
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
