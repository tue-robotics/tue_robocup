#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

import actionlib

# Perception
from perception_srvs.srv import StartPerception
from pein_msgs.msg import LearnAction, LearnGoal # for face learning

import collections

class Perception(object):
    """Interface to amigo's perception services"""

    def __init__(self, wait_service=False):
        rospy.logwarn("Currently no face detection/recognition implemented")

        rospy.loginfo("Wait for service {0}".format(wait_service))
        if wait_service:
            rospy.loginfo("Waiting for Perception Service")
            rospy.wait_for_service("/start_perception", timeout=2.0)

        self.sv_recognition = rospy.ServiceProxy("/start_perception", StartPerception)

        '''Face stuff'''
        #self.learn_face_feedback = rospy.Subscriber("/face_detections/feedback", std_msgs.msg.Int32, self.__cb_learn_face)
        self.learn_face_counter = collections.deque()

        '''Face Learning Action Client'''
        self.ac_face_learning = actionlib.SimpleActionClient("/face_learning/action_server", LearnAction)
        # self.ac_face_learning = actionlib.SimpleActionClient("/face_learning/pein_action_server", LearnAction)

    def close(self):
        pass

    def toggle(self, modules):
        return self.sv_recognition(modules)

    def toggle_recognition(self, faces=False, objects=False):
        #TODO: Everything related to faces is not yet tested and is not yet supposed to actually work. These methods are for testing the execs only!
        if faces and objects:
            rospy.loginfo("Perception.start: Both categories (Faces and Objects) specified for recognition, CPU hogging initiated")
            rospy.logwarn("No face recognition")
            #result = self.sv_recognition(["blob_clustering", "face_recognition", "face_detection"])
            result = self.sv_recognition(["template_matching", "face_recognition", "face_detection"])
        if faces and not objects:
            rospy.logwarn("No face recognition")
            result = self.sv_recognition(["face_recognition", "face_detection"])
        if not faces and objects:
            #result = self.sv_recognition(["blob_clustering"])
            result = self.sv_recognition(["template_matching"])
        if not faces and not objects:
            rospy.loginfo("Perception.stop: Both faces and objects not recognized anymore")
            result = self.sv_recognition([])
        return result

    '''Face learning'''
    def learn_person(self, name, n_models = 20, view = 'front', publish_while_learning = False):
        '''
        Maintainer: ziyang,

        Depend on:
        roslaunch openni_launch openni.launch
        roslaunch pein_supervisor start.launch
        roslaunch pein_face_recognition start.launch
        '''
        goal = LearnGoal()
        goal.module = "face_learning"
        goal.n_models = n_models
        goal.model_name = name
        goal.view = view
        goal.publish_while_learning = publish_while_learning
        self.ac_face_learning.send_goal(goal)
        rospy.loginfo("Learning person '{0}'...".format(name))
        return self.ac_face_learning.wait_for_result()

    '''Cancel face learning'''
    def cancel_learn_persons(self):
        self.ac_face_learning.cancel_all_goals()

    '''Face stuff'''
    #def __cb_learn_face(self, result):
    #    self.learn_face_counter.append(result.data)

    '''Face stuff'''
    def get_learn_face_counter(self):
        rospy.logwarn("Learning persons currently not implemented, so cannot get counter")
        return self.learn_face_counter


if __name__ == "__main__":
    rospy.init_node("amigo_perception_executioner", anonymous=True)
    perception = Perception()
