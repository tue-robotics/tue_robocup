#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

import actionlib

# Perception
from perception_srvs.srv import StartPerception
from pein_msgs.msg import LearnAction, LearnGoal # for face learning
import pein_srvs.srv

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

        '''Laser service'''
        self.sv_laser_detector = rospy.ServiceProxy("/find_obj_in_roi", pein_srvs.srv.FindObjInRoi)

        '''Bin detector'''
        self.sv_bin_detector = rospy.ServiceProxy("/set_roi_bin_detection", pein_srvs.srv.FindObjInRoi)

        '''Table detector'''
        self.sv_table_detector = rospy.ServiceProxy("/table_detector_2d/set_region_of_interest_table", pein_srvs.srv.FindObjInRoi)

        ''' Service to load template matching configuration '''
        self.sv_template_config = rospy.ServiceProxy("/template_matching/srv_input", pein_srvs.srv.StartStop)

        ''' Temporarily included to test toggle_perception_2d '''
        import geometry_msgs
        self.testpoint = geometry_msgs.msg.PointStamped()
        self.testpoint.header.frame_id = '/map'
        self.testpoint.point.x = 0.66
        self.testpoint.point.y = 0.73
        self.testpoint.point.z = 0.83

    def close(self):
        pass

    def toggle(self, modules):
        # Possibilities (?):
        # - "template_matching"
        # - "object_detector_2d"
        # - "face_recognition"
        # - ("face_detection")
        # - "face_segmentation"
        # - "ppl_detection"
        # - ...
        rospy.loginfo("modules are {0}".format(modules))
        return self.sv_recognition(modules)

    def toggle_recognition(self, faces=False, objects=False, people=False):
        rospy.logwarn("This function is deprecated, please use toggle instead")
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
        #Get rid of stupid multiple if clauses later
        if people:
            result = self.sv_recognition(["ppl_detection"])
        return result

    def toggle_perception_2d(self, pointstamped, length_x=0.5, length_y=0.5, length_z=0.5):

        rospy.logwarn("This function is deprecated, toggle object_detector_2d separately and use set_perception_roi to set the roi")
        # ToDo: does this work as we hope?
        # ToDo: what is a suitable length/width?
        # ToDo: what is status?
        ''' Switch on object detector 2d '''
        result = self.sv_recognition(["object_detector_2d"])
        if not result:
            rospy.logerr("Service call object_detector_2d not succeeded")
            return False

        ''' Set region of interest '''
        #from pein_srvs.srv import FindObjInRoi
        request = pein_srvs.srv.FindObjInRoiRequest()
        request.x = pointstamped.point.x
        request.y = pointstamped.point.y
        request.z = pointstamped.point.z
        request.length_x = length_x
        request.length_y = length_y
        request.length_z = length_z
        request.frame = pointstamped.header.frame_id

        ''' Wait for service '''
        try:
            rospy.wait_for_service("/find_obj_in_roi", timeout=5.0)
            self.sv_laser_detector = rospy.ServiceProxy("/find_obj_in_roi", pein_srvs.srv.FindObjInRoi)
            response = self.sv_laser_detector(request)
        except rospy.ServiceException, e:
            rospy.logerr("Laser service not available: {0}".format(e))
            return False

        return response

    def set_perception_roi(self, pointstamped, length_x=0.5, length_y=0.5, length_z=0.5):
        ''' Sets region of interest for object_detector_2d and tabletop_segmentation'''
        #from pein_srvs.srv import FindObjInRoi
        request = pein_srvs.srv.FindObjInRoiRequest()
        request.x = pointstamped.point.x
        request.y = pointstamped.point.y
        request.z = pointstamped.point.z
        request.length_x = length_x
        request.length_y = length_y
        request.length_z = length_z
        request.frame = pointstamped.header.frame_id

        ''' Wait for service '''
        try:
            rospy.wait_for_service("/find_obj_in_roi", timeout=5.0)
            self.sv_laser_detector = rospy.ServiceProxy("/find_obj_in_roi", pein_srvs.srv.FindObjInRoi)
            response = self.sv_laser_detector(request)
        except rospy.ServiceException, e:
            rospy.logerr("Laser service not available: {0}".format(e))
            return False

        return response

    def set_table_roi(self, x, y, z, length_x=0.5, length_y=0.5, length_z=0.5, frame="/map"):
        request = pein_srvs.srv.FindObjInRoiRequest()
        request.x = x
        request.y = y
        request.z = z
        request.length_x = length_x
        request.length_y = length_y
        request.length_z = length_z
        request.frame = frame

        ''' Wait for service '''
        try:
            rospy.wait_for_service("/table_detector_2d/set_region_of_interest_table", timeout=5.0)
            response = self.sv_table_detector(request)
        except rospy.ServiceException, e:
            rospy.logerr("Laser service not available: {0}".format(e))
            return False

        return response

    def toggle_bin_detection(self, pointstamped, length_x=3.0, length_y=3.0, length_z=1.0):
        ''' Starts bin detection '''
        request = pein_srvs.srv.FindObjInRoiRequest()
        request.x = pointstamped.point.x
        request.y = pointstamped.point.y
        request.z = pointstamped.point.z
        request.length_x = length_x
        request.length_y = length_y
        request.length_z = length_z
        request.frame = pointstamped.header.frame_id

        # try:
        #     rospy.wait_for_service("/set_roi_bin_detection", timeout=5.0)
        #     self.sv_bin_detector = rospy.ServiceProxy("/set_roi_bin_detection", pein_srvs.srv.FindObjInRoi)
        #     response = self.sv_bin_detector(request)
        # except rospy.ServiceException, e:
        #     rospy.logerr("Laser service not available: {0}".format(e))
        #     return False

        sv_available = rospy.wait_for_service("/set_roi_bin_detection", timeout=5.0)
        if sv_available:
            self.sv_bin_detector = rospy.ServiceProxy("/set_roi_bin_detection", pein_srvs.srv.FindObjInRoi)
            response = self.sv_bin_detector(request)
        else:
            rospy.logerr("Laser service not available: {0}".format(e))
            return False

        return response

    def load_template_matching_config(self, config_name, status='inactive'):
        ''' Loads configuration for pein template_matching, does not start it by default '''
        request = pein_srvs.srv.StartStopRequest()
        request.key.append('status')
        request.key.append('configuration')
        request.value.append(status)
        request.value.append(config_name)
        try:
            response = self.sv_template_config(request)
            return response
        except rospy.ServiceException, e:
            rospy.logerr("Load template config not available: {0}".format(e))
            return False

    '''Face learning'''
    def learn_person(self, name, n_models = 10, view = 'front', publish_while_learning = False):
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
        return self.ac_face_learning.wait_for_result(rospy.Duration(20))

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
