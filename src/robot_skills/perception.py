#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

import actionlib

# Perception
from perception_srvs.srv import StartPerception
from pein_msgs.msg import LearnAction, LearnGoal, ROI
import pein_srvs.srv

import collections

import tue_recorder.msg

class Perception(object):
    """Interface to amigo's perception services"""

    def __init__(self, wait_service=False):
        rospy.logwarn("Currently no face detection/recognition implemented")

        rospy.loginfo("Wait for service {0}".format(wait_service))
        if wait_service:
            rospy.loginfo("Waiting for Perception Service")
            rospy.wait_for_service("/start_perception", timeout=2.0)

        self.sv_recognition = rospy.ServiceProxy("/start_perception", StartPerception)

        '''Default object recognition method'''
        self.default_object_recognition_method = "object_segmentation"

        '''Face stuff'''
        #self.learn_face_feedback = rospy.Subscriber("/face_detections/feedback", std_msgs.msg.Int32, self.__cb_learn_face)
        self.learn_face_counter = collections.deque()

        '''Face Learning Action Client'''
        self.ac_face_learning = actionlib.SimpleActionClient("/face_learning/action_server", LearnAction)
        # self.ac_face_learning = actionlib.SimpleActionClient("/face_learning/pein_action_server", LearnAction)

        '''Set ROI service'''
        self.sv_set_roi = rospy.ServiceProxy("/find_obj_in_roi", pein_srvs.srv.FindObjInRoi)

        '''Bin detector'''
        self.sv_bin_detector = rospy.ServiceProxy("/set_roi_bin_detection", pein_srvs.srv.FindObjInRoi)

        '''Table detector'''
        self.sv_table_detector = rospy.ServiceProxy("/table_detector_2d/set_region_of_interest_table", pein_srvs.srv.FindObjInRoi)

        ''' Service to load template matching configuration '''
        self.sv_template_config = rospy.ServiceProxy("/template_matching/srv_input", pein_srvs.srv.StartStop)

        ''' Publisher for signaling image recording '''
        self.pub_rec = rospy.Publisher('/recorder/start', tue_recorder.msg.Start, queue_size=10)

        '''People detection ROI'''
        self.ppl_detection_laser = rospy.ServiceProxy('ppl_detection_generic/start_with_roi', pein_srvs.srv.StartStopWithROIArray)

        ''' List with modules that should be always on '''
        self.always_on_modules = []

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

        # start recording
        if modules:
            self.rec_start("amigo_top_kinect", "perception-toggle", 5, 2.0)
        
        # If 'object_recognition' is called, this is replaced by the current default module
        modules = [module.replace("object_recognition", self.default_object_recognition_method) for module in modules]
        
        if (len(modules) == 0):
            modules = self.always_on_modules

        #''' Add always_on_modules ''' --> WILL BE REINSTATED WHEN TOGGLED OFF
        #for module in self.always_on_modules:
        #    if not module in modules:
        #        modules.append(module)

        rospy.loginfo("modules are {0}".format(modules))
        return self.sv_recognition(modules)

    def toggle_always_on(self, modules):
        ''' Switches on modules that remain switched on when switching other modules on and off '''
        for module in modules:
            if not module in self.always_on_modules:
                self.always_on_modules.append(module)
        rospy.loginfo("Always on modules are {0}".format(self.always_on_modules))
        self.toggle(self.always_on_modules)

    def toggle_always_off(self, modules):
        ''' Switches off modules that remain switched on when switching other modules on and off '''
        for module in modules:
            self.always_on_modules = [x for x in self.always_on_modules if x != module]
        rospy.loginfo("Always on modules are {0}".format(self.always_on_modules))
        self.toggle(self.always_on_modules)

    def toggle_everything_off(self):
        ''' Switches off everything, i.e., normal and always_on modules '''
        self.always_on_modules = []
        self.toggle([])

    def toggle_recognition(self, faces=False, objects=False, people=False):
        rospy.logwarn("This function is deprecated, please use toggle instead")

        # start recording
        self.rec_start("amigo_top_kinect", "toggle_recognition", 5, 2.0)

        #TODO: Everything related to faces is not yet tested and is not yet supposed to actually work. These methods are for testing the execs only!
        if faces and objects:
            rospy.loginfo("Perception.start: Both categories (Faces and Objects) specified for recognition, CPU hogging initiated")
            rospy.logwarn("No face recognition")
            #result = self.sv_recognition(["blob_clustering", "face_recognition", "face_detection"])
            result = self.sv_recognition([self.default_object_recognition_method, "face_recognition", "face_detection"])
        if faces and not objects:
            rospy.logwarn("No face recognition")
            result = self.sv_recognition(["face_recognition", "face_detection"])
        if not faces and objects:
            #result = self.sv_recognition(["blob_clustering"])
            result = self.sv_recognition([self.default_object_recognition_method])
        if not faces and not objects:
            rospy.loginfo("Perception.stop: Both faces and objects not recognized anymore")
            result = self.sv_recognition([])
        #Get rid of stupid multiple if clauses later
        if people:
            result = self.sv_recognition(["ppl_detection"])
        return result

    def toggle_perception_2d(self, pointstamped, length_x=0.5, length_y=0.5, length_z=0.5):

        rospy.logwarn("This function is deprecated, toggle object_detector_2d separately and use set_perception_roi to set the roi")

        # start recording
        self.rec_start("amigo_top_kinect", "toggle_perception_2d", 5, 2.0)

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
            self.sv_set_roi = rospy.ServiceProxy("/find_obj_in_roi", pein_srvs.srv.FindObjInRoi)
            response = self.sv_set_roi(request)
        except (rospy.ServiceException, rospy.ROSException), e:
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
            self.sv_set_roi = rospy.ServiceProxy("/find_obj_in_roi", pein_srvs.srv.FindObjInRoi)
            response = self.sv_set_roi(request)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Set ROI service not available: {0}".format(e))
            return False

        return response

    def set_table_roi(self, pointstamped, length_x=0.5, length_y=0.5, length_z=0.5):
        request = pein_srvs.srv.FindObjInRoiRequest()
        request.x = pointstamped.point.x
        request.y = pointstamped.point.y
        request.z = pointstamped.point.z
        request.length_x = length_x
        request.length_y = length_y
        request.length_z = length_z
        request.frame = pointstamped.header.frame

        ''' Wait for service '''
        try:
            rospy.wait_for_service("/table_detector_2d/set_region_of_interest_table", timeout=5.0)
            response = self.sv_table_detector(request)
        except rospy.ServiceException, e:
            rospy.logerr("Laser service not available: {0}".format(e))
            return False

        return response

    def toggle_bin_detection(self, pointstamped, length_x=3.0, length_y=3.0, length_z=1.0):

        # start recording
        self.rec_start("amigo_top_kinect", "toggle_bin_detection", 5, 2.0)

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
            rospy.logerr("Laser service not available")
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

        # start recording
        self.rec_start("amigo_top_kinect", "learn_person", 5, 2.0)

        goal = LearnGoal()
        goal.module = "face_learning"
        goal.n_models = n_models
        goal.model_name = name
        goal.view = view
        goal.publish_while_learning = publish_while_learning
        self.ac_face_learning.send_goal(goal)
        rospy.loginfo("Learning person '{0}'...".format(name))
        return self.ac_face_learning.wait_for_result(rospy.Duration(20)) #TODO: This may be too long

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

    def rec_start(self, source, context, duration, freq):
        msg = tue_recorder.msg.Start()
        msg.source = source
        msg.context = context
        msg.max_duration = rospy.Duration(duration)
        msg.frequency = freq
        
        self.pub_rec.publish(msg)        


    def people_detection_torso_laser(self, pointstamped, time=4.0, length_x=3.0, length_y=3.0, length_z=1.0):

        ''' Starts people detection '''
        request = pein_srvs.srv.StartStopWithROIArrayRequest()
        request.status = True
        
        roi_ppl = ROI()
        roi_ppl.x = pointstamped.point.x
        roi_ppl.y = pointstamped.point.y
        roi_ppl.z = pointstamped.point.z
        roi_ppl.length_x = length_x
        roi_ppl.length_y = length_y
        roi_ppl.length_z = length_z
        roi_ppl.frame = pointstamped.header.frame_id

        request.rois.append(roi_ppl)

        print "\n request ppl_detection = ", request, "\n"

        ''' Wait for service '''
        try:
            rospy.wait_for_service("/ppl_detection_generic/start_with_roi", timeout=5.0)
            response = self.ppl_detection_laser(request)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("People Detection Laser service not available: {0}".format(e))
            return False

        ''' Wait time before stopping people detection '''
        rospy.sleep(time)
        request.status = False
        self.ppl_detection_laser(request)

        return response
        

if __name__ == "__main__":
    rospy.init_node("amigo_perception_executioner", anonymous=True)
    perception = Perception()
