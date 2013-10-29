#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach
#import object_msgs.msg
import math

from sensor_msgs.msg import LaserScan
import geometry_msgs

from psi import Compound

import util.reasoning_helpers as urh
import robot_skills.util.msg_constructors as msgs

class Learn_Person(smach.State):
    '''
    Maintainer: ziyang, Loy

    Face learning state, learn face from left, right, and front view.
    '''

    def __init__(self, robot, name=None):
        smach.State.__init__(self,
                             outcomes = ['face_learned', 'learn_failed'])
        self.robot = robot
        self.name_query = Compound("name_to_learn", "Name")
        self.name = name

    def execute(self, userdate=None):
        if self.name:
            name_to_learn = self.name
        else:
            answers = self.robot.reasoner.query(self.name_query)
            if answers:
                name_to_learn = str(answers[0]["Name"])
            else:
                rospy.logerr("Name to learn is not known.")
                name_to_learn = "unknown"

        rospy.loginfo('Get user name {0}'.format(name_to_learn))
        self.robot.speech.speak("This may take a while, please be patient while I try to learn your face.")
        speech_sentence = [ 'Please look at my left arm, until I am finished learning.',
                            'Now look at my right arm, please wait until I am finished learning',
                            'Please look at my face, till I am finished.']

        # learn left face
        self.robot.leftArm.send_joint_goal(-1.159, 0.511, -1.021, 1.669, -0.603, 0.255, 0.0206,timeout=2)

        self.robot.speech.speak(speech_sentence[0])
        result = self.robot.perception.learn_person(name_to_learn, view = 'left', publish_while_learning = False)
        if result == True:
            self.robot.reasoner.assertz(Compound("learned_person", name_to_learn, Compound("view", "left")))
        self.robot.speech.speak("Finished learning your left side")

        # learn right face
        self.robot.leftArm.send_joint_goal(-1.39, 1.096, -0.967, 1.352, -0.9489, 0.5272, 0.0367,timeout=2)
        rospy.sleep(1)
        self.robot.leftArm.reset_arm()
        self.robot.rightArm.send_joint_goal(-1.159, 0.511, -1.021, 1.669, -0.603, 0.255, 0.0206,timeout=2)

        self.robot.speech.speak(speech_sentence[1])
        result = self.robot.perception.learn_person(name_to_learn, view = 'right')
        if result == True:
            self.robot.reasoner.assertz(Compound("learned_person", name_to_learn, Compound("view", "right")))
        self.robot.speech.speak("Finished learning your right side")

        # learn front face
        self.robot.rightArm.send_joint_goal(-1.159, 1.096, -1.021, 1.669, -0.603, 0.255, 0.0206,timeout=2)
        rospy.sleep(1)
        self.robot.rightArm.reset_arm()


        self.robot.speech.speak(speech_sentence[2])
        result = self.robot.perception.learn_person(name_to_learn, view = 'front')
        if result == True:
            self.robot.reasoner.assertz(Compound("learned_person", name_to_learn, Compound("view", "front")))
        self.robot.speech.speak("Learning succeeded. Now I should recognize you, next time!")
        
        return 'face_learned'

class LookForObjectsAtROI(smach.State):
    def __init__(self, robot, lookat_query, object_query, maxdist=0.8, modules=["template_matching"], waittime=2.5):
        #TODO: Remove 'looking' outcome, it is never given.
            smach.State.__init__(self, outcomes=['looking','object_found','no_object_found','abort'],
                                    input_keys=[],
                                    output_keys=[])
            self.lookat_query = lookat_query
            self.object_query = object_query
            self.robot = robot
            self.maxdist = maxdist
            self.modules = modules
            self.waittime= waittime
            assert hasattr(self.robot, "reasoner")
            try:
                assert hasattr(self.robot, "perception")
            except AssertionError:
                rospy.logerr("perception not available, but still trying without")
                self.robot.speech.speak("I can't see a thing, but I'll try to be of service anyway. Wish me luck, or stop me before I do something silly.")
            assert hasattr(self.robot, "head")

    def calc_dist(self, (xA,yA,zA), (xB,yB,zB)):
            dist = math.sqrt(abs(xA-xB)**2 + abs(yA-yB)**2 + abs(zA-zB)**2)
            return dist

    def execute(self, userdata):

        # Query reasoner for position to look at

        try:
            lookat_answers = self.robot.reasoner.query(self.lookat_query)
            basepos = self.robot.base.location.pose.position
            basepos = (basepos.x, basepos.y, basepos.z)
            selected_roi_answer = urh.select_answer(lookat_answers, 
                                                lambda answer: urh.xyz_dist(answer, basepos), 
                                                minmax=min)
            rx,ry,rz = urh.answer_to_tuple(selected_roi_answer)
            rospy.loginfo("[TEST ERIK] LOOKING AT (X = {0}, Y = {1}, Z = {2})".format(rx,ry,rz))
            lookat_point = msgs.PointStamped(rx,ry,rz)
            print lookat_point

            # Send spindle goal to bring head to a suitable location
            # Correction for standard height: with a table heigt of 0.76 a spindle position
            # of 0.35 is desired, hence offset = 0.76-0.35 = 0.41
            # Minimum: 0.15 (to avoid crushing the arms), maximum 0.4
            # ToDo: do we need to incorporate wait functions?
            spindle_target = max(0.15, min(lookat_point.point.z - 0.41, self.robot.spindle.upper_limit))
            rospy.loginfo("Target height: {0}, spindle_target: {1}".format(lookat_point.point.z, spindle_target))

            self.robot.spindle.send_goal(spindle_target,timeout=5.0)
            self.robot.head.send_goal(lookat_point, keep_tracking=False)
        except ValueError, ve:
            rospy.loginfo("lookat_answers = {0}".format(lookat_answers))
            rospy.loginfo("Further processing yielded {0}".format(ve))
            self.robot.speech.speak("I did not find an object.")
            return 'no_object_found'
            
        # Toggle perception on

        #rospy.sleep(3.0)

        rospy.loginfo("Start object recognition")
        #result = self.robot.perception.toggle_recognition(objects=True)
        result = self.robot.perception.toggle(self.modules)

        '''Try to set the region of interest. This is not implemented for every module '''
        target_point = geometry_msgs.msg.PointStamped()
        target_point.header.frame_id = "/map"
        target_point.header.stamp = rospy.Time()
        target_point.point.x = lookat_point.point.x
        target_point.point.y = lookat_point.point.y
        target_point.point.z = lookat_point.point.z
        try:
            self.robot.perception.set_perception_roi(target_point, length_x=0.6, length_y=0.6, length_z=0.4)
        except rospy.exceptions.ROSException as e:
            rospy.logwarn("Cannot set perception roi for modules {0}: {1}. Module may not support ROIs".format(self.modules, e))

        # Let the object recognition run for a certain period
        # ToDo: replace until new objects have appeared
        #starttime = rospy.Time.now()
        #object_answers = []
        #while ( (not object_answers) and ( (rospy.Time.now() - starttime) < rospy.Duration(self.waittime)) ):
        #    object_answers = self.robot.reasoner.query(self.object_query)
        #    rospy.sleep(0.1)
        rospy.sleep(rospy.Duration(self.waittime))

        rospy.loginfo("Stop object recognition")

        #result = self.robot.perception.toggle_recognition(objects=False)
        result = self.robot.perception.toggle([])

        # Query reasoner for objects
        try:
            rospy.loginfo("Querying reasoner")
            object_answers = self.robot.reasoner.query(self.object_query)
            #Sort by distance to lookat_point
            #import ipdb; ipdb.set_trace()
            # closest_QA = min(object_answers, key=lambda ans: self.calc_dist((lookat_point.x, lookat_point.y,lookat_point.z), (     float(ans["X"]),
            #                                                                                     float(ans["Y"]),
            #                                                                                     float(ans["Z"]))))

            # object_id = closest_QA["ObjectID"]
            rospy.loginfo("Selecting closest answer")
            closest_QA = urh.select_answer(object_answers, 
                                                lambda answer: urh.xyz_dist(answer, (rx,ry,rz)), 
                                                minmax=min,
                                                criteria=[  lambda answer: urh.xyz_dist(answer, (rx,ry,rz)) < self.maxdist,
                                                            lambda answer: answer["Z"] > 0.4]) #The object is above 0.4m
            ox,oy,oz = urh.answer_to_tuple(closest_QA)
            lookat_point = msgs.PointStamped(ox,oy,oz)
            # retract current object id
            r = self.robot.reasoner

            r.query(Compound("retractall", Compound("current_object", "X")))
            rospy.loginfo("Asserting new ID")
            # assert new object id
            object_id = closest_QA["ObjectID"]
            r.assertz(Compound("current_object", object_id))
            return 'object_found'
        except ValueError, ve2:
            return 'no_object_found'
            

class LookForObjectsAtPoint(smach.State):
    def __init__(self, robot, object_query, point_stamped, modules=["template_matching"], waittime=2.5):
        smach.State.__init__(self, outcomes=['looking','object_found','no_object_found','abort'],
                                input_keys=[],
                                output_keys=[])
        self.object_query = object_query
        self.robot = robot
        self.point_stamped = point_stamped
        self.modules = modules
        self.waittime= waittime
        assert hasattr(self.robot, "head")
        try:
            assert hasattr(self.robot, "perception")
        except AssertionError:
            rospy.logerr("perception not available, but still trying without")
            self.robot.speech.speak("I can't see a thing, but I'll try to be of service anyway. Wish me luck, or stop me before I do something silly.")
        assert hasattr(self.robot, "reasoner")

    def execute(self, userdata):
        # Send spindle goal to bring head to a suitable location
        # Correction for standard height: with a table heigt of 0.76 a spindle position
        # of 0.35 is desired, hence offset = 0.76-0.35 = 0.41
        # Minimum: 0.15 (to avoid crushing the arms), maximum 0.4
        # ToDo: do we need to incorporate wait functions?
        spindle_target = max(0.15, min(self.point_stamped.point.z - 0.41, self.robot.spindle.upper_limit))

        rospy.loginfo("Target height: {0}, spindle_target: {1}".format(self.point_stamped.point.z, spindle_target))
        self.robot.spindle.send_goal(spindle_target)
        self.robot.head.send_goal(self.point_stamped)

        # Toggle perception on

        rospy.loginfo("Start object recognition")
        #result = self.robot.perception.toggle_recognition(objects=True)
        result = self.robot.perception.toggle(self.modules)

        # Let the object recognition run for a certain period
        '''Try to set the region of interest. This is not implemented for every module '''
        target_point = geometry_msgs.msg.PointStamped()
        target_point.header.frame_id = "/map"
        target_point.header.stamp = rospy.Time()
        target_point.point.x = self.point_stamped.point.x
        target_point.point.y = self.point_stamped.point.y
        target_point.point.z = self.point_stamped.point.z
        try:
            self.robot.perception.set_perception_roi(target_point, length_x=0.6, length_y=0.6, length_z=0.4)
        except Exception as e:
            rospy.loginfo("Cannot set perception roi for modules {0}".format(self.modules))
            rospy.loginfo("Error: {0}".format(e))
        
        #starttime = rospy.Time.now()
        #object_answers = []
        #while ( (not object_answers) and ( (rospy.Time.now() - starttime) < rospy.Duration(self.waittime)) ):
        #    object_answers = self.robot.reasoner.query(self.object_query)
        #    rospy.sleep(0.1)
        rospy.sleep(rospy.Duration(self.waittime))

        rospy.loginfo("Stop object recognition")

        #result = self.robot.perception.toggle_recognition(objects=False)
        result = self.robot.perception.toggle([])

        # Query reasoner for objects
        answers = self.robot.reasoner.query(self.object_query)

        if not answers:
            return 'no_object_found'
        else:
            def calc_dist((xA,yA,zA), (xB,yB,zB)):
                dist = math.sqrt(abs(xA-xB)**2 + abs(yA-yB)**2 + abs(zA-zB)**2)
                return dist
            p = (self.point_stamped.point.x,  self.point_stamped.point.y, self.point_stamped.point.z)
            closest_QA = min(answers, key=lambda ans: calc_dist(p, (float(ans["X"]), float(ans["Y"]), float(ans["Z"]))))

            object_id = closest_QA["ObjectID"]
            #object_id = answers[0]["ObjectID"]

            # retract current object id
            r = self.robot.reasoner

            r.query(Compound("retractall", Compound("current_object", "X")))

            # assert new object id
            r.assertz(Compound("current_object", object_id))

            return 'object_found'

class Read_laser(smach.State):
    def __init__(self, robot, door):
        smach.State.__init__(self, outcomes=['laser_read'])
                                   #input_keys=['rate','laser_value','distance','locations','target','challenge_duration'],
                                   #output_keys=['start_time','global_end_time'])
        self.rate = 10
        self.min_threshold = 0.02   # Minimum distance to exclude dirt on the laser
        self.door_threshold = 1.2   # Maximum allowed distance to door: anything bigger than this means that the door is open
        self.door_open = 0
        self.robot = robot
        self.laser_value = 0.0

        self.door = door

        self.laser_listener = rospy.Subscriber("/amigo/base_front_laser",LaserScan,self.callback_laser)
        try:
            assert hasattr(self.robot, "perception")
        except AssertionError:
            rospy.logerr("perception not available, but still trying without")
            self.robot.speech.speak("I can't see a thing, but I'll try to be of service anyway. Wish me luck, or stop me before I do something silly.")
        assert hasattr(self.robot, "reasoner")

    def callback_laser(self, data):
        laser_range = data.ranges
        index = len(laser_range)/2
        
        if data.header.frame_id == "/amigo/base_laser":
            self.laser_value = laser_range[index]
        else:
            self.laser_value = 0.0

    def execute(self, userdata):
        self.robot.lights.set_color(0,1,1)
        r = self.robot.reasoner
        rospy.Rate(self.rate).sleep()

        ## Publish middle laser value
        # rospy.loginfo(Middle laser value "{0} ".format(self.laser_value))

        if self.laser_value > self.door_threshold:
            self.door_open = 1
            if not r.query(Compound("state", self.door,"open")):
                if r.query(Compound("state", self.door,"closed")):
                    r.query(Compound("retract", Compound("state", self.door,"closed")))
                r.assertz(Compound("state", self.door,"open"))
        else:
            self.door_open = 0
            if not r.query(Compound("state", self.door,"closed")):
                if r.query(Compound("state", self.door,"open")):
                    r.query(Compound("retract", Compound("state", self.door,"open")))
                r.assertz(Compound("state", self.door,"closed"))

        self.robot.lights.set_color(0,0,1)
        return 'laser_read'

class TogglePeopleDetector(smach.State):
    """Enables or disables PeopleDetector"""
    def __init__(self, robot, roi_query=None, on=True):
        smach.State.__init__(self, outcomes=["toggled"])
        self.robot = robot
        self.roi_query = roi_query
        self.on = on

    def execute(self, userdata=None):
        if self.on:
            result = self.robot.perception.toggle(["ppl_detection"])
        else:
            result = self.robot.perception.toggle(["ppl_detection_off"])
        return "toggled"
