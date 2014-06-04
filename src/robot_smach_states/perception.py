#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach
import navigation
import reasoning
import human_interaction
#import object_msgs.msg
import math

from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
import geometry_msgs

from psi import Compound, Conjunction, Sequence

import util.reasoning_helpers as urh
import robot_skills.util.msg_constructors as msgs
from robot_skills.util.transformations import tf_transform

class Learn_Person(smach.State):
    '''
    Maintainer: ziyang, Loy

    Face learning state, learn face from left, right, and front view.
    '''

    #def __init__(self, robot, name=None, models_per_view={'front':10, 'left':10, 'right':10}):
    def __init__(self, robot, name=None, models_per_view={'front':10}):
        smach.State.__init__(self,
                             outcomes = ['face_learned', 'learn_failed'])
        self.robot = robot
        self.name_query = Compound("name_to_learn", "Name")
        self.name = name
        self.models_per_view = models_per_view

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
        self.robot.speech.speak("I will now learn your face.", block=False)
        speech_sentence = [ 'Please look at my left arm',
                            'Now look at my right arm',
                            'Please look at my face']

        if 'left' in self.models_per_view:
            # learn left face
            self.robot.leftArm.send_joint_goal(-1.159, 0.511, -1.021, 1.669, -0.603, 0.255, 0.0206,timeout=2)

            self.robot.speech.speak(speech_sentence[0], block=False)
            result = self.robot.perception.learn_person(
                name_to_learn, 
                view='left', 
                publish_while_learning=False, 
                n_models=self.models_per_view['left'])
            
            if result == True:
                self.robot.reasoner.assertz(Compound("learned_person", name_to_learn, Compound("view", "left")))
            self.robot.speech.speak("Finished learning your left side", block=False)

        if 'right' in self.models_per_view:
            # learn right face
            self.robot.leftArm.send_joint_goal(-1.39, 1.096, -0.967, 1.352, -0.9489, 0.5272, 0.0367,timeout=2)
            self.robot.leftArm.reset_arm()
            self.robot.rightArm.send_joint_goal(-1.159, 0.511, -1.021, 1.669, -0.603, 0.255, 0.0206,timeout=2)

            self.robot.speech.speak(speech_sentence[1], block=False)
            result = self.robot.perception.learn_person(name_to_learn, 
                view = 'right', 
                n_models=self.models_per_view['right'])

            if result == True:
                self.robot.reasoner.assertz(Compound("learned_person", name_to_learn, Compound("view", "right")))
            self.robot.speech.speak("Finished learning your right side", block=False)

        if 'front' in self.models_per_view:
            # learn front face
            #self.robot.rightArm.reset_arm()

            self.robot.speech.speak(speech_sentence[2], block=False)
            result = self.robot.perception.learn_person(name_to_learn, 
                                                        view = 'front', 
                                                        n_models=self.models_per_view['front'])

            if result == True:
                self.robot.reasoner.assertz(Compound("learned_person", name_to_learn, Compound("view", "front")))
            self.robot.speech.speak("Learning succeeded", block=False)
        
        return 'face_learned'

class RecognizePerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['no_people_found', 'no_people_recognized', 'person_recognized'],
                                   input_keys=[],
                                   output_keys=[])
        self.robot = robot
        
        self.detect_query = Conjunction(Compound( "property_expected", "ObjectID", "class_label", "face"),
                                        Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")))
        
        self.recognize_query = Conjunction(Compound( "property_expected", "ObjectID", "class_label", "face"),
                                           Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                           Compound( "property", "ObjectID", "name", Compound("discrete", "DomainSize", "NamePMF")))

    def execute(self, userdata=None):
        ''' Reset robot '''
        self.robot.speech.speak("Let me see who's here", block=False)
        self.robot.head.look_up()
        self.robot.spindle.reset()

        # Do we need this?
        #''' Toggle perception and wait for someone to appear'''
        #cntr = 0;
        #self.robot.perception.toggle(["face_segmentation"])
        #while (not person_result and cntr < 10):
        #    rospy.sleep(0.5)
        #    cntr += 1
        #    rospy.loginfo("Checking for a person for the {0} time".format(cntr))
        #    person_result = self.robot.reasoner.query(self.detect_query)
        #    self.robot.perception.toggle([])

        #if not person_result:
        #    self.robot.speech.speak("No one here. Face segmentation did not find a person here", block=False)
        #    return 'no_people_found'

        #self.robot.speech.speak("Hi there, human. Please look into my eyes, so I can recognize you.", block=False)  
    
        cntr = 0
        name = None
        self.robot.perception.toggle(["face_recognition", "face_segmentation"])

        while (cntr < 10 and not name):
            rospy.sleep(0.5)
            cntr += 1
            rospy.loginfo("Checking for the {0} time".format(cntr))

            person_result = self.robot.reasoner.query(self.recognize_query)

            # get the name PMF, which has the following structure: [p(0.4, exact(will)), p(0.3, exact(john)), ...]
            if len(person_result) > 0:
                name_pmf = person_result[0]["NamePMF"]

                if len(person_result) > 1:
                    rospy.logwarn("Multiple faces detected, only checking the first one!")

                name=None
                name_prob=0
                for name_possibility in name_pmf:
                    print name_possibility
                    prob = float(name_possibility[0])
                    if prob > 0.175 and prob > name_prob:
                        name = str(name_possibility[1][0])
                        #print "Updated most probable name to " + str(name)
                        name_prob = prob

        self.robot.perception.toggle([])
        
        if not person_result:
            rospy.logwarn("No person names received from world model")
            return 'no_people_found'
        elif not name:
            self.robot.speech.speak("I don't know who you are.", block=False)
            return 'no_people_recognized'
        elif name:
            self.robot.speech.speak("Hello " + str(name), block=False)
            #userdata.name = str(name)
            return 'person_recognized'        
            

class LookForObjectsAtROI(smach.State):
    def __init__(self, robot, lookat_query, object_query, maxdist=0.8, modules=["object_recognition"], waittime=2.5):
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
                self.robot.speech.speak("I can't see a thing, but I'll try to be of service anyway. Wish me luck, or stop me before I do something silly.", block=False)
            assert hasattr(self.robot, "head")

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
            self.robot.head.send_goal(lookat_point, keep_tracking=False, pan_vel=0.75, tilt_vel=0.75)
        except ValueError, ve:
            rospy.loginfo("lookat_answers = {0}".format(lookat_answers))
            rospy.loginfo("Further processing yielded {0}".format(ve))
            self.robot.speech.speak("I did not find an object.", block=False)
            return 'no_object_found'
            
        # Toggle perception on

        #rospy.sleep(3.0)

        rospy.loginfo("Start object recognition")
        #result = self.robot.perception.toggle_recognition(objects=True)
        result = self.robot.perception.toggle(self.modules)

        '''Try to set the region of interest. This is not implemented for every module '''
        #target_point = msgs.PointStamped(lookat_point.point.x, lookat_point.point.y, lookat_point.point.z, "/map", rospy.Time())
        #try:
        #    self.robot.perception.set_perception_roi(target_point, length_x=0.6, length_y=0.6, length_z=0.4)
        #except rospy.exceptions.ROSException as e:
        #    rospy.logwarn("Cannot set perception roi for modules {0}: {1}. Module may not support ROIs".format(self.modules, e))

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
        self.robot.perception.toggle([])

        # Query reasoner for objects
        try:
            rospy.loginfo("Querying reasoner")
            object_answers = self.robot.reasoner.query(self.object_query)
            #Sort by distance to lookat_point
            #import ipdb; ipdb.set_trace()
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
        except ValueError:
            return 'no_object_found'
            
class LookForObjectsAtPoint(smach.State):
    def __init__(self, robot, object_query, point_stamped, modules=["object_recognition"], waittime=2.5, maxdist=0.8):
        smach.State.__init__(self, outcomes=['looking','object_found','no_object_found','abort'],
                                input_keys=[],
                                output_keys=[])
        self.object_query = object_query
        self.robot = robot
        self.point_stamped = point_stamped
        self.modules = modules
        self.waittime= waittime
        self.maxdist = maxdist

        assert hasattr(self.robot, "head")
        try:
            assert hasattr(self.robot, "perception")
        except AssertionError:
            rospy.logerr("perception not available, but still trying without")
            self.robot.speech.speak("I can't see a thing, but I'll try to be of service anyway. Wish me luck, or stop me before I do something silly.", block=False)
        assert hasattr(self.robot, "reasoner")

    def execute(self, userdata=[]):
        # Send spindle goal to bring head to a suitable location
        # Correction for standard height: with a table heigt of 0.76 a spindle position
        # of 0.35 is desired, hence offset = 0.76-0.35 = 0.41
        # Minimum: 0.15 (to avoid crushing the arms), maximum 0.4
        # ToDo: do we need to incorporate wait functions?
        spindle_target = max(0.15, min(self.point_stamped.point.z - 0.41, self.robot.spindle.upper_limit))

        rospy.loginfo("Target height: {0}, spindle_target: {1}".format(self.point_stamped.point.z, spindle_target))
        self.robot.spindle.send_goal(spindle_target)
        self.robot.head.send_goal(self.point_stamped, pan_vel=0.75, tilt_vel=0.75)

        # Toggle perception on
        rospy.loginfo("Start object recognition")
        self.robot.perception.toggle(self.modules)

        # Let the object recognition run for a certain period
        '''Try to set the region of interest. This is not implemented for every module '''
        #target_point = msgs.PointStamped(self.point_stamped.point.x, self.point_stamped.point.y, self.point_stamped.point.z, "/map", rospy.Time())
        #try:
        #    self.robot.perception.set_perception_roi(target_point, length_x=0.6, length_y=0.6, length_z=0.4)
        #except Exception as e:
        #    rospy.loginfo("Cannot set perception roi for modules {0}".format(self.modules))
        #    rospy.loginfo("Error: {0}".format(e))
        
        rospy.sleep(rospy.Duration(self.waittime))

        rospy.loginfo("Stop object recognition")

        self.robot.perception.toggle([])

        # Query reasoner for objects
        try:
            rospy.loginfo("Querying reasoner")
            object_answers = self.robot.reasoner.query(self.object_query)
            #Sort by distance to lookat_point
            #import ipdb; ipdb.set_trace()

            #Transform from base link to map.
            transformedpoint = tf_transform(self.point_stamped, self.point_stamped.header.frame_id, "/map")
            rx,ry,rz = transformedpoint.x, transformedpoint.y, transformedpoint.z

            rospy.loginfo("Selecting closest answer")
            closest_QA = urh.select_answer(object_answers, 
                                                lambda answer: urh.xyz_dist(answer, (rx,ry,rz)), 
                                                minmax=min,
                                                criteria=[  lambda answer: urh.xyz_dist(answer, (rx,ry,rz)) < self.maxdist,
                                                            lambda answer: answer["Z"] > 0.4]) #The object is above 0.4m
            # retract current object id
            r = self.robot.reasoner

            r.query(Compound("retractall", Compound("current_object", "X")))
            rospy.loginfo("Asserting new ID")
            # assert new object id
            object_id = closest_QA["ObjectID"]
            r.assertz(Compound("current_object", object_id))
            return 'object_found'
        except ValueError:
            return 'no_object_found'

class LookAtPoint(smach.State):
    def __init__(self, robot, lookat_query, maxdist=0.8, modules=["template_matching"], waittime=2.5):
            smach.State.__init__(self, outcomes=['looking','no_point_found','abort'],
                                    input_keys=[],
                                    output_keys=[])
            self.lookat_query = lookat_query
            self.robot = robot
            self.maxdist = maxdist
            self.modules = modules
            self.waittime= waittime
            assert hasattr(self.robot, "reasoner")
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
            rospy.loginfo("Looking at (X = {0}, Y = {1}, Z = {2})".format(rx,ry,rz))
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
            return "looking"
        except ValueError, ve:
            rospy.loginfo("lookat_answers = {0}".format(lookat_answers))
            rospy.loginfo("Further processing yielded {0}".format(ve))
            self.robot.speech.speak("I did not find an object.", block=False)
            return 'no_point_found'

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
            self.robot.speech.speak("I can't see a thing, but I'll try to be of service anyway. Wish me luck, or stop me before I do something silly.", block=False)
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
    #TODO: If ToggleModules works, make this state a specialization of that
    def __init__(self, robot, roi_query=None, on=True):
        smach.State.__init__(self, outcomes=["toggled"])
        self.robot = robot
        self.roi_query = roi_query
        self.on = on

    def execute(self, userdata=None):
        if self.on:
            result = self.robot.perception.toggle(["ppl_detection"])
        else:
            result = self.robot.perception.toggle([])
        return "toggled"

class ToggleModules(smach.State):
    """Enables or disables PeopleDetector"""
    def __init__(self, robot, modules):
        smach.State.__init__(self, outcomes=["toggled"])
        self.robot = robot
        self.modules = modules

    def execute(self, userdata=None):
        self.robot.perception.toggle(self.modules)
        return "toggled"

class ToggleDemoLaser(smach.State):
    """Toggle the demo laser service on. This allows to navigate relative to an object rather than to a static map."""
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "failed"])

        self.robot = robot
        self.demo_laser_service = rospy.ServiceProxy('/toggle_demo_laser', Empty)  

    def execute(self, userdata):
        try:
            self.demo_laser_service()
            return "done"
        except rospy.ServiceException:
            return "failed"

class StandingPeopleDetector(smach.State):
    """Enables or disables PeopleDetector"""
    #TODO: If ToggleModules works, make this state a specialization of that
    def __init__(self, robot, distance_to_walls=0.2, time=None, room=None, point_stamped=None, length_x=None, length_y=None, length_z=None):
        smach.State.__init__(self, outcomes=["done", "failed"])
        self.robot = robot
        self.time = time
        self.room = room
        self.point_stamped = point_stamped
        self.length_x = length_x
        self.length_y = length_y
        self.length_z = length_z
        self.distance_to_walls = int(distance_to_walls)  # This is the distance that is retracted from the room dimensions in x and y directions.

    def execute(self, userdata=None):

        # First check if room is given or already a point. 
        
        if self.room:
            
            room_dimensions = Compound("room_dimensions",self.room, Compound("size", "Xmin", "Ymin", "Zmin", "Xmax", "Ymax", "Zmax"))

            room_dimensions_answers = self.robot.reasoner.query(room_dimensions)


            if not room_dimensions_answers:
                rospy.logerr("Dimensions for room were not found in reasoner!!")
                return "failed"

            # print "room_dimensions_answers = ", room_dimensions_answers

            # assumed is that there is only one block per room.
            room_dimensions_answer = room_dimensions_answers[0]
            x_min = float(room_dimensions_answer["Xmin"])
            y_min = float(room_dimensions_answer["Ymin"])
            z_min = float(room_dimensions_answer["Zmin"])
            x_max = float(room_dimensions_answer["Xmax"])
            y_max = float(room_dimensions_answer["Ymax"])
            z_max = float(room_dimensions_answer["Zmax"])

            # print "\n x_min = ", x_min, "\n"
            # print "\n x_max = ", x_max, "\n"
            
            new_pointstamped = geometry_msgs.msg.PointStamped()
            new_pointstamped.point.x = (x_max-x_min)*0.5 + x_min
            new_pointstamped.point.y = (y_max-y_min)*0.5 + y_min
            new_pointstamped.point.z = (z_max-z_min)*0.5 + z_min
            new_pointstamped.header.frame_id = '/map'         

            # print "\n new_pointstamped = ", new_pointstamped, "\n"

            new_length_x = x_max-x_min - 2 * self.distance_to_walls
            new_length_y = y_max-y_min - 2 * self.distance_to_walls
            new_length_z = z_max-z_min - 2 * self.distance_to_walls

            # print "\n new_length_x = ", new_length_x, "\n"
            # print "\n new_length_y = ", new_length_y, "\n"
            # print "\n new_length_z = ", new_length_z, "\n"

            self.robot.perception.people_detection_torso_laser(new_pointstamped, self.time, abs(new_length_x), abs(new_length_y), abs(new_length_z))

        elif self.point_stamped:
            self.robot.perception.people_detection_torso_laser(self.point_stamped, self.time, self.length_x, self.length_y, self.length_z)
        else:
            rospy.logerr("No room or point was defined for detecting people")
            return "failed"

        return "done"

class CheckForPerson(smach.State):
    def __init__(self, robot, room=None, point_stamped=None, length_x=None, length_y=None, length_z=None):
        smach.State.__init__(self, outcomes=["found", "not_found"])

        self.robot = robot

        # understanding not yet used. But should be.
        self.room = room
        self.point_stamped = point_stamped
        self.length_x = length_x
        self.length_y = length_y
        self.length_z = length_z

    def execute(self, userdata=None):

        person_query = Conjunction( Compound( "property_expected", "ObjectID", "class_label", "person"),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X","Y","Z")),
                                    Compound( "not", Compound( "person_checked_positively", "ObjectID")),
                                    Compound( "not", Compound( "person_checked_negatively", "ObjectID")),
                                    Compound( "not", Compound( "person_unreachable", "ObjectID")))

        person_result = self.robot.reasoner.query(person_query)
        
        # TODO ERIK: ER KUNNEN PERSONEN NU BIJ ZITTEN DIE NIET IN DE KAMER ZITTEN WAAR JE GRAAG NAAR WIL KIJKEN. ZORGEN DAT DEZE CLASS OOK RUIMTE AFHANKELIJK IS.

        if not person_result:
            #self.robot.speech.speak("No one here.",block=False)
            return "not_found"

        print "person_result = ", person_result

        if len(person_result) > 1:
            self.robot.speech.speak("I think I see some people",block=False)
        else:
            self.robot.speech.speak("I think I found someone",block=False)

        # TODO ERIK: HIER BEPALEN DAT DE DICHTSBIJZIJNDE PERSOON GECONTROLEERD MOET WORDEN IPV DE EERSTE UIT DE LIJST.

        self.robot.reasoner.assertz(Compound("current_checked_person", person_result[0]["ObjectID"], Compound("point_3d", Sequence(person_result[0]["X"], person_result[0]["Y"], 1.5))))

        return "found"  


class CheckForFaces(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['detected','no_person','failed'])
        self.robot = robot

    def execute(self, userdata=None):

        # self.robot.spindle.reset()
        # self.robot.head.set_pan_tilt(tilt=-0.2)
        
        # # to make sure head is at right position.
        # rospy.sleep(0.5)

        query_detect_person = Conjunction(Compound("property_expected", "ObjectID", "class_label", "face"),
                                          Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                          Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        self.response_start = self.robot.perception.toggle(['face_segmentation'])

        if self.response_start.error_code == 0:
            rospy.loginfo("Face segmentation has started correctly")
        elif self.response_start.error_code == 1:
            rospy.logerr("Face segmentation failed to start")
            self.robot.head.look_up(tilt_vel=0.75)
            return "failed"

        wait_machine = reasoning.Wait_query_true(self.robot, query_detect_person, 4)
        wait_result = wait_machine.execute()

        rospy.loginfo("Face segmentation will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Face segmentation is stopped")
        elif self.response_stop.error_code == 1:
            self.robot.lights.set_color(1, 0, 0)
            rospy.sleep(0.1)
            self.robot.lights.set_color(0, 0, 1)
            rospy.loginfo("Failed stopping face segmentation")

        if wait_result == "timed_out":
            return "no_person"

        elif wait_result == "query_true":
            answers = self.robot.reasoner.query(query_detect_person)
            possible_locations = [( float(answer["X"]), 
                                    float(answer["Y"]), 
                                    float(answer["Z"])) for answer in answers]
            x,y,z = possible_locations[0]

            if z > 1.5:
                self.robot.spindle.high()
                rospy.logdebug("Spindle should come up now!")

            lookat_point = msgs.PointStamped(x,y,z)
            rospy.loginfo("AMIGO should look at person now. (x = {0}, y = {1}, z = {2})".format(x,y,z))
            self.robot.head.send_goal(lookat_point,timeout=0)

            return 'detected'



class StandingPeopleDetectorWithFace(smach.StateMachine):
    """Highlevel state. Possibility to use only Enables or disables PeopleDetector"""
    def __init__(self, robot, check_all_persons=False, distance_to_walls=0.2, time=4, room=None, point_stamped=None, length_x=None, length_y=None, length_z=None):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "no_person_found","failed"])
        self.robot = robot
        self.time = time
        self.room = room
        self.point_stamped = point_stamped
        self.length_x = length_x
        self.length_y = length_y
        self.length_z = length_z
        self.distance_to_walls = distance_to_walls  # This is the distance that is retracted from the room dimensions in x and y directions.
        self.check_all_persons = check_all_persons

        current_checked_person_query = Compound("current_checked_person", "ObjectID", Compound("point_3d",Sequence("X","Y","Z")))

        # removes previous defined dropoff points
        self.robot.reasoner.query(Compound("retractall", Compound("current_checked_person", "ObjectID", Compound("point_3d",Sequence("X","Y","Z")))))
        self.robot.reasoner.query(Compound("retractall",Compound("person_unreachable", "ObjectID")))
        self.robot.reasoner.query(Compound("retractall",Compound("person_checked_negatively", "ObjectID")))
        self.robot.reasoner.query(Compound("retractall",Compound("person_checked_positively", "ObjectID")))
        
        with self:
            smach.StateMachine.add("DETECT_PEOPLE_TORSO_LASER",
                                    StandingPeopleDetector(self.robot, distance_to_walls=self.distance_to_walls, time=self.time, room=self.room, point_stamped=self.point_stamped, length_x=self.length_x, length_y=self.length_y, length_z=self.length_z),
                                    transitions={   'done':'CHECK_FOR_PEOPLE',
                                                    'failed':'failed'})

            smach.StateMachine.add("CHECK_FOR_PEOPLE",
                                    CheckForPerson(self.robot, room=self.room, point_stamped=self.point_stamped, length_x=self.length_x, length_y=self.length_y, length_z=self.length_z),
                                    transitions={   'found':'NAVIGATE_TO_PERSON',
                                                    'not_found':'no_person_found'})

            smach.StateMachine.add( "NAVIGATE_TO_PERSON",
                                    navigation.NavigateGeneric(robot, lookat_query=current_checked_person_query, xy_dist_to_goal_tuple=(0.8,0)),
                                    transitions={   "arrived":"LOOK_AT_PERSON",
                                                    "unreachable":'SAY_UNREACHABLE_CHECKING_PERSON',
                                                    "preempted":'SAY_UNREACHABLE_CHECKING_PERSON',
                                                    "goal_not_defined":'SAY_UNREACHABLE_CHECKING_PERSON'})

            smach.StateMachine.add("SAY_UNREACHABLE_CHECKING_PERSON",
                                    human_interaction.Say(robot,"I was not able to check a possible found person", block=False),
                                    transitions={'spoken':'ASSERT_PERSON_UNREACHABLE'})

            smach.StateMachine.add("ASSERT_PERSON_UNREACHABLE",
                                reasoning.Execute_query(robot, [Conjunction(    Compound("current_checked_person", "ObjectID", Compound("point_3d",Sequence("X","Y","Z"))),
                                                                                Compound("assertz",Compound("person_unreachable", "ObjectID")),
                                                                                Compound("retractall", Compound("current_checked_person", "ObjectID", Compound("point_3d",Sequence("X","Y","Z")))))]),
                                transitions={'executed':'CHECK_FOR_PEOPLE'})

            smach.StateMachine.add('LOOK_AT_PERSON',
                                LookAtPoint(robot, lookat_query=current_checked_person_query),
                                transitions={   'looking':'CHECK_FOR_FACE',
                                                'no_point_found':'CHECK_FOR_FACE',
                                                'abort':'CHECK_FOR_FACE'}) # abort never happens in this state

            smach.StateMachine.add('CHECK_FOR_FACE',
                                CheckForFaces(robot),
                                transitions={   'detected':'SAY_SUCCES_CHECKING_PERSON',
                                                'no_person':'SAY_FAILED_CHECKING_PERSON',
                                                'failed':'SAY_FAILED_CHECKING_PERSON'})

            # TODO: Check if found person (face) is recognized before

            smach.StateMachine.add("SAY_SUCCES_CHECKING_PERSON",
                                    human_interaction.Say(robot,"Hi there!", block=False),
                                    transitions={'spoken':'ASSERT_PERSON_CHECKED_POSITIVELY'})

            smach.StateMachine.add("SAY_FAILED_CHECKING_PERSON",
                                    human_interaction.Say(robot,"Nope no one here", block=False),
                                    transitions={'spoken':'ASSERT_PERSON_CHECKED_NEGATIVELY'})

            smach.StateMachine.add("ASSERT_PERSON_CHECKED_NEGATIVELY",
                                reasoning.Execute_query(robot, [Conjunction(    Compound("current_checked_person", "ObjectID", Compound("point_3d",Sequence("X","Y","Z"))),
                                                                                Compound("assertz",Compound("person_checked_negatively", "ObjectID")),
                                                                                Compound("retractall", Compound("current_checked_person", "ObjectID", Compound("point_3d",Sequence("X","Y","Z")))))]),
                                transitions={'executed':'CHECK_FOR_PEOPLE'})

            smach.StateMachine.add("ASSERT_PERSON_CHECKED_POSITIVELY",
                                reasoning.Execute_query(robot, [Conjunction(    Compound("current_checked_person", "ObjectID", Compound("point_3d",Sequence("X","Y","Z"))),
                                                                                Compound("assertz",Compound("person_checked_positively", "ObjectID")),
                                                                                Compound("retractall", Compound("current_checked_person", "ObjectID", Compound("point_3d",Sequence("X","Y","Z")))))]),
                                transitions={'executed':'succeeded'}) # IN CASE MULTIPLE PERSONS NEED TO BE FOUND, MAKE NEW STATE.
            
            # TODO: amigo stops now when he found someone. The possibility to check everyone must still be build in.




