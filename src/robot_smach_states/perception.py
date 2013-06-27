#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach
#import object_msgs.msg
import math

import std_msgs.msg

import util

from sensor_msgs.msg import LaserScan
import geometry_msgs

from human_interaction import Say, Timedout_QuestionMachine, Say_generated, QuestionMachine
from reasoning import Wait_query_true, Retract_facts
from psi import Conjunction, Compound

import util.reasoning_helpers as urh

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

        # # learn left face
        # self.robot.leftArm.send_joint_goal(-1.159, 0.511, -1.021, 1.669, -0.603, 0.255, 0.0206,timeout=2)

        # self.robot.speech.speak(speech_sentence[0])
        # result = self.robot.perception.learn_person(name_to_learn, view = 'left', publish_while_learning = False)
        # if result == True:
        #     self.robot.reasoner.assertz(Compound("learned_person", name_to_learn, Compound("view", "left")))
        # self.robot.speech.speak("Finished learning your left side")

        # # learn right face
        # self.robot.leftArm.send_joint_goal(-1.39, 1.096, -0.967, 1.352, -0.9489, 0.5272, 0.0367,timeout=2)
        # rospy.sleep(1)
        # self.robot.leftArm.reset_arm()
        # self.robot.rightArm.send_joint_goal(-1.159, 0.511, -1.021, 1.669, -0.603, 0.255, 0.0206,timeout=2)

        # self.robot.speech.speak(speech_sentence[1])
        # result = self.robot.perception.learn_person(name_to_learn, view = 'right')
        # if result == True:
        #     self.robot.reasoner.assertz(Compound("learned_person", name_to_learn, Compound("view", "right")))
        # self.robot.speech.speak("Finished learning your right side")

        # # learn front face
        # self.robot.rightArm.send_joint_goal(-1.159, 1.096, -1.021, 1.669, -0.603, 0.255, 0.0206,timeout=2)
        # rospy.sleep(1)
        # self.robot.rightArm.reset_arm()


        self.robot.speech.speak(speech_sentence[2])
        result = self.robot.perception.learn_person(name_to_learn, view = 'front')
        if result == True:
            self.robot.reasoner.assertz(Compound("learned_person", name_to_learn, Compound("view", "front")))
        self.robot.speech.speak("Learning succeeded. Now I should recognize you, next time!")
        
        return 'face_learned'

# @util.deprecated
# class Learn_person_face_recognition(smach.State):
#     def __init__(self, robot):
#         smach.State.__init__(self,
#                              outcomes=['failed', 'succeeded'],
#                              input_keys=['rate','command','target'],
#                              output_keys=['target'])
#         self.robot = robot
#         self.no_of_introductions = 0
#         try:
#             assert hasattr(self.robot, "perception")
#         except AssertionError:
#             rospy.logerr("perception not available, but still trying without")
#             self.robot.speech.speak("I can't see a thing, but I'll try to be of service anyway. Wish me luck, or stop me before I do something silly.")

#     def execute(self, gl):
#         #rospy.Rate(rate).sleep()

#         self.robot.speech.speak("I see you. I will now start to learn your face.")
#         feedback = True
#         calibration_time_out = rospy.Duration(20)
#         current_time = rospy.Time.now()
#         self.robot.perception.learn_person(gl.target.name)#send_calibration_goal("operator")
#         counter = self.robot.perception.get_learn_face_counter()
#         while counter == []:
#             rospy.sleep(0.1)
#             if (rospy.Time.now() - current_time) > calibration_time_out:
#                 feedback = False
#                 break

#         #Do some feedback
#         rospy.loginfo("Starting feedback")
#         if feedback:
#             max = counter[0]
#             speech_feedback = ['Please look at my right arm', 'Please look at my left arm.', 'Beautiful']
#             left_arm_coordinates = [ [-0.1,-0.2,0.2,0.8,0.0,0.0,0.0], [-0.844,0.465,-0.72,1.5,0.05,0.0,0.0], [-0.1,-0.2,0.2,0.8,0.0,0.0,0.0] ]
#             right_arm_coordinates = [ [-0.844,0.465,-0.72,1.5,0.05,0.0,0.0], [-0.1,-0.2,0.2,0.8,0.0,0.0,0.0],  [-0.1,-0.2,0.2,0.8,0.0,0.0,0.0] ]
#             speak_freq = max / float(len(speech_feedback))
#             rospy.loginfo("[LEARN-FACE] speech freq: {0}".format(speak_freq))
#             i = 0
#             last_index = -1
#             try:
#                 while counter[len(counter) - 1] > 0:
#                     i = (max - counter[len(counter) - 1])
#                     #dirty hack
#                     if i == max: break

#                     index = int(round(i / speak_freq))
#                     if last_index != index:
#                         rospy.loginfo("Feedback")
#                         self.robot.leftArm.send_joint_goal(*left_arm_coordinates[index])
#                         self.robot.rightArm.send_joint_goal(*right_arm_coordinates[index])
#                         self.robot.speech.speak(speech_feedback[index])
#                         last_index = index
#                     elif (rospy.Time.now() - current_time) > calibration_time_out:
#                         feedback = False
#                         break
#             except IndexError:
#                 self.robot.leftArm.reset_arm()
#                 self.robot.rightArm.reset_arm()
#                 pass

#             self.robot.speech.speak("Learning succeeded. I will now start to follow you.")
#             return 'succeeded'

#         if feedback:
#             print "face calibration failed"
#             if self.no_of_introductions < 3:
#                 #self.robot.speech.speak("Please be patient. I need some more time")
#                 self.robot.speech.speak('I have trouble seeing you clearly, please look into my eyes.')
#                 self.no_of_introductions = self.no_of_introductions + 1
#                 return 'failed'
#             else:
#                 gl.target.name = ""
#                 rospy.loginfo("Backup plan initiated, face recognition failed")
#                 self.robot.speech.speak("backup-plan, just go")
#                 return 'succeeded'

# ############################## State Wait_for_door ###################################
# @util.deprecated
# class Wait_for_door(smach.State):
#     def __init__(self, robot=None):
#         smach.State.__init__(self, outcomes=['waiting', 'door_open'])
#         self.robot = robot
#         assert hasattr(self.robot, "reasoner")

#     def execute(self,userdata):
#         query = Compound("state", "door1", "open")  # maybe change door1 to entrance_door would be better

#         while not self.preempt_requested():
#             rospy.Rate(5).sleep()
#             answer = self.robot.reasoner.query(query)
#             if answer:
#                 return 'door_open'

# # Wait_for_door_2 is the same is Wait_for_door, but with the subscriber of the laser integrated and some not used userdata deleted.
# @util.deprecated
# class Wait_for_door2(smach.State):
#     def __init__(self, robot=None):
#         smach.State.__init__(self, outcomes=['waiting','door_open'],
#                                    input_keys=['rate','laser_value','distance','locations','target','challenge_duration'],
#                                    output_keys=['start_time','global_end_time'])
#         self.rate = 10
#         self.laser_reference = 0.0
#         self.laser_threshold = 1.0
#         self.door_open = 0
#         self.robot = robot
#         self.laser_value = 0.0

#         self.laser_listener = rospy.Subscriber("/middle_point",std_msgs.msg.Float32,self.callback_laser)

#     def callback_laser(self, data):
#         self.laser_value = data.data

#     def execute(self, userdata):

#         rospy.Rate(self.rate).sleep()

#         #laser_reference = gl.laser_value
#         # Only update reference value if it is not yet initialized, i.e. laser_reference = 0.0
#         if self.laser_reference == 0.0:
#             self.laser_reference = self.laser_value

#         if self.laser_value > (self.laser_reference + self.laser_threshold):
#             rospy.loginfo("{0} > ({1} + {2})".format(self.laser_value, self.laser_reference, self.laser_threshold))
#             self.door_open = 1
#             rospy.loginfo("Door open")

#             #sleep and update
#             #rospy.sleep(5)
#             #self.robot.base.clear_costmap()
#             return 'door_open'

#         #if self.door_open == 1:
#             #target_position = determine_target_position(gl.locations, gl.target)
#             #orientation_goal = self.robot.worldmodel.set_orientation(gl.locations, gl.target)
#             #send_base_goal(target_position, orientation_goal, "/map")

#             #if compute_distance(target_position, robot_position) < 0.20 and gl.approach_distance == 0.0:
#             #    send_base_goal(robot_position, orientation_goal, "/map")
#             #    result = wait_for_base(10)
#             #    return 'entered_room'
#             #else:
#             #    return 'entering room'

#         else:
#             rospy.loginfo("{0} <= ({1} + {2})".format(self.laser_value, self.laser_reference, self.laser_threshold))
#             return 'waiting'

# @util.deprecated
# class Learn_persons(smach.State):
#     def __init__(self, robot=None):
#         smach.State.__init__(self, outcomes=['finished_learning'],
#                                    input_keys=['rate','no_people_to_be_learned','similar_names_dictionary',
#                                                'name','target'],
#                                    output_keys=['learned_persons'])

#         self.no_people_learned = 0
#         self.start_time = rospy.Time.now()
#         self.sentence = ""
#         self.name_count = 1
#         self.names_learned = []
#         self.person_name = ""
#         self.robot = robot

#     def nameCount(self, initcount):
#         self.name_count = initcount
#         for i in self.names_learned:
#             if i == self.person_name:
#                 self.name_count = self.name_count+1

#     def execute(self, gl):

#         rospy.Rate(gl.rate).sleep()

#         # if not enough people are learned
#         print "The number of persons to be learned = ", gl.no_people_to_be_learned

#         self.robot.base.wait(20)

#         self.robot.speech.speak("I am ready to receive guests. Please come in and stand in front of me")

#         while self.no_people_learned < gl.no_people_to_be_learned and not rospy.is_shutdown():

#             rospy.Rate(gl.rate).sleep()

#             # only start if person is available
#             if self.robot.worldmodel.target_is_available(gl.target):

#                         # determine position
#                         target_position = self.robot.worldmodel.determine_target_position(gl.target)


#                         #self.robot.head.send_goal(target_position, "/map")


#                         print "Hello my name is Amigo, what is your name?"


#                         #self.robot.speech.speak("Hello, my name is Amigo, what is your name?")
#                         self.robot.speech.speak("Hello, my name is Amigo, what is your name?")


#                         # if no name is heard, try it again
#                         if gl.name == "":
#                                 #self.robot.speech.speak("I did not hear a name, can you repeat your name please?")
#                                 self.robot.speech.speak("I did not hear a name, can you repeat your name please?")


#                                 # If it again fails, use person as a name
#                                 if gl.name == "":
#                                         #self.robot.speech.speak("I am sorry, I did not understand your name")
#                                         self.robot.speech.speak("I am sorry, I did not understand your name")
#                                         self.person_name = "Person"

#                         else:
#                                 self.person_name = gl.name

#                         # If a name is heard, verify name
#                         if self.person_name != "Person":

#                                 latest_name_dummy = self.person_name

#                                 # check if name is already in list
#                                 self.nameCount(0)
#                                 # if yes, pick similar name instead
#                                 if self.name_count > 0:
#                                     if self.person_name in gl.similar_names_dictionary:
#                                         self.person_name = gl.similar_names_dictionary[self.person_name]

#                                 # ask if name is correct
#                                 self.sentence = "I heard " + self.person_name + ", is that correct?"
#                                 #self.robot.speech.speak(self.sentence)
#                                 self.robot.speech.speak(self.sentence)

#                                 if gl.name == "yes":

#                                         self.person_name = latest_name_dummy

#                                 # if the name was incorrect
#                                 if gl.name == "no":
#                                         #self.robot.speech.speak("I heard the wrong name, can you please repeat your name?")
#                                         self.robot.speech.speak("I heard the wrong name, can you please repeat your name?")
#                                         if gl.name:
#                                             self.person_name = gl.name

#                                 # if a name was heard
#                                 if gl.name:
#                                         # set the person name
#                                         #self.person_name = gl.name

#                                         # check if name is already in list
#                                         self.nameCount(0)
#                                         # if yes, pick similar name instead
#                                         if self.name_count > 0:
#                                             if self.person_name in gl.similar_names_dictionary:
#                                                 self.person_name = gl.similar_names_dictionary[self.person_name]
#                                                 self.person_name = gl.name

#                                 # No name heard, use person as a name
#                                 else:
#                                         print "I am sorry, no name"
#                                         #self.robot.speech.speak("I am sorry, I did not understand your name")
#                                         self.robot.speech.speak("I am sorry, I did not understand your name")
#                                         self.person_name = "Person"

#                         # (optional) Stop speech-to-text

#                         # Check if the name was heard before
#                         self.nameCount(1)
#                         print "the name_count = ", self.name_count

#                         # Add the name to the list of learned names
#                         self.names_learned.append(self.person_name)

#                         # If the name was heard before, add number to the name
#                         if self.name_count == 1:
#                                 self.sentence = ("I will call you " + self.person_name)
#                                 #self.robot.speech.speak(self.sentence)
#                                 self.robot.speech.speak(self.sentence)
#                         else:
#                                 self.person_name = self.person_name + str(self.name_count)
#                                 self.sentence = "I heard your name before, therefore, I will call you " + self.person_name
#                                 #self.robot.speech.speak(self.sentence,"auto")
#                                 self.robot.speech.speak(self.sentence)
#                                 self.name_count = 1 #reset name counter

#                         print self.sentence
#                         # Now create a recognition model
#                         #self.robot.speech.speak("I will now start making a model such that I am able to recognize you later",0)
#                         self.robot.speech.speak("I will now start making a model  such that I am able to recognize you later")
#                         feedback = self.robot.perception.learn_person(self.person_name)
#                         if not feedback:
#                                 print "Failed to calibrate face within the specified time"
#                                 feedback = self.robot.perception.learn_person(self.person_name)

#                         self.no_people_learned = self.no_people_learned + 1

#                         if len(self.names_learned) < gl.no_people_to_be_learned:
#                                 #self.robot.speech.speak("I am done making the model. The next person can enter","auto")
#                                 self.robot.speech.speak("I am done making the model. The next person can enter")
#                         else:
#                                 #self.robot.speech.speak("I am ready, I will now wait for...","auto")
#                                 self.robot.speech.speak("I am ready, I will now wait until someone tells me to continue...")

#         gl.learned_persons = self.names_learned
#         return 'finished_learning'

# ############################## State Identify ##############################
# @util.deprecated
# class Identify(smach.State):
#     def __init__(self, robot=None):
#         smach.State.__init__(self,
#                                    outcomes=['identifying','identified','who_is_who_finished','abort'],
#                                    input_keys=['rate','command','target'])
#         self.target = object_msgs.msg.ExecutionTarget()
#         self.sentence = ""
#         self.no_of_known_identifications = 0
#         self.total_no_of_identifications = 0
#         self.identifiedID = []
#         self.robot = robot

#     def execute(self, gl):

#         # start recognizing
#         #rospy.Rate(gl.rate).sleep()

#         if gl.command == "abort":
#             return 'abort'

#         # if target is set for unidentified person, as in challenge who_is_who
#         elif gl.target.name == "":
#             rospy.loginfo("gl.target.name='{0}'".format(gl.target.name))
#             # if unidentified person is available
#             if self.robot.worldmodel.target_is_available(gl.target):
#                 rospy.loginfo("target '{0}' is available".format(gl.target))

#                 # focus on target ID rather than name, since the name changes after the person has been identified
#                 target_ID = self.robot.worldmodel.determine_target_ID(gl.target)
#                 rospy.loginfo("determined new target_ID='{0}'".format(target_ID))

#                 target_position = self.robot.worldmodel.determine_target_position_by_ID(target_ID)

#                 # adjust for head height
#                 '''Disabled bacause of bugginess'''
#                 #self.robot.head.send_goal(target_position, "/map")

#                 rospy.loginfo("Start recognition")
#                 self.robot.perception.toggle_recognition(faces=True)
#                 #rospy.sleep(5)
#                 while not self.robot.worldmodel.target_is_available_by_ID(gl.target.ID):
#                     rospy.loginfo("Waiting for target with ID {0} to be available".format(gl.target.ID))
#                     rospy.sleep(1.0)
#                 self.robot.perception.toggle_recognition(faces=False)

#                 target_name = self.robot.worldmodel.determine_target_name_by_ID(target_ID)
#                 rospy.loginfo("ID {0} recognized as {1}".format(target_ID, target_name))

#                 if target_name == "unknown":

#                     self.sentence = ("Hello, I do not believe I know you")
#                     self.robot.speech.speak(self.sentence)

#                     self.total_no_of_identifications = self.total_no_of_identifications + 1

#                     if self.total_no_of_identifications >= 5:
#                         self.robot.speech.speak("I have found all the people I was looking for. Goodbye")
#                         return 'who_is_who_finished'
#                     else:
#                         return 'identified'

#                 elif target_name == None:

#                         return 'identifying'

#                 elif target_name != "":

#                     self.sentence = ("Hello " + target_name)
#                     self.robot.speech.speak(self.sentence)

#                     self.total_no_of_identifications = self.total_no_of_identifications + 1
#                     self.no_of_known_identifications = self.no_of_known_identifications + 1

#                     if self.total_no_of_identifications >= 5 or self.no_of_known_identifications >= 4:
#                         self.robot.speech.speak("I have found all the people I was looking for. Goodbye",0)
#                         return 'who_is_who_finished'
#                     else:
#                         return 'identified'

#                 else:
#                     return 'identifying'
#             else:
#                 return 'identifying'


#         # if target is set for a specific person, as in follow_me
#         elif gl.target.name != "":
#             rospy.logdebug("gl.target.name != '' but '{0}'".format(gl.target.name))
#             # set local temporary target to unidentified person
#             self.target.class_label = gl.target.class_label
#             self.target.name = ""

#             if self.robot.worldmodel.target_is_available(gl.target):
#                     self.robot.speech.speak("Hello operator, I will continue following you now")
#                     self.identifiedID = []
#                     return 'identified'

#             # Added Nov 10 2011
#             for i in range(len(self.robot.worldmodel.world_data.object)):

#                 if data.object[i].class_label == target.class_label:

#                      if not data.object[i].ID in self.identifiedID:

#                          target_position = self.robot.worldmodel.determine_target_position(self.robot.worldmodel.world_data[i])

#                          # adjust for head height
#                          target_position.z = target_position.z+ 0.3

#                          self.robot.head.send_goal(target_position, "/map")

#                          self.robot.perception.toggle_recognition(faces=True)
#                          rospy.sleep(5)
#                          self.robot.perception.toggle_recognition(faces=False)

#                          self.identifiedID.append(data.object[i].ID)

#                          return 'identifying'

#             # if unknown person is available
#             if self.robot.worldmodel.target_is_available(self.target):

#                 # focus on target ID rather than name, since the name changes after the person has been identified
#                 target_ID = self.robot.worldmodel.determine_target_ID(self.target)

#                 target_position = self.robot.worldmodel.determine_target_position_by_ID(target_ID)

#                 # adjust for head height
#                 target_position.z = target_position.z+ 0.3

#                 self.robot.head.send_goal(target_position, "/map")

#                 # wait
#                 self.robot.perception.toggle_recognition(faces=True)
#                 rospy.sleep(5)
#                 self.robot.perception.toggle_recognition(faces=False)

#                 target_name = self.robot.worldmodel.determine_target_name_by_ID(target_ID)

#                 if target_name == gl.target.name:


#                     self.robot.speech.speak("Hello operator, I will continue following you now")

#                     return 'identified'

#                 elif target_name != gl.target.name and target_name != "":

#                     self.robot.speech.speak("I am sorry, but I'm looking for my operator")

#                     self.robot.head.reset_position()
#                     return 'identifying'

#                 else:

#                     return 'identifying'
#             else:
#                 return 'identifying'

# ############################## State Identify ##############################
# @util.deprecated
# class Identify_follow(smach.State):
#     def __init__(self, robot=None):
#         smach.State.__init__(self, outcomes=['identifying','identified','who_is_who_finished','abort'],
#                                    input_keys=['rate','command','target','target_ID'],
#                                    output_keys=['target_ID'])

#         self.target = object_msgs.msg.ExecutionTarget()
#         self.sentence = ""
#         self.no_of_known_identifications = 0
#         self.total_no_of_identifications = 0
#         self.robot = robot

#     def execute(self, gl):

#         # start recognizing
#         rospy.Rate(gl.rate).sleep()

#         if gl.command == "abort":
#             return 'abort'




#         # if target is set for a specific person, as in follow_me
#         elif gl.target.name != "":

#             # set local temporary target to unidentified person
#             self.target.class_label = gl.target.class_label
#             self.target.name = ""

#             # if unknown person is available
#             if self.robot.worldmodel.target_is_available(self.target):

#                 # focus on target ID rather than name, since the name changes after the person has been identified
#                 target_ID = self.robot.worldmodel.determine_target_ID(self.target)

#                 target_position = self.robot.worldmodel.determine_target_position_by_ID(target_ID)

#                 # adjust for head height
#                 target_position.z = target_position.z+ 0.3

#                 self.robot.head.send_goal(target_position, "/map")

#                 # wait
#                 self.robot.perception.toggle_recognition(faces=True)
#                 rospy.sleep(5)


#                 target_name = self.robot.worldmodel.determine_target_name_by_ID(target_ID)

#                 self.robot.perception.toggle_recognition(faces=False)

#                 if target_name == gl.target.name:


#                     self.robot.speech.speak("Hello operator, I will continue following you now","auto")

#                     gl.target_ID = target_ID

#                     return 'identified'

#                 elif target_name != gl.target.name and target_name != "":

#                     self.robot.speech.speak("I am sorry, but I'm looking for my operator")

#                     self.robot.head.reset_position()


#                     return 'identifying'

#                 else:

#                     return 'identifying'
#             else:
#                 return 'identifying'

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
            basepos = self.robot.base.location[0]
            basepos = (basepos.x, basepos.y, basepos.z)
            selected_roi_answer = urh.select_answer(lookat_answers, 
                                                lambda answer: urh.xyz_dist(answer, basepos), 
                                                minmax=min)
            rx,ry,rz = urh.answer_to_tuple(selected_roi_answer)
            rospy.loginfo("[TEST ERIK] LOOKING AT (X = {0}, Y = {1}, Z = {2})".format(rx,ry,rz))
            lookat_point = self.robot.head.point(rx,ry,rz)
            print lookat_point

            # Send spindle goal to bring head to a suitable location
            # Correction for standard height: with a table heigt of 0.76 a spindle position
            # of 0.35 is desired, hence offset = 0.76-0.35 = 0.41
            # Minimum: 0.15 (to avoid crushing the arms), maximum 0.4
            # ToDo: do we need to incorporate wait functions?
            spindle_target = max(0.15, min(lookat_point.z - 0.41, self.robot.spindle.upper_limit))
            rospy.loginfo("Target height: {0}, spindle_target: {1}".format(lookat_point.z, spindle_target))

            self.robot.spindle.send_goal(spindle_target,waittime=5.0)
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
        target_point.point.x = lookat_point.x
        target_point.point.y = lookat_point.y
        target_point.point.z = lookat_point.z
        try:
            self.robot.perception.set_perception_roi(target_point, length_x=0.6, length_y=0.6, length_z=0.4)
        except Exception as e:
            rospy.loginfo("Cannot set perception roi for modules {0}".format(self.modules))
            rospy.loginfo("Error: {0}".format(e))

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
            lookat_point = self.robot.head.point(ox,oy,oz)
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
        self.robot.head.send_goal(self.point_stamped.point, frame_id=self.point_stamped.header.frame_id) #TODO Loy: Make all methods that take xyz and a frame_id take a PointStamped instead and define that convenience somewhere else

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

# @util.deprecated
# class Look(smach.State):
#         def __init__(self, robot, objects=False, persons=False, nr_targets=1, timeout = 10.0):
#                 smach.State.__init__(self, outcomes=['looking','object_found','person_found','nothing_found','abort'],
#                                          input_keys=['rate'])
#                 self.robot = robot
#                 self.objects=objects
#                 self.persons=persons
#                 self.nr_targets=nr_targets
#                 self.perception_timeout=rospy.Duration(timeout)

#         def execute(self, gl):

#             rospy.Rate(gl.rate).sleep()

#             #self.robot.head.look_down() #TODO Loy: removed this for the who is who @robocup. Not needed here.

#             if False: #gl.command == "abort": #TODO Loy: dirty hack to no be depending on gl.command
#                 self.robot.head.reset_position()
#                 return 'abort'
#             else:
#                 if (self.objects and not self.persons):
#                     ''' Looking for objects'''
#                     self.robot.head.look_down()

#                     rospy.loginfo("Start object recognition")
#                     result = self.robot.perception.toggle_recognition(objects=True)
#                     rospy.loginfo("Result starting object_recognition = {0}".format(result))

#                     # Let the object recognition run for a certain period
#                     # TODO: Make this prettier: wait for NEW entries in the world model instead of any entries
#                     #rospy.sleep(10.0)
#                     starttime_loop = rospy.Time.now()
#                     #rospy.loginfo("Number of world_model_targets ={0}".format(len(self.robot.worldmodel.search_target())))
#                     #rospy.loginfo("Time passed since starting object recognition = {0}".format(rospy.Time.now()-starttime_loop))
#                     ''' This doesn't work yet: search_target does not work if you don't give an argument '''
#                     ''' Probably a list with possible class_labels should be provided or the function should contain a region of interest '''
#                     '''while (len(self.robot.worldmodel.search_target()) < nr_targets and (rospy.Time.now()-starttime_loop)<rospy.Duration(self.perception_timeout)):
#                         rospy.loginfo("Waiting for object recognition")
#                         rospy.sleep(0.5)'''

#                     rospy.sleep(10.0)

#                     rospy.loginfo("Stop object recognition")
#                     result = self.robot.perception.toggle_recognition(objects=False)
#                     rospy.sleep(5.0)

#                     '''if len(self.robot.worldmodel.search_target()) > 0:
#                         return 'object_found'
#                     else:
#                         return 'nothing_found' '''

#                     self.robot.head.reset_position()

#                     return 'object_found'

#                 elif (self.persons and not self.objects):
#                     ''' Looking for persons '''

#                     rospy.loginfo("Start people recognition")
#                     result = self.robot.perception.toggle_recognition(faces=True)

#                     # Let the people recognition run for a certain period
#                     # TODO: Make this prettier: wait for NEW entries in the world model instead of any entries
#                     #rospy.sleep(10.0)
#                     starttime_loop = rospy.Time.now()
#                     while (len(self.robot.worldmodel.search_target(class_label='person')) < nr_targets and (rospy.Time.now()-starttime_loop)<rospy.Duration(self.perception_timeout)):
#                         rospy.loginfo("Waiting for people recognition")
#                         rospy.sleep(0.5)

#                     rospy.loginfo("Stop object recognition")
#                     result = self.robot.perception.toggle_recognition(faces=False)
#                     rospy.sleep(5.0)

#                     if len(self.robot.worldmodel.search_target()) > 0:
#                         return 'person_found'
#                     else:
#                         return 'nothing_found'

#                 elif (self.persons and self.objects):
#                     ''' Looking for both persons as well as objects '''
#                     rospy.logwarn("Looking at both persons and objects not yet implemented correctly")

#                     result = self.robot.perception.toggle_recognition(objects=True,faces=True)

#                     # Let the object recognition run for a certain period
#                     # TODO: Make this variable
#                     rospy.sleep(10.0)

#                     rospy.loginfo("Stop object recognition")
#                     result = self.robot.perception.toggle_recognition(objects=False,faces=False)
#                     rospy.sleep(5.0)

#                     if len(self.robot.worldmodel.search_target()) > 0:
#                         return 'object_found'
#                     else:
#                         return 'nothing_found'

#                 else:
#                     ''' Not looking for anything '''
#                     rospy.logwarn("Looking for neither objects nor persons")
#                     return 'nothing_found'

#                 ''' ***OBSOLETE***
#                 rospy.loginfo("Looking for an object, target={0}".format(gl.target))

#                 #This is NOT needed, the body_detector works continuously.
#                 #This is needed when identifying people/objects though

#                 #if gl.target.class_label == 'person':
#                 #    self.robot.perception.toggle_recognition(faces=True)
#                 #else:
#                 #    self.robot.perception.toggle_recognition(objects=True)

#                 rospy.Rate(gl.rate).sleep()

#                 #self.robot.head.search_movement() #Removed for who is who, just keep looking straight ahead

#                 #Loy 20-4-12: this line was: self.robot.worldmodel.target_is_available(gl.target)
#                 #Testing if actively searching works better
#                 found_targets = self.robot.worldmodel.search_target(label=gl.target.class_label)
#                 if found_targets:
#                         found_target = found_targets[0] #TODO Loy: get closest
#                         gl.target = found_target
#                         return 'found'
#                 else:
#                         return 'looking' '''

# @util.deprecated
# class Wait_for_Object(smach.State):
#     def __init__(self,
#                  object_class_label=None,
#                  object_ID=None,
#                  object_name=None,
#                  waittime=60,
#                  robot=None,
#                  range=2.0):
#         smach.State.__init__(self,
#                        input_keys=['rate'],
#                        outcomes=['found','not_in_range','timed_out', 'aborted'],#, 'waiting_for_object'],
#                        output_keys=['target'])
#         self.robot = robot

#         self.object_class_label = object_class_label
#         self.object_ID = object_ID
#         self.object_name = object_name

#         self.waittime = waittime
#         self.range = range

#     def execute(self, userdata):
#         starttime = rospy.Time.now()

#         while rospy.Time.now() < starttime + rospy.Duration(self.waittime) and not rospy.is_shutdown():
#             target = self.robot.closest_target(ID    = self.object_ID,
#                                                          class_label  = self.object_class_label,
#                                                          name   = self.object_name)
#             if target:
#                 #target = targets[0]
#                 rospy.loginfo("Matched object: {0}".format(self.robot.worldmodel.repr_obj(target)))

#                 target_loc = target.pose.position
#                 robot_loc = self.robot.base.location[0]

#                 rospy.loginfo('robot.loc = ({0:.2f}, {1:.2f}, {2:.2f})'.format(robot_loc.x, robot_loc.y, robot_loc.z))

#                 dist = util.transformations.compute_distance(target_loc, robot_loc)
#                 rospy.loginfo("Distance to matched object: {0}".format(dist))

#                 if dist < self.range:
#                     userdata.target = target
#                     return 'found'
#                 else:
#                     return 'not_in_range'
#             else:
#                 rospy.Rate(userdata.rate).sleep()
#                 #return 'looking'

#             if self.preempt_requested() or rospy.is_shutdown():
#                 return 'aborted'
#         else:
#             if self.preempt_requested():
#                 return 'aborted'
#             else:
#                 return 'timed_out'

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

        self.laser_listener = rospy.Subscriber("/base_scan",LaserScan,self.callback_laser)
        try:
            assert hasattr(self.robot, "perception")
        except AssertionError:
            rospy.logerr("perception not available, but still trying without")
            self.robot.speech.speak("I can't see a thing, but I'll try to be of service anyway. Wish me luck, or stop me before I do something silly.")
        assert hasattr(self.robot, "reasoner")

    def callback_laser(self, data):
        laser_range = data.ranges
        index = len(laser_range)/2
        
        if data.header.frame_id == "/front_laser":
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
