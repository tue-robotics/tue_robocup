#! /usr/bin/env python

#TODO: 
# - what do you want on your sandwich?
#   - Jam
#   - peanut butter
#   - cheese
#   - salami
#   - chocolade

import roslib; roslib.load_manifest('challenge_demo')
import rospy

import smach

from speech_interpreter.srv import GetInfo
from robot_skills.amigo import Amigo
from robot_skills.arms import State as ArmState
import robot_smach_states as states

from robot_smach_states.util.startup import startup #gives you speech-exception reading-out-loud and smach_viewer server

from psi import Conjunction, Compound

from tf.transformations import euler_from_quaternion

from interpret_email import MailInterpreter
from maluuba_ros.srv import Interpret
from speech_interpreter.srv import GetYesNo

import datetime

# DONE : Get breakfast from intermediate pos: 2.24, -3.73, -1.95 TEST
# Aankijken voor vragen CHECK OF NODIG IS
# 2x opties moet 1x TEST
# Beide posities richting personen TEST


HOLD_TRAY_POSE =        [-0.1,  0.13,   0.4,    1.5,    0,      0.5,    0]
SUPPORT_PATIENT_POSE =  [-0.1,  -1.57,  0,      1.57,   0,      0,      0]
RESET_POSE =            [-0.1,  0.13,   0,      0.3,    0,      0.0,    0]
HOLD_CAN_POSE =         [-0.1, -0.3,    0.0,    1.87,   0.1,    0.0,    0.0]

KITCHEN_LOC = (2.24, -3.73, -1.95)#Compound("sink", "a")
BREAKFAST_1 = "breakfast_1"
BREAKFAST_2 = "breakfast_2"
SINGPOS = Compound("dinner_table", "a")
BOWL_POS = "bowl_prior"

query_dropoff_loc = Compound("dropoff_point", "trash_bin", Compound("point_3d", "X", "Y", "Z"))

class TurnAround(smach.State):
    def __init__(self, robot, angle):
        smach.State.__init__(self, outcomes=["done" , "failed"])
        self.robot = robot
        self.angle = angle

    def execute(self, userdata):
        orig = self.robot.base.location
        orig_pos = orig[0]
        orig_orient = orig[1]
        orig_orient = [orig_orient.x, orig_orient.y, orig_orient.z, orig_orient.w]
        angle = list(euler_from_quaternion(orig_orient))
        angle[2] += self.angle
        new_orient = self.robot.base.orient(angle[2])

        result = self.robot.base.send_goal(orig_pos, new_orient)
        if result:
            return "done"
        else:
            return "failed"


class AskHowFeel(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

        self.get_status_person_service = rospy.ServiceProxy('interpreter/get_info_user', GetInfo)

    def execute(self, userdata=None):
        #TODO:self.robot.head.reset_position(), of look_down. 
        try:
            self.response = self.get_status_person_service("demo_challenge_status_person", 4 , 60)  # This means that within 4 tries and within 60 seconds an answer is received. 
            if self.response.answer == "no_answer" or self.response.answer == "wrong_answer":
                self.robot.speech.speak("I will just service you some breakfast", mood="neutral")
            elif self.response.answer == "fine" or self.response.answer == "good":
                self.robot.speech.speak("Maybe I can make it even better by serving some breakfast", mood="excited")
            elif self.response.answer == "bad" or self.response.answer == "mwah" or self.response.answer == "sick" or self.response.answer == "ill":
                self.robot.speech.speak("Maybe I can make it a little better by serving some breakfast", mood="sad")
        except Exception, e:
            rospy.logerr("Could not get_status_person_service: {0}.".format(e))
            
        return "done"


class AskBreakfast(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done" , "failed"])
        self.robot = robot
        self.get_breakfast_question_service = rospy.ServiceProxy('interpreter/get_info_user', GetInfo)
        self.ask_breakfast_failed = 0
        self.person_breakfast = 0

    def execute(self, userdata=None):     
        self.response = self.get_breakfast_question_service("demo_challenge_breakfast", 3 , 60)  # This means that within 4 tries and within 60 seconds an answer is received. 

        if self.response.answer == "no_answer" or  self.response.answer == "wrong_answer":
            if self.ask_breakfast_failed == 1:
                self.robot.speech.speak("I will just give you a sandwich with peanutbutter")
                self.response.answer = "peanutbutter"
            if self.ask_breakfast_failed == 0:
                self.robot.speech.speak("I will just give you a sandwich with cheese")
                self.response.answer = "cheese"
                self.ask_breakfast_failed = 1
        else:
            self.robot.speech.speak("I will bring you a sandwich with {0}".format(self.response.answer).replace('_', ' '))

        if self.person_breakfast == 1:
            self.robot.reasoner.query(Compound("assert", Compound("breakfast", self.response.answer)))
        if self.person_breakfast == 0:
            self.robot.reasoner.query(Compound("assert", Compound("breakfast", self.response.answer)))
            self.person_breakfast = 1          

        return_result = self.robot.reasoner.query(Compound("breakfast", "Breakfast"))        
        if not return_result:
            self.robot.speech.speak("That's horrible, I forgot what breakfast you want. I should see a doctor!")
            return "failed"
        return "done"


class AskAnythingElse(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

        self.get_anything_else_service = rospy.ServiceProxy('interpreter/get_info_user', GetInfo)
        self.maluubasrv = rospy.ServiceProxy('maluuba/interpret', Interpret)
        self.mail_interpreter = MailInterpreter(open(roslib.packages.get_pkg_dir("challenge_demo") + "/config/mailconfig.yaml"))
        self.get_yes_no_service = rospy.ServiceProxy('interpreter/get_yes_no', GetYesNo)


    def tell_time(self):
        import time
        timestr = time.strftime( "%M past %I", time.localtime(time.time())) #%I is hour in 12-hour format
        self.robot.speech.speak("It is now {0}".format(timestr), replace={" 0":" "})

    def remind(self):
        response = self.maluubasrv(self.response.answer)
        interpretation = response.interpretation
        start, end = self.mail_interpreter.extract_times(interpretation.entities)
        start = self.mail_interpreter.datetime_as_sentence(start)
        
        #self.robot.speech.speak("Do you want a reminder to {0}, {1}?".format(interpretation.entities.message, start))
        
        #resp = self.get_yes_no_service(2 , 8) # 3 tries, each max 10 seconds
        
        self.robot.speech.speak("Please look at me, so I can recognize you and send the reminder to your phone")
        self.response_start = self.robot.perception.toggle(["face_recognition"])
        rospy.sleep(6.0)
        rospy.loginfo("Face recognition will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        person_result = self.robot.reasoner.query(
                                        Conjunction(  
                                            Compound( "property_expected", "ObjectID", "class_label", "face"),
                                            Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                            Compound( "property", "ObjectID", "name", Compound("discrete", "DomainSize", "NamePMF"))))
        # Interpret face regnition results
        name = "loy"
        rospy.loginfo(person_result)
        if person_result:
            try:
                name_possibility = person_result[0]["NamePMF"]
                name = str(name_possibility[0][1][0])
            except IndexError:
                self.robot.speech.speak("Not sure about your name, but Loy always forgets everything, so I'll remind him!".format(name))
        else:
            self.robot.speech.speak("Not sure about your name, but Loy always forgets everything, so I'll remind him!".format(name))
        
        self.robot.speech.speak("Allright {0}, check your phone in a minute for the reminder.".format(name), block=False)
        #import ipdb; ipdb.set_trace()
        self.mail_interpreter.process_interpretation(interpretation, receiver=name)

    def execute(self, userdata=None):
        try:
            self.response = self.get_anything_else_service("demo_challenge_anything_else", 4 , 60)  
            # This means that within 4 tries and within 60 seconds an answer is received. 
            #import ipdb; ipdb.set_trace()
            if not self.response.answer == "no_answer" or self.response.answer == "wrong_answer":
                self.response.answer = self.response.answer.replace("_", " ")

            if self.response.answer == "no_answer" or self.response.answer == "wrong_answer":
                rospy.logwarn("No answer or wrong answer not yet implemented")
                ## WHAT TO DO IF FAILURE TODO JANNO

                #import time, random
                #timestr = time.strftime( "it's %M past %H", time.localtime(time.time()))
                #self.robot.speech.speak("It is now {0}".format(timestr))

            #elif self.response.answer == "Can you tell me what time it is?":
            elif "time" in self.response.answer:
                self.tell_time()
            elif "remind me" in self.response.answer.lower():                
                self.remind()
            else:
                rospy.logwarn("Answer not understood")
                self.robot.speech.speak("I am terribly sorry but I do not know what you mean", mood="sad", block=False)
                self.robot.speech.speak("I better go and get your breakfast", mood="neutral", block=False)
        except Exception, e:
            rospy.logerr("Could not get_status_person_service: {0}.".format(e))
            self.robot.speech.speak("I'm not sure what to do, sorry about that.")
            self.robot.speech.speak("I better go and get your breakfast", mood="neutral", block=False)
        return "done"


class TalkToCook(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:
            smach.StateMachine.add( 'REPORT', 
                                    states.Say(robot, ["Hi cook, I will wake the patients, escort them to the breakfast and take their breakfast orders. I'll be back!"]), 
                                    transitions={   'spoken':"Done"})

class EscortToBreakfast(smach.StateMachine):
    def __init__(self, robot, patient_destination_query):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:
            smach.StateMachine.add( 'SUPPORT_LEFT',
                                    states.ArmToJointPos(robot, robot.leftArm, SUPPORT_PATIENT_POSE, timeout=4.0),
                                    transitions={'done':'SUPPORT_RIGHT',
                                                 'failed':'SUPPORT_RIGHT'})

            smach.StateMachine.add( 'SUPPORT_RIGHT',
                                    states.ArmToJointPos(robot, robot.rightArm, SUPPORT_PATIENT_POSE, timeout=4.0),
                                    transitions={'done':'DUMMY',
                                                 'failed':'DUMMY'})
            smach.StateMachine.add( 'DUMMY', 
                                    states.Say(robot, ["Please follow me, I'll bring you to the breakfast room. You can support yourself by using my arms"]), 
                                    transitions={   'spoken':"GOTO_BREAKFAST"})
            
            smach.StateMachine.add( 'GOTO_BREAKFAST', 
                                    states.NavigateGeneric(robot, goal_query=patient_destination_query), 
                                    transitions={   "arrived":"Done",
                                                    "unreachable":"Done",
                                                    "preempted":"Aborted",
                                                    "goal_not_defined":"Aborted",})

class ResetRobot(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.spindle.reset()
        self.robot.leftArm.send_gripper_goal_close()
        self.robot.rightArm.send_gripper_goal_close()
        self.robot.leftArm.reset_arm()
        self.robot.rightArm.reset_arm()
        self.robot.head.reset_position()
        return 'done'

class InteractionPart(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['done'])

        with self:
            # smach.StateMachine.add("INITIAL_RESET_ROBOT",
            #                         ResetRobot(robot),
            #                         transitions={   'done'              : 'SAY_HOW_FEEL'})


            smach.StateMachine.add( "SAY_HOW_FEEL",
                                    states.Say(robot, "How are you today?"),
                                    transitions={   'spoken'            : "ASK_HOW_FEEL"})
           
            smach.StateMachine.add('ASK_HOW_FEEL',
                                    AskHowFeel(robot),
                                    transitions={   "done"              : "ASK_WHAT_FOR_BREAKFAST"})

            # smach.StateMachine.add( "RECITE_BREAKFAST_OPTIONS",
            #                         states.Say(robot, "On today's breakfast menu, we have sandwiches with jam, salami, cheece, peanut butter or chocolate. Which do you want?"),
            #                         transitions={   'spoken'            : "ASK_WHAT_FOR_BREAKFAST"})

            smach.StateMachine.add( 'ASK_WHAT_FOR_BREAKFAST', 
                                    AskBreakfast(robot),
                                    transitions={   'done'              : 'SAY_ANYTHING_ELSE',
                                                    'failed'            : 'SAY_ANYTHING_ELSE'})

            smach.StateMachine.add( "SAY_ANYTHING_ELSE",
                                    states.Say(robot, "Is there anything else I can do for you?"),
                                    transitions={   'spoken'            : "ASK_ANYTHING_ELSE"})

            smach.StateMachine.add( 'ASK_ANYTHING_ELSE',
                                    AskAnythingElse(robot),
                                    transitions={   'done'              : 'done'})

#class PoorChocolateNuts(smach.State):
#    def __init__(self, robot):
#        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
#        self.robot = robot
#
#    def execute(self, userdata=None):
#
#        #poor_poi_query = 
#        answers = self.robot.reasoner.query(poor_poi_query)
#        if not answers:
#            rospy.logerr("Point of interest for pooring not defined, aborting pooring")
#            return 'failed'

class PoorChocolateNuts(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        lookat_query = Compound("point_of_interest", BOWL_POS, Compound("point_3d", "X", "Y", "Z")) 
        bowl_query = Compound("position", "ObjectID", Compound("point", "X", "Y", "Z"))#TODO: Check for correct type
        grabpoint_query = Conjunction(  Compound("current_object", "ObjectID"),
                                        Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))

        @smach.cb_interface(outcomes=['succeeded','failed'])
            def send_laser_goal(userdata, laser_target, timeout):
                if self.robot.spindle.send_laser_goal(laser_target, timeout=timeout):
                    return 'succeeded'
                else:
                    return 'failed'

        @smach.cb_interface(outcomes=['done'])
            def assert_prior_bowl_pos(userdata):
                answers = self.robot.reasoner.query(lookat_query)
                if answers:
                    answer = answers[0]
                    self.robot.reasoner.assertz(Compound("position", "bowl-fixed", Compound("point", answer["X"], answer["Y"], answer["Z"])))
                else:
                    self.robot.reasoner.assertz(Compound("position", "bowl-fixed", Compound("point", 6.0, -0,43, 0.82)))

                self.robot.reasoner.assertz(Compound("current_object", "bowl-fixed"))


        with self:
            smach.StateMachine.add( "LOWER_LASER",
                                    smach.CBState(send_laser_goal,cb_kwargs={'laser_target':82, 'timeout':4}),
                                    transitions={   'succeeded'             : 'LOOK_FOR_BOWL',
                                                    'failed'                : 'ASSERT_PRIOR_BOWL_POS'})

            smach.StateMachine.add( "ASSERT_PRIOR_BOWL_POS",
                                    smach.CBState(assert_prior_bowl_pos),
                                    transitions={   'done'                  : 'PREPARE_GRAB'})

            smach.StateMachine.add( "LOOK_FOR_BOWL",
                                    states.LookForObjectsAtROI(robot, lookat_query, bowl_query, modules=["object_detector_2d"], maxdist=0.3, waittime=5.0),
                                    transitions={   'object_found'          : 'PRE_POOR_POS',
                                                    'looking'               : 'ASSERT_PRIOR_BOWL_POS',
                                                    'no_object_found'       : 'ASSERT_PRIOR_BOWL_POS',
                                                    'abort'                 : 'ASSERT_PRIOR_BOWL_POS'})
            # #'looking','object_found','no_object_found','abort'
            smach.StateMachine.add('PREPARE_GRAB', 
                                    states.PrepareGrasp(robot.leftArm, robot, grabpoint_query),
                                    transitions={   'succeeded'             :   'PREPARE_ORIENTATION',
                                                    'failed'                :   'failed'})

            smach.StateMachine.add( "PREPARE_ORIENTATION", 
                                    states.PrepareOrientation(robot.leftArm, robot, grabpoint_query),
                                    transitions={   'orientation_succeeded' : 'PRE_POOR_POS',
                                                    'orientation_failed'    : 'PRE_POOR_POS',
                                                    'abort'                 : 'failed',
                                                    'target_lost'           : 'failed'})

            smach.StateMachine.add( "PRE_POOR_POS", 
                                    states.ArmToQueryPoint(robot, robot.leftArm, grabpoint_query, time_out=20, pre_grasp=True, first_joint_pos_only=True),
                                    transitions={   'succeeded'             : 'POOR_POS1',
                                                    'failed'                : 'CARRY_POSE_FAIL'})

            smach.StateMachine.add( "POOR_POS1", 
                                    states.ArmToUserPose(robot.leftArm, 0.1, 0.1, 0.25, 0.0, 0.0 , 0.0, 
                                                                time_out=20, pre_grasp=False, frame_id="/base_link", delta=True),
                                    transitions={   'succeeded'             : 'POOR_POS2',
                                                    'failed'                : 'CARRY_POSE_FAIL'})

            smach.StateMachine.add( "POOR_POS2", 
                                    states.ArmToUserPose(robot.leftArm, 0.1, 0.0, 0.0, 0.0, 0.0 , 0.0, 
                                                                time_out=20, pre_grasp=False, frame_id="/base_link", delta=True),
                                    transitions={   'succeeded'             : 'POOR1',
                                                    'failed'                : 'CARRY_POSE_FAIL'})

            #smach.StateMachine.add( "POOR",
            #                       states.ArmToUserPose(robot.leftArm, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 
            #                            time_out=20, pre_grasp=False, frame_id="/base_link", delta=True),
            #                        transitions={   'succeeded'             : 'RETRACT',
            #                                       'failed'                : 'CARRY_POSE_FAIL'})

            smach.StateMachine.add( "POOR1",
                                    states.ArmToJointPos(robot, robot.leftArm, [0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0], delta=True, timeout=5.0),
                                    transitions={   'done'                  : 'POOR2',
                                                    'failed'                : 'CARRY_POSE_FAIL'})

            smach.StateMachine.add( "POOR2",
                                    states.ArmToJointPos(robot, robot.leftArm, [0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0], delta=True, timeout=5.0),
                                    transitions={   'done'                  : 'POOR3',
                                                    'failed'                : 'CARRY_POSE_FAIL'})

            smach.StateMachine.add( "POOR3",
                                    states.ArmToJointPos(robot, robot.leftArm, [0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0], delta=True, timeout=5.0),
                                    transitions={   'done'                  : 'POOR4',
                                                    'failed'                : 'CARRY_POSE_FAIL'})

            smach.StateMachine.add( "POOR4",
                                    states.ArmToJointPos(robot, robot.leftArm, [0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0], delta=True, timeout=5.0),
                                    transitions={   'done'                  : 'POOR5',
                                                    'failed'                : 'CARRY_POSE_FAIL'})

            smach.StateMachine.add( "POOR5",
                                    states.ArmToJointPos(robot, robot.leftArm, [0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0], delta=True, timeout=5.0),
                                    transitions={   'done'                  : 'POOR6',
                                                    'failed'                : 'CARRY_POSE_FAIL'})

            smach.StateMachine.add( "POOR6",
                                    states.ArmToJointPos(robot, robot.leftArm, [0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0], delta=True, timeout=5.0),
                                    transitions={   'done'                  : 'RETRACT',
                                                    'failed'                : 'RETRACT'})

            smach.StateMachine.add('RETRACT', 
                                    states.ArmToUserPose(robot.leftArm, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, time_out=20, pre_grasp=False, frame_id="/base_link", delta=True),
                                    transitions={   'succeeded'             : 'CARRY_POSE_SUCCEEDED',
                                                    'failed'                : 'CARRY_POSE_SUCCEEDED'})

            smach.StateMachine.add( "CARRY_POSE_SUCCEEDED",
                                    states.PrepareGrasp(robot.leftArm, robot, grabpoint_query),
                                    transitions={   'succeeded'             :   'succeeded',
                                                    'failed'                :   'succeeded'})

            smach.StateMachine.add( "CARRY_POSE_FAIL",
                                    states.PrepareGrasp(robot.leftArm, robot, grabpoint_query),
                                    transitions={   'succeeded'             :   'failed',
                                                    'failed'                :   'failed'})

            #target_position_bl = transformations.tf_transform(target_position, "/map","/base_link", tf_listener=self.robot.tf_listener)


class SingSong(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata=None):
        import os
        TTS_EXE_FILE = "~/ros/fuerte/tue/trunk/tue_human_machine_interfacing/text_to_speech_philips/exec/ptts_v911.exe"
        filename = "/tmp/speech.wav"
        os.system("rm {0}".format(filename))
        command = TTS_EXE_FILE+" -i ~/ros/fuerte/tue/trunk/tue_human_machine_interfacing/text_to_speech_philips/singing_US.txt -k xcyst4l363x6c5j40tzz-v4d6kgt0tr0c7hj1l3lq-cmsn39tskyz9cvwv4z -o {0}".format(filename)
        os.system(command)
        os.system("aplay {0}".format(filename))

        return 'done'


class Part1(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        with self:
            smach.StateMachine.add( 'GOTO_BREAKFAST_TABLE', 
                                    states.NavigateGeneric(robot, goal_name=BREAKFAST_1), 
                                    transitions={   "arrived"           : "INTERACTION",
                                                    "unreachable"       : "SAY_CANNOT_REACH_BREAKFAST_TABLE",
                                                    "preempted"         : "failed",
                                                    "goal_not_defined"  : "SAY_CANNOT_REACH_BREAKFAST_TABLE"})

            smach.StateMachine.add( 'INTERACTION',
                                    InteractionPart(robot),
                                    transitions={   'done'              : 'GOTO_KITCHEN'})

            smach.StateMachine.add( 'GOTO_KITCHEN', 
                                    states.NavigateGeneric(robot, goal_pose_2d=KITCHEN_LOC), 
                                    transitions={   "arrived"           : "REPORT_BREAKFAST",
                                                    "unreachable"       : "REPORT_BREAKFAST",
                                                    "preempted"         : "failed",
                                                    "goal_not_defined"  : "REPORT_BREAKFAST"})

            smach.StateMachine.add( 'SAY_CANNOT_REACH_BREAKFAST_TABLE',
                                    states.Say(robot, ["I better go to the cook straight away"]), 
                                    transitions={   'spoken'            : "GOTO_KITCHEN_BACKUP"})

            smach.StateMachine.add( 'GOTO_KITCHEN_BACKUP', 
                                    states.NavigateGeneric(robot, goal_pose_2d=KITCHEN_LOC), 
                                    transitions={   "arrived"           : "SAY_KITCHEN_BACKUP",
                                                    "unreachable"       : "SAY_KITCHEN_BACKUP",
                                                    "preempted"         : "failed",
                                                    "goal_not_defined"  : "SAY_KITCHEN_BACKUP"})

            smach.StateMachine.add( 'SAY_KITCHEN_BACKUP',
                                    states.Say(robot, ["Can you please handover the tray so i can take it to the patients?"]), 
                                    transitions={   'spoken'            : "HOLDUP_ARMS_FOR_TRAY_LEFT"})

            def generate_report_sentence(*args,**kwargs):
                try:
                    breakfasts  = robot.reasoner.query(Compound("breakfast", "Breakfast"))
                    #patients    = robot.reasoner.query(Compound("current_patient", "Patient"))
                    breakfast   = breakfasts[0]["Breakfast"]
                    #patient     = patients[0]["Patient"]
                    #return "{0} wants {1} for breakfast. If you give it to me, i'll bring it to {0}. I'll wait here for a second so you can give it to me".format(patient, breakfast)
                    return "Can you please put a sandwich with {0} on a tray so I can serve that for breakfast?".format(breakfast).replace('_', ' ')
                except:
                    return "I forgot what I should get for breakfast"
            smach.StateMachine.add('REPORT_BREAKFAST',
                                    states.Say_generated(robot, sentence_creator=generate_report_sentence),
                                    transitions={ 'spoken':'HOLDUP_ARMS_FOR_TRAY_LEFT' })

            smach.StateMachine.add( 'HOLDUP_ARMS_FOR_TRAY_LEFT', 
                                    states.ArmToJointPos(robot, robot.leftArm, HOLD_TRAY_POSE, timeout=4.0),
                                    transitions={   'done'              : "HOLDUP_ARMS_FOR_TRAY_RIGHT",
                                                    'failed'            : "RESET_ROBOT_NO_ARMS"})

            smach.StateMachine.add( 'HOLDUP_ARMS_FOR_TRAY_RIGHT', 
                                    states.ArmToJointPos(robot, robot.rightArm, HOLD_TRAY_POSE, timeout=4.0),
                                    transitions={   'done'              : "WAIT_FOR_LOAD",
                                                    'failed'            : "RESET_ROBOT_NO_ARMS"})

            smach.StateMachine.add( 'WAIT_FOR_LOAD',
                                    states.Wait_time(robot, waittime=10),
                                    transitions={   'waited':'GO_BACK_TO_BREAKFAST_TABLE',
                                                    'preempted':'GO_BACK_TO_BREAKFAST_TABLE'})

            smach.StateMachine.add( 'RESET_ROBOT_NO_ARMS',
                                    ResetRobot(robot),
                                    transitions={   'done'              : 'SAY_NO_ARMS'})

            smach.StateMachine.add( 'SAY_NO_ARMS',
                                    states.Say(robot, ["I am terribly sorry but my arms hurt, can you please bring it yourself"]), 
                                    transitions={   'spoken'            : "failed"})

            smach.StateMachine.add( 'GO_BACK_TO_BREAKFAST_TABLE', 
                                    states.NavigateGeneric(robot, goal_name=BREAKFAST_1), 
                                    transitions={   "arrived"           : "SAY_HERES_BREAKFAST",
                                                    "unreachable"       : "failed",
                                                    "preempted"         : "failed",
                                                    "goal_not_defined"  : "failed"})

            smach.StateMachine.add( 'SAY_HERES_BREAKFAST',
                                    states.Say(robot, ["Here is your breakfast, please take it from my tray"]),
                                    transitions={   'spoken'            : 'RETURN_TRAY'})

            smach.StateMachine.add( 'RETURN_TRAY', 
                                    states.NavigateGeneric(robot, goal_pose_2d=KITCHEN_LOC), 
                                    transitions={   "arrived"           : "SAY_TAKE_TRAY",
                                                    "unreachable"       : "SAY_FAIL_RETURN_TRAY",
                                                    "preempted"         : "failed",
                                                    "goal_not_defined"  : "SAY_FAIL_RETURN_TRAY"})

            smach.StateMachine.add( 'SAY_FAIL_RETURN_TRAY',
                                    states.Say(robot, ["Dear cook, I have trouble reaching you, can you please help me"]),
                                    transitions={   'spoken'            : 'SAY_TAKE_TRAY'})

            smach.StateMachine.add( 'SAY_TAKE_TRAY',
                                    states.Say(robot, ["I have delivered the breakfast, please take the tray from my arms"]),
                                    transitions={   'spoken'            : 'RESET_ROBOT'})

            smach.StateMachine.add( 'RESET_ROBOT',
                                    ResetRobot(robot),
                                    transitions={   'done'              : 'succeeded'})


class Part2(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        with self:
            
            smach.StateMachine.add( 'GOTO_BREAKFAST_TABLE', 
                                    states.NavigateGeneric(robot, goal_name=BREAKFAST_2), 
                                    transitions={   "arrived"           : "INTERACTION",
                                                    "unreachable"       : "SAY_CANNOT_REACH_BREAKFAST_TABLE",
                                                    "preempted"         : "failed",
                                                    "goal_not_defined"  : "SAY_CANNOT_REACH_BREAKFAST_TABLE"})

            smach.StateMachine.add( 'INTERACTION',
                                    InteractionPart(robot),
                                    transitions={   'done'              : 'GOTO_KITCHEN'})

            smach.StateMachine.add( 'GOTO_KITCHEN', 
                                    states.NavigateGeneric(robot, goal_pose_2d=KITCHEN_LOC), 
                                    transitions={   "arrived"           : "REPORT_BREAKFAST",
                                                    "unreachable"       : "REPORT_BREAKFAST",
                                                    "preempted"         : "failed",
                                                    "goal_not_defined"  : "REPORT_BREAKFAST"})

            smach.StateMachine.add( 'SAY_CANNOT_REACH_BREAKFAST_TABLE',
                                    states.Say(robot, ["I better go to the cook straight away"]), 
                                    transitions={   'spoken'            : "GOTO_KITCHEN_BACKUP"})

            smach.StateMachine.add( 'GOTO_KITCHEN_BACKUP', 
                                    states.NavigateGeneric(robot, goal_pose_2d=KITCHEN_LOC), 
                                    transitions={   "arrived"           : "HOLDUP_ARM_FOR_CAN_LEFT",
                                                    "unreachable"       : "HOLDUP_ARM_FOR_CAN_LEFT",
                                                    "preempted"         : "failed",
                                                    "goal_not_defined"  : "HOLDUP_ARM_FOR_CAN_LEFT"})

            smach.StateMachine.add( 'SAY_KITCHEN_BACKUP',
                                    states.Say(robot, ["Can you please handover the can so i can take it to the patients"]), 
                                    transitions={   'spoken'            : "HOLDUP_ARM_FOR_CAN_LEFT"})

            def generate_report_sentence(*args,**kwargs):
                try:
                    breakfasts  = robot.reasoner.query(Compound("breakfast", "Breakfast"))
                    #patients    = robot.reasoner.query(Compound("current_patient", "Patient"))
                    breakfast   = breakfasts[0]["Breakfast"]
                    #patient     = patients[0]["Patient"]
                    return "Can you please give me a sandwich with {0} so I can serve that for breakfast".format(breakfast).replace('_', ' ')
                except:
                    return "I forgot what I should bring for breakfast"
            smach.StateMachine.add('REPORT_BREAKFAST',
                                    states.Say_generated(robot, sentence_creator=generate_report_sentence),
                                    transitions={ 'spoken':'HOLDUP_ARM_FOR_CAN_LEFT' })

            smach.StateMachine.add( 'HOLDUP_ARM_FOR_CAN_LEFT', 
                                    states.ArmToJointPos(robot, robot.leftArm, HOLD_CAN_POSE, timeout=4.0),
                                    transitions={   'done'              : "OPEN_LEFT_GRIPPER",
                                                    'failed'            : "HOLDUP_ARM_FOR_CAN_RIGHT"})

            smach.StateMachine.add( 'HOLDUP_ARM_FOR_CAN_RIGHT', 
                                    states.ArmToJointPos(robot, robot.leftArm, HOLD_CAN_POSE, timeout=4.0),
                                    transitions={   'done'              : "OPEN_LEFT_GRIPPER",
                                                    'failed'            : "OPEN_LEFT_GRIPPER"})

            smach.StateMachine.add('OPEN_LEFT_GRIPPER', 
                                    states.SetGripper(robot, robot.leftArm, gripperstate=ArmState.OPEN),
                                    transitions={   'succeeded'         : 'CLOSE_LEFT_GRIPPER',
                                                    'failed'            : 'CLOSE_LEFT_GRIPPER'})

            smach.StateMachine.add('CLOSE_LEFT_GRIPPER', 
                                    states.SetGripper(robot, robot.leftArm, gripperstate=ArmState.CLOSE),
                                    transitions={   'succeeded'         : 'GO_BACK_TO_BREAKFAST_TABLE',
                                                    'failed'            : 'GO_BACK_TO_BREAKFAST_TABLE'})

            smach.StateMachine.add( 'WAIT_FOR_LOAD',
                                    states.Wait_time(robot, waittime=10),
                                    transitions={   'waited':'GO_BACK_TO_BREAKFAST_TABLE',
                                                    'preempted':'GO_BACK_TO_BREAKFAST_TABLE'})

            smach.StateMachine.add( 'SAY_NO_ARMS',
                                    states.Say(robot, ["I am terribly sorry but my arms hurt, can you please bring it yourself"]), 
                                    transitions={   'spoken'            : "failed"})
            
            smach.StateMachine.add( 'GO_BACK_TO_BREAKFAST_TABLE', 
                                    states.NavigateGeneric(robot, goal_name=BREAKFAST_2), 
                                    transitions={   "arrived"           : "SAY_HERES_BREAKFAST",
                                                    "unreachable"       : "failed",
                                                    "preempted"         : "failed",
                                                    "goal_not_defined"  : "failed"})

            smach.StateMachine.add( 'SAY_HERES_BREAKFAST',
                                    states.Say(robot, ["Here is your breakfast"]),
                                    transitions={   'spoken'            : 'POOR_CHOCOLATE_NUTS'})

            smach.StateMachine.add( 'POOR_CHOCOLATE_NUTS',
                                    PoorChocolateNuts(robot),
                                    transitions={   'succeeded'         : 'SAY_DISPOSE_CAN',
                                                    'failed'            : 'SAY_CANT_POOR'})

            smach.StateMachine.add( 'SAY_CANT_POOR',
                                    states.Say(robot, ["I am terribly sorry but i cannot poor the chocolate nuts, can you please take them from my gripper"]),
                                    transitions={   'spoken'            : 'HAND_OVER_CHOCOLATE_NUTS'})

            smach.StateMachine.add( 'HAND_OVER_CHOCOLATE_NUTS',
                                    states.HandoverToHuman(robot.leftArm, robot),
                                    transitions={    'succeeded'         : 'SAY_DISPOSE_CAN',
                                                    'failed'            : 'SAY_DISPOSE_CAN'})

            smach.StateMachine.add( 'SAY_DISPOSE_CAN',
                                    states.Say(robot, ["I will throw the can right in the thrashbin"]),
                                    transitions={   'spoken'            : 'DISPOSE_CAN'})

            # ToDo: what if we carry with the right arm? Guess this needs to be asserted for the pooring motion anyway
            # ToDo: fix query?
            disposal_query = Compound("dropoff_point", "trashbin", Compound("point_3d", "X", "Y", "Z"))
            smach.StateMachine.add( 'DISPOSE_CAN',
                                    states.DropObject(robot.leftArm, robot, disposal_query),
                                    transitions={   "succeeded"         : "GO_TO_BREAKFAST_TABLE_FINAL",
                                                    "failed"            : "GO_TO_BREAKFAST_TABLE_FINAL",
                                                    "target_lost"       : "GO_TO_BREAKFAST_TABLE_FINAL"})

            smach.StateMachine.add( 'GO_TO_BREAKFAST_TABLE_FINAL', 
                                    states.NavigateGeneric(robot, goal_name=SINGPOS), 
                                    transitions={   "arrived"           : "ASK_SING_A_SONG",
                                                    "unreachable"       : "ASK_SING_A_SONG",
                                                    "preempted"         : "ASK_SING_A_SONG",
                                                    "goal_not_defined"  : "ASK_SING_A_SONG"})

            smach.StateMachine.add( 'ASK_SING_A_SONG',
                                    states.Say(robot, ["Do you want me to sing a song for you?"]),
                                    transitions={   'spoken'            : 'SING_A_SONG_YESNO'})

            smach.StateMachine.add("SING_A_SONG_YESNO",
                                    states.Ask_yes_no(robot),
                                    transitions={"yes":"SING_A_SONG", 
                                                 "no":"succeeded", 
                                                 "preempted":"succeeded"})

            smach.StateMachine.add( 'SING_A_SONG',
                                    SingSong(),
                                    transitions={   'done'              : "DROPOFF_OBJECT"})
            
            smach.StateMachine.add("DROPOFF_OBJECT",
                                    #PlaceObject(side, robot, placement_query, dropoff_height_offset=0.1):
                                    #states.Gripper_to_query_position(robot, robot.leftArm, query_dropoff_loc),
                                    states.DropObject(robot.leftArm, robot, query_dropoff_loc),
                                    transitions={   'succeeded':'succeeded',
                                                    'failed':'succeeded',
                                                    'target_lost':'succeeded'})


class DemoChallenge(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        robot.reasoner.query(Compound("retractall", Compound("at", "X", "Y")))
        robot.reasoner.query(Compound("retractall", Compound("current_patient", "X")))
        robot.reasoner.query(Compound("retractall", Compound("breakfast", "X")))

        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))
        #Assert the current challenge.
        robot.reasoner.assertz(Compound("challenge", "demo_challenge"))

        #query all at(Bed, Persons), to get where both patients are at. 
        #Set the closest person as the current_patient
        #Then Get the roi for that patient's bed an move there.
        #Then go to that current_patient's PATIENT, and retract that current_patient is at his bed, 
        #but assert that he's at his PATIENT.
        sleeping_patient_query = Conjunction(
                        Compound("at", "Patient", "Object"), 
                        Compound("type", "Object", "bed"))

        current_patient_query = Compound("current_patient", "Patient")

        patient_ROI = Conjunction(
                        current_patient_query,
                        Compound("at", "Patient", "Object"),
                        Compound("base_pose","Object", Compound("pose_2d", "X", "Y", "Phi")))

        patient_pickup = Conjunction(
                        current_patient_query,
                        Compound("at", "Patient", "Object"),
                        Compound("base_pose","Object", "pickup" ,Compound("pose_2d", "X", "Y", "Phi")))

        patient_destination_query = Conjunction(
                        current_patient_query,
                        Compound("owner","Object", "Patient"),
                        Compound("type","Object", "breakfasttable"),
                        Compound("base_pose","Object", Compound("pose_2d", "X", "Y", "Phi")))        

        patient_dropoff_query = Conjunction(
                        current_patient_query,
                        Compound("owner","Object", "Patient"),
                        Compound("type","Object", "breakfasttable"),
                        Compound("base_pose","Object", "dropoff", Compound("pose_2d", "X", "Y", "Phi")))

        @smach.cb_interface(outcomes=['done'])
        def look_down(userdata):
            robot.head.look_down()
            return "done"

        with self:
            smach.StateMachine.add( 'INITIALIZE', 
                                    states.Initialize(robot), 
                                    transitions={   'initialized':'INIT_POSE',
                                                    'abort':'Aborted'})

            smach.StateMachine.add('INIT_POSE',
                                states.Set_initial_pose(robot, "custom_initial"),
                                transitions={   'done':'SAY_START',
                                                'preempted':'SAY_START',
                                                'error':'SAY_START'})

            smach.StateMachine.add( 'SAY_START',
                                    states.Say(robot, ["Good morning everyone, lets see what i can do today"], block=False),
                                    transitions={   'spoken'            : 'PART1'})

            smach.StateMachine.add('PART1',
                                Part1(robot),
                                transitions={   'succeeded' : 'PART2',
                                                'failed'    : 'PART2'})

            smach.StateMachine.add('PART2',
                                Part2(robot),
                                transitions={   'succeeded' : 'RESET_ROBOT',
                                                'failed'    : 'RESET_ROBOT'})

            smach.StateMachine.add( 'RESET_ROBOT',
                                ResetRobot(robot),
                                transitions={   'done'      : 'SAY_DONE'})            

            smach.StateMachine.add( 'SAY_DONE',
                                    states.Say(robot, ["This was my demonstration, I'm done, thank you for your attention!"], block=False),
                                    transitions={   'spoken'            : 'Done'})

if __name__ == "__main__":
    rospy.init_node('demo_chalenge_exec')

    startup(DemoChallenge)
