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
import robot_smach_states as states

from robot_smach_states.util.startup import startup #gives you speech-exception reading-out-loud and smach_viewer server

from psi import Conjunction, Compound

from tf.transformations import euler_from_quaternion

HOLD_TRAY_POSE = [-0.1, 0.13, 0, 1.57, 0, 0.3, 0]
SUPPORT_PATIENT_POSE = [-0.1, -1.57, 0, 1.57, 0,0,0]
RESET_POSE = [-0.1, 0.13, 0, 0.3, 0, 0.3, 0]

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

class AskBreakfast(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done" , "failed"])
        self.robot = robot
        self.get_breakfast_question_service = rospy.ServiceProxy('interpreter/get_info_user', GetInfo)
        self.ask_breakfast_failed = 0
        self.person_breakfast = 0

    def execute(self, userdata=None):
        self.robot.reasoner.query(Compound("retractall", Compound("current_patient", "X")))        

        self.response = self.get_breakfast_question_service("demo_challenge", 3 , 60)  # This means that within 4 tries and within 60 seconds an answer is received. 

        if self.response.answer == "no_answer" or  self.response.answer == "wrong_answer":
            if self.ask_breakfast_failed == 1:
                self.robot.speech.speak("I will just give you a sandwich with peanutbutter")
                self.response.answer = "peanutbutter"
            if self.ask_breakfast_failed == 0:
                self.robot.speech.speak("I will just give you a sandwich with cheese")
                self.response.answer = "cheese"
                self.ask_breakfast_failed = 1
        else:
            self.robot.speech.speak("I will bring you a sandwich with " + self.response.answer)

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
                                    states.ArmToJointPos(robot, robot.leftArm, SUPPORT_PATIENT_POSE),
                                    transitions={'done':'SUPPORT_RIGHT',
                                                 'failed':'SUPPORT_RIGHT'})

            smach.StateMachine.add( 'SUPPORT_RIGHT',
                                    states.ArmToJointPos(robot, robot.rightArm, SUPPORT_PATIENT_POSE),
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

class DemoChallenge(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        robot.reasoner.query(Compound("retractall", Compound("at", "X", "Y")))
        robot.reasoner.query(Compound("retractall", Compound("current_patient", "X")))

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
                                states.Set_initial_pose(robot, "breakfast_1"),
                                transitions={   'done':'GOTO_KITCHEN',
                                                'preempted':'GOTO_KITCHEN',
                                                'error':'GOTO_KITCHEN'})

            smach.StateMachine.add( 'GOTO_KITCHEN', 
                                    states.NavigateGeneric(robot, goal_name="kitchen"), 
                                    transitions={   "arrived":"ASK_BREAKFAST_OPTIONS",
                                                    "unreachable":"Aborted",
                                                    "preempted":"Aborted",
                                                    "goal_not_defined":"Aborted",})

            smach.StateMachine.add( 'ASK_BREAKFAST_OPTIONS', 
                                    TalkToCook(robot), 
                                    transitions={   'Done':"DETERMINE_CURRENT_PATIENT"})

            @smach.cb_interface(outcomes=['patient_set', 'done'])
            def determine_current_patient(userdata):
                answers = robot.reasoner.query(sleeping_patient_query)
                if not answers:
                    rospy.loginfo("No answer for query {0}".format(sleeping_patient_query))
                    return 'done'
                else:         
                    patient = answers[0]["Patient"]
                    rospy.loginfo("Current patient = {0}".format(patient))
                    robot.reasoner.query(Compound("retractall", Compound("current_patient", "X")))
                    robot.reasoner.assertz(Compound("current_patient", patient))
                    return 'patient_set'
            
            smach.StateMachine.add('DETERMINE_CURRENT_PATIENT', smach.CBState(determine_current_patient),
                                    transitions={   'patient_set':'GOTO_PATIENT',
                                                    'done':'EXIT'})

            smach.StateMachine.add( 'GOTO_PATIENT', 
                                    states.NavigateGeneric(robot, goal_query=patient_ROI),
                                     transitions={  "arrived":"LOOK_TO_PATIENT",
                                                    "unreachable":"DETERMINE_CURRENT_PATIENT",
                                                    "preempted":"SAY_DONE",
                                                    "goal_not_defined":"DETERMINE_CURRENT_PATIENT"})

            smach.StateMachine.add('LOOK_TO_PATIENT', smach.CBState(look_down),
                                    transitions={   'done':'WAKE_UP'})

            smach.StateMachine.add( 'WAKE_UP', 
                                    states.PlaySound("/home/amigo/Music/Toeter1.mp3", blocking=True), 
                                    transitions={   'played':'SAY_GOODMORNING',
                                                    'error':'SAY_GOODMORNING'})

            def generate_time_sentence(*args, **kwargs):
                import time, random
                timestr = time.strftime( "it's %M past %H", time.localtime(time.time()))
                sentences = [   "Goodmorning sir, its {0} already, time to wake up. Hold on to me, I'll escort you to breakfast".format(timestr), 
                                "Wakey Wakey! Its {0}, time to wake up. I'll bring you to the canteen".format(timestr)]
                return random.choice(sentences)
            smach.StateMachine.add( 'SAY_GOODMORNING', 
                                    states.Say_generated(robot, generate_time_sentence),#["Goodmorning sir. Hold on to me, I'll escort you to breakfast", "Wakey Wakey! I'll bring you to the canteen"]), 
                                    transitions={   'spoken':"TURN_AROUND"})

            smach.StateMachine.add( "TURN_AROUND",
                                    TurnAround(robot, 3.14),
                                    transitions={'done':'ESCORT_TO_BREAKFAST',
                                                 'failed':'ESCORT_TO_BREAKFAST'})

            smach.StateMachine.add( 'ESCORT_TO_BREAKFAST', 
                                    EscortToBreakfast(robot, patient_destination_query), 
                                    transitions={   'Done':"ASSERT_NEW_PATIENT_POSITION",
                                                    'Aborted':""})

            @smach.cb_interface(outcomes=['asserted', 'done'])
            def assert_new_patient_pos(userdata):
                patient_answers = robot.reasoner.query(current_patient_query)
                destination_answers = robot.reasoner.query(patient_destination_query)
                #import ipdb; ipdb.set_trace()
                if not patient_answers or not destination_answers:
                    # no more exporation targets found
                    return 'done'
                elif patient_answers and destination_answers:         
                    patient = patient_answers[0]["Patient"]
                    destination = destination_answers[0]["Object"]

                    robot.reasoner.query(Compound("retractall", Compound("at", patient, "X"))) #The patient 
                    robot.reasoner.assertz(Compound("at", patient, destination))        
                    return 'asserted'
                else:
                    rospy.loginfo("Something went terribly wrong! Exiting with done")
                    return "done"
            
            smach.StateMachine.add('ASSERT_NEW_PATIENT_POSITION', smach.CBState(assert_new_patient_pos),
                                    transitions={   'asserted':'SAY_SIT_DOWN',
                                                    'done':'SAY_SIT_DOWN'})

            smach.StateMachine.add( 'SAY_SIT_DOWN', 
                                    states.Say(robot, ["Please take a seat", "Please sit down"]), 
                                    transitions={   'spoken':"LOOK_TO_BREAKFAST"})

            @smach.cb_interface(outcomes=['done'])
            def look_down(userdata):
                robot.head.look_down()
                return "done"
            smach.StateMachine.add('LOOK_TO_BREAKFAST', smach.CBState(look_down),
                                    transitions={   'done':'RECITE_BREAKFAST_OPTIONS'})

            smach.StateMachine.add( "RECITE_BREAKFAST_OPTIONS",
                                    states.Say(robot, "On today's breakfast menu, we have sandwiches with jam, salami, cheece, peanut butter or chocolade. Which do you want?"),
                                    transitions={'spoken':"ASK_WHAT_FOR_BREAKFAST"})

            smach.StateMachine.add( 'ASK_WHAT_FOR_BREAKFAST', 
                                    AskBreakfast(robot),
                                    transitions={   'done':'GOTO_KITCHEN_TO_REPORT_ORDER',
                                                    'failed':'GOTO_PATIENT'})

            smach.StateMachine.add( 'GOTO_KITCHEN_TO_REPORT_ORDER', 
                                    states.NavigateGeneric(robot, goal_name="kitchen"), 
                                    transitions={   "arrived":"HOLDUP_ARMS_FOR_TRAY_LEFT",
                                                    "unreachable":"Aborted",
                                                    "preempted":"Aborted",
                                                    "goal_not_defined":"Aborted"})
            
            smach.StateMachine.add( 'HOLDUP_ARMS_FOR_TRAY_LEFT', 
                                    states.ArmToJointPos(robot, robot.leftArm, HOLD_TRAY_POSE),
                                    transitions={   'done':"HOLDUP_ARMS_FOR_TRAY_RIGHT",
                                                    'failed':"HOLDUP_ARMS_FOR_TRAY_RIGHT"})
            smach.StateMachine.add( 'HOLDUP_ARMS_FOR_TRAY_RIGHT', 
                                    states.ArmToJointPos(robot, robot.rightArm, HOLD_TRAY_POSE),
                                    transitions={   'done':"REPORT_BREAKFAST",
                                                    'failed':"REPORT_BREAKFAST"})

            def generate_report_sentence(*args,**kwargs):
                try:
                    breakfasts  = robot.reasoner.query(Compound("breakfast", "Breakfast"))
                    patients    = robot.reasoner.query(Compound("current_patient", "Patient"))
                    breakfast   = breakfasts[0]["Breakfast"]
                    patient     = patients[0]["Patient"]
                    return "{0} wants {1} for breakfast. If you give it to me, i'll bring it to {0}".format(patient, breakfast)
                except:
                    return "I forgot what whatsisname again wants for breakfast"
            smach.StateMachine.add('REPORT_BREAKFAST',
                                    states.Say_generated(robot, sentence_creator=generate_report_sentence),
                                    transitions={ 'spoken':'CARRY_TO_PATIENT' })

            smach.StateMachine.add( 'CARRY_TO_PATIENT', 
                                    states.NavigateGeneric(robot, goal_query=patient_destination_query), 
                                    transitions={   "arrived":"HANDOVER_BREAKFAST",
                                                    "unreachable":"Aborted",
                                                    "preempted":"Aborted",
                                                    "goal_not_defined":"Aborted"})

            smach.StateMachine.add( 'HANDOVER_BREAKFAST', 
                                    states.Say(robot, [ "Your breakfast, sir. Remember to take your pills after eating", 
                                                        "Enjoy your breakfast, sir. But, after you are done, take the pills the doctor supscribed!"]), 
                                    transitions={   'spoken':"ASSERT_CURRENT_PATIENT_DONE"})

            @smach.cb_interface(outcomes=['asserted'])
            def retract_current_patient(userdata):
                patient_answers = robot.reasoner.query(current_patient_query)
                #import ipdb; ipdb.set_trace()
                if patient_answers:       
                    patient = patient_answers[0]["Patient"]

                    robot.reasoner.query(Compound("retractall", Compound("current_patient", patient, "X"))) #The patient 
                    #robot.reasoner.assertz(Compound("served", patient))        
                    return 'asserted'
            
            smach.StateMachine.add('ASSERT_CURRENT_PATIENT_DONE', smach.CBState(retract_current_patient),
                                    transitions={   'asserted':'RESET_ARM_LEFT'})

            smach.StateMachine.add( 'RESET_ARM_LEFT', 
                                    states.ArmToJointPos(robot, robot.leftArm, RESET_POSE),
                                    transitions={   'done':"RESET_ARM_RIGHT",
                                                    'failed':"RESET_ARM_RIGHT"})
            smach.StateMachine.add( 'RESET_ARM_RIGHT', 
                                    states.ArmToJointPos(robot, robot.rightArm, RESET_POSE),
                                    transitions={   'done':"DETERMINE_CURRENT_PATIENT",
                                                    'failed':"DETERMINE_CURRENT_PATIENT"})

            smach.StateMachine.add( 'EXIT', 
                                    states.NavigateGeneric(robot, goal_name="exit"), 
                                    transitions={   "arrived":"SAY_DONE",
                                                    "unreachable":"SAY_DONE",
                                                    "preempted":"SAY_DONE",
                                                    "goal_not_defined":"SAY_DONE"})

            smach.StateMachine.add( 'SAY_DONE', 
                                    states.Say(robot, ["I'm done here. It's always nice to help people", "My work here is done."]), 
                                    transitions={   'spoken':"Done"})

if __name__ == "__main__":
    rospy.init_node('demo_chalenge_exec')

    startup(DemoChallenge)
