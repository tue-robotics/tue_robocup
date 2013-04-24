#! /usr/bin/env python

#import ipdb; ipdb.set_trace() #import sys; [item for item in sys.path if "tue_execution_pack" in item]

import roslib; roslib.load_manifest('challenge_demo')
import rospy

import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from robot_smach_states.util.startup import startup #gives you speech-exception reading-out-loud and smach_viewer server

from psi import Conjunction, Compound

class GetBreakFastOptions(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:
            smach.StateMachine.add( 'DUMMY', 
                                    states.Say(robot, ["What is for breakfast?"]), 
                                    transitions={   'spoken':"Done"})

class EscortToBreakfast(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        with self:
            smach.StateMachine.add( 'DUMMY', 
                                    states.Say(robot, ["Please follow me, I'll bring you to the breakfast room"]), 
                                    transitions={   'spoken':"Done"})

class DemoChallenge(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

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

        patient_destination_query = Conjunction(
                        current_patient_query,
                        Compound("owner","Object", "Patient"),
                        Compound("base_pose","Object", Compound("pose_2d", "X", "Y", "Phi")))

        with self:
            smach.StateMachine.add( 'INITIALIZE', 
                                    states.Initialize(robot), 
                                    transitions={   'initialized':'GOTO_KITCHEN',
                                                    'abort':'Aborted'})

            smach.StateMachine.add( 'GOTO_KITCHEN', 
                                    states.NavigateGeneric(robot, goal_name="kitchen"), 
                                    transitions={   "arrived":"ASK_BREAKFAST_OPTIONS",
                                                    "unreachable":"Aborted",
                                                    "preempted":"Aborted",
                                                    "goal_not_defined":"Aborted",})

            smach.StateMachine.add( 'ASK_BREAKFAST_OPTIONS', 
                                    GetBreakFastOptions(robot), 
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
                    import ipdb; ipdb.set_trace()       
                    return 'patient_set'
            
            smach.StateMachine.add('DETERMINE_CURRENT_PATIENT', smach.CBState(determine_current_patient),
                                    transitions={   'patient_set':'GOTO_PATIENT',
                                                    'done':'EXIT'})

            smach.StateMachine.add( 'GOTO_PATIENT', 
                                    states.NavigateGeneric(robot, goal_query=patient_ROI),
                                     transitions={  "arrived":"WAKE_UP",
                                                    "unreachable":"DETERMINE_CURRENT_PATIENT",
                                                    "preempted":"SAY_DONE",
                                                    "goal_not_defined":"DETERMINE_CURRENT_PATIENT"})

            smach.StateMachine.add( 'WAKE_UP', 
                                    states.PlaySound(robot, "alarm.mp3"), 
                                    transitions={   'played':'SAY_GOODMORNING',
                                                    'error':'SAY_GOODMORNING'})

            smach.StateMachine.add( 'SAY_GOODMORNING', 
                                    states.Say(robot, ["Goodmorning sir. Hold on to me, I'll escort you to breakfast", "Wakey Wakey! I'll bring you to the canteen"]), 
                                    transitions={   'spoken':"ESCORT_TO_BREAKFAST"})

            # smach.StateMachine.add( 'ESCORT_TO_BREAKFAST', 
            #                         EscortToBreakfast(robot), 
            #                         transitions={   'Done':"ASK_WHAT_FOR_BREAKFAST",
            #                                         'Aborted':""})

            smach.StateMachine.add( 'ESCORT_TO_BREAKFAST', 
                                    states.NavigateGeneric(robot, goal_query=patient_destination_query),
                                     transitions={  "arrived":"ASK_WHAT_FOR_BREAKFAST",
                                                    "unreachable":"ASK_WHAT_FOR_BREAKFAST",
                                                    "preempted":"SAY_DONE",
                                                    "goal_not_defined":"ASK_WHAT_FOR_BREAKFAST"})


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
                    robot.reasoner.assertz(Compound("at", patient, "X"))        
                    return 'asserted'
                else:
                    rospy.loginfo("Something went terribly wrong! Exiting with done")
                    return "done"
            
            smach.StateMachine.add('ASSERT_NEW_PATIENT_POSITION', smach.CBState(assert_new_patient_pos),
                                    transitions={   'asserted':'GOTO_PATIENT',
                                                    'done':'EXIT'})

            smach.StateMachine.add( 'ASK_WHAT_FOR_BREAKFAST', 
                                    states.Timedout_QuestionMachine(
                                            robot=robot,
                                            default_option = "sandwich", 
                                            sentence = "What do you want for breakfast", 
                                            options = { "sandwich":Compound("breakfast", "sandwich"),
                                                        "eggs":Compound("breakfast", "eggs")}),
                                    transitions={   'answered':'GOTO_KITCHEN_2',
                                                    'not_answered':'GOTO_PATIENT'})

            smach.StateMachine.add( 'GOTO_KITCHEN_2', 
                                    states.NavigateGeneric(robot, goal_name="kitchen"), 
                                    transitions={   "arrived":"REPORT_BREAKFAST",
                                                    "unreachable":"Aborted",
                                                    "preempted":"Aborted",
                                                    "goal_not_defined":"Aborted"})

            def generate_report_sentence(*args,**kwargs):
                try:
                    breakfasts  = robot.reasoner.query(Compound("breakfast", "Breakfast"))
                    patients    = robot.reasoner.query(Compound("current_patient", "Patient"))
                    breakfast   = breakfasts[0]["Breakfast"]
                    patient     = patients[0]["Patient"]
                    return "{0} wants {1} for breakfast".format(patient, breakfast)
                except:
                    return "I forgot what whatsisname was for breakfast"
            smach.StateMachine.add('REPORT_BREAKFAST',
                                    states.Say_generated(robot, sentence_creator=generate_report_sentence),
                                    transitions={ 'spoken':'HOLDUP_ARMS' })

            smach.StateMachine.add( 'HOLDUP_ARMS', 
                                    states.ArmToJointPos(robot, robot.leftArm, [-0.1, -1.57, 0, 1.57, 0,0,0]), #Support pose
                                    transitions={   'done':"CARRY_TO_PATIENT",
                                                    'failed':"CARRY_TO_PATIENT"})

            smach.StateMachine.add( 'CARRY_TO_PATIENT', 
                                    states.NavigateGeneric(robot, goal_query=patient_ROI), 
                                    transitions={   "arrived":"HANDOVER_BREAKFAST",
                                                    "unreachable":"Aborted",
                                                    "preempted":"Aborted",
                                                    "goal_not_defined":"Aborted"})

            smach.StateMachine.add( 'HANDOVER_BREAKFAST', 
                                    states.Say(robot, ["Your breakfast, sir."]), 
                                    transitions={   'spoken':"GOTO_KITCHEN_3"})

            smach.StateMachine.add( 'GOTO_KITCHEN_3', 
                                    states.NavigateGeneric(robot), 
                                    transitions={   "arrived":"GOTO_KITCHEN_2",
                                                    "unreachable":"Aborted",
                                                    "preempted":"Aborted",
                                                    "goal_not_defined":"Aborted"})

            smach.StateMachine.add( 'EXIT', 
                                    states.NavigateGeneric(robot, goal_name="exit"), 
                                    transitions={   "arrived":"SAY_DONE",
                                                    "unreachable":"SAY_DONE",
                                                    "preempted":"SAY_DONE",
                                                    "goal_not_defined":"SAY_DONE"})

            smach.StateMachine.add( 'SAY_DONE', 
                                    states.Say(robot, ["I'm done here. It's always nice to help people"]), 
                                    transitions={   'spoken':"Done"})




if __name__ == "__main__":
    rospy.init_node('demo_chalenge_exec')

    startup(DemoChallenge)
