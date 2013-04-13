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
                                    states.Say(robot, ["Plase follow me, I'll bring you to the breakfast room"]), 
                                    transitions={   'spoken':"Done"})

class DemoChallenge(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))
        #Assert the current challenge.
        robot.reasoner.assertz(Compound("challenge", "demo"))

        bed_query =     Conjunction(
                        Compound("type","Object", "bed"),
                        Compound("position","Object", Compound("point_3d", "X", "Y", "Z")))        

        patient_query = Conjunction(
                        Compound("type","Object", "person"),
                        Compound("position","Object", Compound("point_3d", "X", "Y", "Z")))

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
                                    transitions={   'Done':"GOTO_BED_2"})

            smach.StateMachine.add( 'GOTO_BED_2', 
                                    states.Visit_query_outcome_3d(robot, query=bed_query),
                                     transitions={  "arrived":"WAKE_UP",
                                                    "unreachable":"Aborted",
                                                    "preempted":"Aborted",
                                                    "goal_not_defined":"Aborted",
                                                    "all_matches_tried":"EXIT"})

            smach.StateMachine.add( 'WAKE_UP', 
                                    states.PlaySound(robot, "alarm.mp3"), 
                                    transitions={   'played':'SAY_GOODMORNING',
                                                    'error':'SAY_GOODMORNING'})

            smach.StateMachine.add( 'SAY_GOODMORNING', 
                                    states.Say(robot, ["Goodmorning sir", "Wakey Wakey!"]), 
                                    transitions={   'spoken':"ESCORT_TO_BREAKFAST"})

            smach.StateMachine.add( 'ESCORT_TO_BREAKFAST', 
                                    EscortToBreakfast(robot), 
                                    transitions={   'Done':"ASK_WHAT_FOR_BREAKFAST",
                                                    'Aborted':""})

            smach.StateMachine.add( 'ASK_WHAT_FOR_BREAKFAST', 
                                    states.Timedout_QuestionMachine(
                                            robot=robot,
                                            default_option = "sandwich", 
                                            sentence = "What do you want for breakfast", 
                                            options = { "sandwich":Compound("breakfast", "sandwich"),
                                                        "eggs":Compound("breakfast", "eggs")}),
                                    transitions={   'answered':'GOTO_KITCHEN_2',
                                                    'not_answered':'GOTO_BED_2'})

            smach.StateMachine.add( 'GOTO_KITCHEN_2', 
                                    states.NavigateGeneric(robot, goal_name="kitchen"), 
                                    transitions={   "arrived":"REPORT_BREAKFAST",
                                                    "unreachable":"Aborted",
                                                    "preempted":"Aborted",
                                                    "goal_not_defined":"Aborted"})


            smach.StateMachine.add( 'REPORT_BREAKFAST', 
                                    states.Say(robot), 
                                    transitions={   'spoken':"HOLDUP_ARMS"})

            smach.StateMachine.add( 'HOLDUP_ARMS', 
                                    states.ArmToJointPos(robot, robot.leftArm, [0,0,0,0,0,0,0]), #TODO pose
                                    transitions={   'done':"GOTO_PATIENT_2",
                                                    'failed':"GOTO_PATIENT_2"})

            smach.StateMachine.add( 'GOTO_PATIENT_2', 
                                    states.NavigateGeneric(robot, goal_query=patient_query), 
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
                                    states.NavigateGeneric(robot), 
                                    transitions={   "arrived":"REPORT_BREAKFAST",
                                                    "unreachable":"Aborted",
                                                    "preempted":"Aborted",
                                                    "goal_not_defined":"Aborted"})




if __name__ == "__main__":
    rospy.init_node('demo_chalenge_exec')

    startup(DemoChallenge)
