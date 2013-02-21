#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_cocktail_party')
import rospy

from tue_execution_pack import states, smach, util, robot_parts

from robot_parts.reasoner import *

#===============================TODOs===========================================
#
#===============================================================================

#================================ Bugs/Issues ==================================
#
#===============================================================================

#========================== Other ideas for executive improvement ==============
#
#===============================================================================
 
class CocktailParty(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])

        robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
        robot.reasoner.query(Compound("retractall", Compound("goal", "X")))
        robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
        robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
        robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
        robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))

        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))

        robot.reasoner.assertz(Compound("challenge", "cocktailparty"))

        with self:
            query_room = Compound("waypoint", "party_room", Compound("pose_2d", "X", "Y", "Phi"))
            smach.StateMachine.add( "START_CHALLENGE",
                                    states.StartChallenge(robot, query_room), 
                                    transitions={   "Done":"CALL_PERSON", 
                                                    "Aborted":"Aborted", 
                                                    "Failed":"SAY_FAILED"})

            smach.StateMachine.add( "CALL_PERSON", 
                                    states.Say(robot, "Ladies and gentlemen, please step in front of me to order your drinks."),
                                    transitions={   "spoken":"LEARN_PERSON"})


            smach.StateMachine.add( "LEARN_PERSON",
                                    states.Learn_Person(robot),
                                    transitions={   "person_learned":"ASK_DRINK", 
                                                    "learning_failed":"ASK_DRINK"})

            smach.StateMachine.add('ASK_DRINK', 
                                    states.Timedout_QuestionMachine(
                                            robot=robot,
                                            default_option = "coke", 
                                            sentence = "What would you like to drink?", 
                                            options = { "coke":Compound("goal", Compound("serve", "coke")),
                                                        "fanta":Compound("goal", Compound("serve", "fanta"))
                                                      }),
                                    transitions={   'answered':'DRIVE_TO_STORAGE',
                                                    'not_answered':'ASK_DRINK'})

            query_room = Compound("waypoint", "storage_room", Compound("pose_2d", "X", "Y", "Phi"))   
            smach.StateMachine.add( 'DRIVE_TO_STORAGE',
                                    states.Navigate_to_queryoutcome(robot, query_room, X="X", Y="Y", Phi="Phi"),
                                    transitions={   "arrived":"RETURN_DRINK",  # TODO
                                                    "unreachable":'Aborted',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'Aborted'})

            query_room = Compound("waypoint", "party_room", Compound("pose_2d", "X", "Y", "Phi"))   
            smach.StateMachine.add( 'RETURN_DRINK',
                                    states.Navigate_to_queryoutcome(robot, query_room, X="X", Y="Y", Phi="Phi"),
                                    transitions={   "arrived":"FINISH",  # TODO
                                                    "unreachable":'Aborted',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'Aborted'})

            smach.StateMachine.add( 'FINISH', states.Finish(robot),
                                transitions={'stop':'Done'})

            smach.StateMachine.add("SAY_FAILED", 
                                    states.Say(robot, "I could not accomplish my task, sorry about that, please forgive me."),
                                    transitions={   "spoken":"Failed"})
 
if __name__ == '__main__':
    rospy.init_node('executioner')
 
    util.startup(CocktailParty)

    #print Compound("bla")