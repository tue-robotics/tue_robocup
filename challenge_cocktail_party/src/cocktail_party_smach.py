#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_cocktail_party')
import rospy

from amigo_skills.amigo import Amigo

from amigo_smach_states import *

#===============================TODOs===========================================
#
#===============================================================================

#================================ Bugs/Issues ==================================
#
#===============================================================================

#========================== Other ideas for executive improvement ==============
#
#===============================================================================

class WaitForPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["waiting" , "continue"])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.speech.speak("Ladies and gentlemen, please step in front of me to order your drinks.", language="us", personality="kyle", voice="default", mood="excited")

        query_detect_person = Conjunction(  Compound( "property_expected", "ObjectID", "class_label", "face"),
                                            Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo"))
                                          )

        self.robot.perception.toggle(["face_segmentation"])

        wait_machine = Wait_query_true(self.robot, query_detect_person, 10)
        wait_result = wait_machine.execute()

        self.robot.perception.toggle([])

        if wait_result == "timed_out":
            self.robot.speech.speak("Please, don't keep me waiting.", language="us", personality="kyle", voice="default", mood="excited")
            return "waiting"
        elif wait_result == "preempted":
            self.robot.speech.speak("Waiting for person was preemted... I don't even know what that means!", language="us", personality="kyle", voice="default", mood="excited")
            return "waiting"
        elif wait_result == "query_true":
            self.robot.speech.speak("Well hello there! Please let me have a look at you, such that I can remember you later.", language="us", personality="kyle", voice="default", mood="excited")
            return "continue"

class LookForDrink(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["looking" , "found", "not_found"])
        self.robot = robot

    def execute(self, userdata=None):
        # Move to the next waypoint in the storage room

        goal_answers = self.robot.reasoner.query(Conjunction(  Compound("=", "Waypoint", Compound("storage_room", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint"))))

        if not goal_answers:
            self.robot.speech.speak("I want to find the drink, but I don't know where to go... I'm sorry!", language="us", personality="kyle", voice="default", mood="excited")
            return "not_found"

        # for now, take the first goal found
        goal_answer = goal_answers[0]

        self.robot.speech.speak("I'm on the move, looking for your drink!", language="us", personality="kyle", voice="default", mood="excited")

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypoint_name = goal_answer["Waypoint"]

        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        if nav_result == "unreachable":                    
            return "continue"
        elif nav_result == "preempted":
            return "continue"

        # we made it to the new goal. Let's have a look to see whether we can find the object here
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        # look to ROI
        roi_answers = self.robot.reasoner.query(Compound("region_of_interest", waypoint_name, Compound("point_3d", "X", "Y", "Z")))
        if roi_answers:
            roi_answer = roi_answers[0]
            self.robot.head.send_goal(self.robot.head.point(float(roi_answer["X"]), float(roi_answer["Y"]), float(roi_answer["Z"])), "/map")

        self.robot.speech.speak("Let's see what I can find here", language="us", personality="kyle", voice="default", mood="excited")

        self.robot.perception.toggle(["template_matching"])
        rospy.sleep(5.0)
        self.robot.perception.toggle([])

        object_answers = self.robot.reasoner.query(Conjunction(  Compound("goal", Compound("serve", "Drink")),
                                           Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                           Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo"))))

        if object_answers:
            self.robot.speech.speak("Hey, I found the drink!", language="us", personality="kyle", voice="default", mood="excited")
            return "found"
        else:
            # have not found the drink, so let's keep looking
            return "looking"
 
class CocktailParty(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])

        robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
        robot.reasoner.query(Compound("retractall", Compound("goal", "X")))
        robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
        robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
        robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
        robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
        robot.reasoner.query(Compound("retractall", Compound("visited", "X")))

        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))

        robot.reasoner.assertz(Compound("challenge", "cocktailparty"))

        # Queries:
        query_party_room = Compound("waypoint", "party_room", Compound("pose_2d", "X", "Y", "Phi"))
        query_grabpoint = Conjunction(  Compound("goal", Compound("serve", "Drink")),
                                           Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                           Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                           Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))

        with self:
            
            smach.StateMachine.add( "START_CHALLENGE",
                                    StartChallenge(robot, "initial", query_party_room), 
                                    transitions={   "Done":"WAIT_FOR_PERSON", 
                                                    "Aborted":"Aborted", 
                                                    "Failed":"SAY_FAILED"})

            smach.StateMachine.add( "WAIT_FOR_PERSON", 
                                    WaitForPerson(robot),
                                    transitions={   "waiting":"WAIT_FOR_PERSON",
                                                    "continue":"LEARN_PERSON"})

            smach.StateMachine.add( "LEARN_PERSON",
                                    Learn_Person(robot),
                                    transitions={   "face_learned":"TAKE_ORDER", 
                                                    "learn_failed":"TAKE_ORDER"})

            smach.StateMachine.add('TAKE_ORDER', 
                                    Timedout_QuestionMachine(
                                            robot=robot,
                                            default_option = "coke", 
                                            sentence = "What would you like to drink?", 
                                            options = { "coke":Compound("goal", Compound("serve", "coke")),
                                                        "fanta":Compound("goal", Compound("serve", "fanta"))
                                                      }),
                                    transitions={   'answered':'LOOK_FOR_DRINK',
                                                    'not_answered':'TAKE_ORDER'})
               
            smach.StateMachine.add( 'LOOK_FOR_DRINK',
                                    LookForDrink(robot),
                                    transitions={   "looking":"LOOK_FOR_DRINK",
                                                    "found":'PICKUP_DRINK',
                                                    "not_found":'SAY_FAILED'})

            smach.StateMachine.add( 'PICKUP_DRINK',
                                    GrabMachine(robot.leftArm, robot, query_grabpoint),
                                    transitions={   "succeeded":"RETURN_DRINK",
                                                    "failed":'PICKUP_DRINK' })            

            smach.StateMachine.add( 'RETURN_DRINK',
                                    Navigate_to_queryoutcome(robot, query_party_room, X="X", Y="Y", Phi="Phi"),
                                    transitions={   "arrived":"FINISH",  # TODO
                                                    "unreachable":'Aborted',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'Aborted'})

            smach.StateMachine.add( 'FINISH', Finish(robot),
                                transitions={'stop':'Done'})

            smach.StateMachine.add("SAY_FAILED", 
                                    Say(robot, "I could not accomplish my task, sorry about that, please forgive me."),
                                    transitions={   "spoken":"Failed"})
 
if __name__ == '__main__':
    rospy.init_node('executive_cocktail_party')
 
    amigo = Amigo(wait_services=True)

    machine = CocktailParty(amigo)

    machine.execute()
