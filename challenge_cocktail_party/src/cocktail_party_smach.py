#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_cocktail_party')
import rospy

from robot_skills.amigo import Amigo

from robot_smach_states import *

names = ["john", "richard", "nancy", "alice", "bob"]
name_index = 0

#===============================TODOs===========================================
#
#===============================================================================

#================================ Bugs/Issues ==================================
#
#===============================================================================

#========================== Other ideas for executive improvement ==============
#
#===============================================================================

def speak(robot, sentence):
    #robot.speech.speak(sentence, language="us", personality="kyle", voice="default", mood="excited")
    print sentence

class WaitForPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["waiting" , "unknown_person", "known_person"])
        self.robot = robot

    def execute(self, userdata=None):
        speak(self.robot, "Ladies and gentlemen, please step in front of me to order your drinks.")

        query_detect_person = Conjunction(  Compound( "property_expected", "ObjectID", "class_label", "face"),
                                            Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo"))
                                          )

        self.robot.perception.toggle(["face_segmentation"])

        wait_machine = Wait_query_true(self.robot, query_detect_person, 10)
        wait_result = wait_machine.execute()

        self.robot.perception.toggle([])

        if wait_result == "timed_out":
            speak(self.robot, "Please, don't keep me waiting.")
            return "waiting"
        elif wait_result == "preempted":
            speak(self.robot, "Waiting for person was preemted... I don't even know what that means!")
            return "waiting"
        elif wait_result == "query_true":
            # check if we already know the person (the ID then already has a name in the world model)
            res = self.robot.reasoner.query(Compound("property_expected", "ObjectID", "name", "Name"))
            if not res:
                return "unknown_person"
            else:
                speak(self.robot, "Hello " + str(res[0]["Name"]) + "!")
                return "known_person"

class LearnPersonName(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["learned" , "failed"])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.reasoner.query(Compound("retractall", Compound("current_person", "X")))        

        q_state = Timedout_QuestionMachine( robot=self.robot,
                                            default_option = "john", 
                                            sentence = "Well hello there! I don't know you yet. Can you please tell me your name?", 
                                            options = { "john"   :Compound("current_person", "john"),
                                                        "richard":Compound("current_person", "richard"),
                                                        "alice"  :Compound("current_person", "alice")
                                                      })
        
        res = q_state.execute()
        if res == "answered":
            return_result = self.robot.reasoner.query(Compound("current_person", "Person"))        
            if not return_result:
                speak(self.robot, "That's horrible, I forgot who I should bring the drink to!")
                return "failed"

            serving_person = str(return_result[0]["Person"])

            speak(self.robot, "Hello " + serving_person + "!")
            return "learned"
        else:
            return "failed"


class LearnPersonCustom(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["face_learned" , "learn_failed"])
        self.robot = robot

    def execute(self, userdata=None):
        # find out who we need to return the drink to
        return_result = self.robot.reasoner.query(Compound("current_person", "Person"))        
        if not return_result:
            speak(self.robot, "That's horrible, I forgot who I should bring the drink to!")
            return "not_found"

        serving_person = str(return_result[0]["Person"])


        speak(self.robot, "Now " + serving_person + ", let me have a look at you, such that I can remember you later.")

        learn_machine = Learn_Person(self.robot, serving_person)
        learn_result = learn_machine.execute()
        
        self.robot.reasoner.query(Compound("retractall", Compound("goal", "X")))  # make sure we're not left with a goal from last time

        return learn_result

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
            speak(self.robot, "I want to find the drink, but I don't know where to go... I'm sorry!")
            return "not_found"

        # for now, take the first goal found
        goal_answer = goal_answers[0]

        speak(self.robot, "I'm on the move, looking for your drink!")

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

        speak(self.robot, "Let's see what I can find here")

        self.robot.perception.toggle(["template_matching"])
        rospy.sleep(5.0)
        self.robot.perception.toggle([])

        object_answers = self.robot.reasoner.query(Conjunction(  Compound("goal", Compound("serve", "Drink")),
                                           Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                           Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo"))))

        if object_answers:
            speak(self.robot, "Hey, I found the drink!")
            return "found"
        else:
            # have not found the drink, so let's keep looking
            return "looking"

class LookForPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["looking" , "found", "not_found"])
        self.robot = robot

    def execute(self, userdata=None):
        # find out who we need to return the drink to
        return_result = self.robot.reasoner.query(Compound("current_person", "Person"))        
        if not return_result:
            speak(self.robot, "That's horrible, I forgot who I should bring the drink to!")
            return "not_found"

        serving_person = str(return_result[0]["Person"])


        # Move to the next waypoint in the party room
        goal_answers = self.robot.reasoner.query(Conjunction(  Compound("=", "Waypoint", Compound("party_room", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint"))))

        if not goal_answers:
            speech.speak(self.robot, str(serving_person) +", I have been looking everywhere but I cannot find you. Can you please step in front of me?")
            return "not_found"

        # for now, take the first goal found
        goal_answer = goal_answers[0]

        speak(self.robot, str(serving_person) + ", I'm on my way!", language="us", personality="kyle", voice="default", mood="excited")

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypoint_name = goal_answer["Waypoint"]

        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        if nav_result == "unreachable":                    
            return "continue"
        elif nav_result == "preempted":
            return "continue"

        self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        # we made it to the new goal. Let's have a look to see whether we can find the person here
        speak(self.robot, "Let me see who I can find here...")

        self.robot.perception.toggle(["face_recognition", "face_segmentation"])
        rospy.sleep(5.0)
        self.robot.perception.toggle([])

        person_result = self.robot.reasoner.query(Conjunction(  Compound( "property_expected", "ObjectID", "class_label", "face"),
                                            Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                            Compound( "property", "ObjectID", "name", Compound("discrete", "DomainSize", "NamePMF"))
                                          ))

        if not person_result:
            speak(self.robot, "No one here. Moving on!")
            return "looking"

        # get the name PMF, which has the following structure: [p(0.4, exact(will)), p(0.3, exact(john)), ...]
        name_pmf = person_result[0]["NamePMF"]
        name=None
        name_prob=0
        for name_possibility in name_pmf:
            print name_possibility
            prob = float(name_possibility[0])
            if prob > 0.6 and prob > name_prob:
                name = str(name_possibility[1][0])
                name_prob = prob

        if not name:
            speak(self.robot, "Mmmmm, I don't know who you are. Moving on!")
            return "looking"        

        if name != serving_person:
            speak(self.robot, "Hello " + str(name) + "! You are not the one I should return this drink to. Moving on!")
            return "looking"

        if name:
            speak(self.robot, "Hello " + str(name))        
            return "found"

        return "not_found"
 
class CocktailParty(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])

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
                                                    "unknown_person":"LEARN_PERSON_NAME",
                                                    "known_person":"TAKE_ORDER"})

            smach.StateMachine.add( "LEARN_PERSON_NAME",
                                    LearnPersonName(robot),
                                    transitions={   "learned":"LEARN_PERSON_FACE",
                                                    "failed":"LEARN_PERSON_NAME"})
            
            smach.StateMachine.add( "LEARN_PERSON_FACE",
                                    LearnPersonCustom(robot),
                                    transitions={   "face_learned":"TAKE_ORDER",
                                                    "learn_failed":"LEARN_PERSON_FACE"})

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
                                    transitions={   "succeeded":"LOOK_FOR_PERSON",
                                                    "failed":'PICKUP_DRINK' })            

            smach.StateMachine.add( 'LOOK_FOR_PERSON',
                                    LookForPerson(robot),
                                    transitions={   "looking":"LOOK_FOR_PERSON",
                                                    "found":'FINISH',
                                                    "not_found":'SAY_FAILED'})

            smach.StateMachine.add( 'FINISH', Finish(robot),
                                transitions={'stop':'Done'})

            smach.StateMachine.add("SAY_FAILED", 
                                    Say(robot, "I could not accomplish my task, sorry about that, please forgive me."),
                                    transitions={   "spoken":"Failed"})
 
if __name__ == '__main__':
    rospy.init_node('executive_cocktail_party')
 
    amigo = Amigo(wait_services=True)

    amigo.reasoner.query(Compound("retractall", Compound("challenge", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("goal", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("explored", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
    amigo.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("current_object", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("current_person", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("visited", "X")))

    amigo.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))

    amigo.reasoner.assertz(Compound("challenge", "cocktailparty"))

    initial_state = None
    #initial_state= "LOOK_FOR_DRINK"

    if initial_state == "LOOK_FOR_DRINK":
        amigo.reasoner.query(Compound("assert", Compound("goal", Compound("serve", "coke"))))
        amigo.reasoner.query(Compound("assert", Compound("current_person", "john")))

    machine = CocktailParty(amigo)
    
    if initial_state != None:
        machine.set_initial_state([initial_state])

    machine.execute()
