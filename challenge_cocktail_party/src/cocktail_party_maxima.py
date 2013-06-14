#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_cocktail_party')
import rospy

import smach_ros

from speech_interpreter.srv import GetInfo

from robot_skills.amigo import Amigo

from robot_smach_states import *

names = ["john", "richard", "nancy", "alice", "bob"]
name_index = 0

grasp_arm = "left"
#grasp_arm = "right"

#===============================TODOs===========================================
# - head goal and base goal must correspond
#===============================================================================

#================================ Bugs/Issues ==================================
#
#===============================================================================

#========================== Other ideas for executive improvement ==============
#
#===============================================================================
class WaitForPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["waiting", "unknown_person", "known_person"])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.speech.speak("Ladies and gentlemen, please step in front of me to order your drink.")

        query_detect_person = Conjunction(Compound("property_expected", "ObjectID", "class_label", "face"),
                                          Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")))
        self.robot.head.set_pan_tilt(tilt=-0.2)
        self.response_start = self.robot.perception.toggle(['face_segmentation'])
        rospy.loginfo("error_code = {0}".format(self.response_start.error_code))
        rospy.loginfo("error_msg = {0}".format(self.response_start.error_msg))

        if self.response_start.error_code == 0:
            rospy.loginfo("Face segmentation has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Face segmentation failed to start")
            self.robot.speech.speak("I was not able to start face segmentation.")
            return "waiting"
        rospy.sleep(3)

        wait_machine = Wait_query_true(self.robot, query_detect_person, 10)
        wait_result = wait_machine.execute()

        self.robot.perception.toggle([])

        if wait_result == "timed_out":
            self.robot.speech.speak("Please, don't keep me waiting.")
            return "waiting"
        elif wait_result == "preempted":
            self.robot.speech.speak("Waiting for person was preemted... I don't even know what that means!")
            return "waiting"
        elif wait_result == "query_true":
            # check if we already know the person (the ID then already has a name in the world model)
            # PERSON UNKNOWN IS ASSERTED
            res = self.robot.reasoner.query(Compound("property_expected", "ObjectID", "name", "Name"))
            if not res:
                return "unknown_person"
            else:
                self.robot.speech.speak("Hello " + str(res[0]["Name"]) + "!")
                return "known_person"

# class LearnPersonName(smach.State):
#     def __init__(self, robot):
#         smach.State.__init__(self, outcomes=["learned" , "failed"])
#         self.robot = robot

#     def execute(self, userdata=None):
#         self.robot.reasoner.query(Compound("retractall", Compound("current_person", "X")))        

#         q_state = Timedout_QuestionMachine( robot=self.robot,
#                                             default_option = "john", 
#                                             sentence = "Well hello there! I don't know you yet. Can you please tell me your name?", 
#                                             options = { "john"   :Compound("current_person", "john"),
#                                                         "richard":Compound("current_person", "richard"),
#                                                         "alice"  :Compound("current_person", "alice")
#                                                       })
        
#         res = q_state.execute()
#         if res == "answered":
#             return_result = self.robot.reasoner.query(Compound("current_person", "Person"))        
#             if not return_result:
#                 self.robot.speech.speak("That's horrible, I forgot who I should bring the drink to!")
#                 return "failed"

#             serving_person = str(return_result[0]["Person"])

#             self.robot.speech.speak("Hello " + serving_person + "!")
#             return "learned"
#         else:
#             return "failed"

class LearnPersonName(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["learned" , "failed"])
        self.robot = robot
        self.get_learn_person_name_service = rospy.ServiceProxy('interpreter/get_info_user', GetInfo)
        self.person_learn_failed = 0

    def execute(self, userdata=None):
        self.robot.reasoner.query(Compound("retractall", Compound("current_person", "X")))    

        self.response = self.get_learn_person_name_service("name_maxima", 3 , 60)  # This means that within 4 tries and within 60 seconds an answer is received.

        if self.response.answer == "no_answer" or self.response.answer == "wrong_answer":
            if self.person_learn_failed == 2:
                self.robot.speech.speak("I will call you William")
                self.response.answer = "william"
                self.person_learn_failed = 3
            if self.person_learn_failed == 1:
                self.robot.speech.speak("I will call you Michael")
                self.response.answer = "michael"
                self.person_learn_failed = 2
            if self.person_learn_failed == 0:
                self.robot.speech.speak("I will call you Maxima")
                self.response.answer = "Maxima"
                self.person_learn_failed = 1

        self.robot.reasoner.query(Compound("assert", Compound("current_person", self.response.answer)))
            

        return_result = self.robot.reasoner.query(Compound("current_person", "Person"))        
        if not return_result:
            self.robot.speech.speak("That's horrible, did not get a name!")
            return "failed"

        serving_person = str(return_result[0]["Person"])

        self.robot.speech.speak("Hello " + serving_person + "!")
        return "learned"


class Ask_drink(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done" , "failed"])
        self.robot = robot
        self.get_drink_service = rospy.ServiceProxy('interpreter/get_info_user', GetInfo)
        self.person_learn_failed = 0

    def execute(self, userdata=None):
        self.response = self.get_drink_service("drink", 3 , 60)  # This means that within 4 tries and within 60 seconds an answer is received. 

        # Check available options from rulebook!
        if self.response.answer == "no_answer" or  self.response.answer == "wrong_answer":
            self.robot.speech.speak("I will just bring you a coke")
            self.response.answer = "coke"
        
        rospy.loginfo("self.response = {0}".format(self.response.answer))
        #import ipdb; ipdb.set_trace()
        self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("serve", self.response.answer))))

        return "done"

class LearnPersonCustom(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["face_learned" , "learn_failed"])
        self.robot = robot

    def execute(self, userdata=None):
        # find out who we need to return the drink to
        return_result = self.robot.reasoner.query(Compound("current_person", "Person"))        
        if not return_result:
            self.robot.speech.speak("That's horrible, I forgot who I should bring the drink to!")
            return "not_found"

        serving_person = str(return_result[0]["Person"])


        self.robot.speech.speak("Now " + serving_person + ", let me have a look at you, such that I can remember you later.")

        learn_machine = Learn_Person(self.robot, serving_person)
        learn_result = learn_machine.execute()
        self.robot.reasoner.query(Compound("retractall", Compound("goal", "X")))  # make sure we're not left with a goal from last time // what does this do?!
        ## TO CHECK IF OUTCOME IS face_learned or learn_failed and ACT adequatly!
        return learn_result

class LookForDrink(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["looking" , "found", "not_found"])
        self.robot = robot

    def execute(self, userdata=None):
        # Move to the next waypoint in the storage room
        # ToDo Location are drink specific, only look at one location or return TEUN
        # At this point reasonar is queried to find the "Drink", has to be associated in knowledge, but current Waypoint determination will be enough probably
        return_result = self.robot.reasoner.query(Compound("goal", Compound("serve", "Drink")))
        serving_drink = str(return_result[0]["Drink"])       
        if not return_result:
            self.robot.speech.speak("I forgot which drink you wanted")


        goal_answers = self.robot.reasoner.query(Conjunction(  Compound("=", "Waypoint", Compound("storage_room", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint"))))

        if not goal_answers:
            self.robot.speech.speak("I want to find the drink, but I don't know where to go... I'm sorry!")
            return "not_found"

        self.robot.speech.speak("I think I know the location of your " + serving_drink)
        # for now, take the first goal found
        goal_answer = goal_answers[0]

        self.robot.speech.speak("I'm on the move, looking for your drink!")

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypoint_name = goal_answer["Waypoint"]

        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        # we tried to make it to the new goal. Let's have a look to see whether we can find the object here
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        ## If nav_result is unreachable DO NOT stop looking, there are more options, return not_found when list of Waypoints is empty
        if nav_result == "unreachable":                    
            return "looking"
        elif nav_result == "preempted":
            return "looking"

        # we made it to the new goal. Let's have a look to see whether we can find the object here
        #self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        # look to ROI
        roi_answers = self.robot.reasoner.query(Compound("point_of_interest", waypoint_name, Compound("point_3d", "X", "Y", "Z")))
        if roi_answers:
            roi_answer = roi_answers[0]
            self.robot.head.send_goal(self.robot.head.point(float(roi_answer["X"]), float(roi_answer["Y"]), float(roi_answer["Z"])), "/map")

        self.robot.speech.speak("Let's see what I can find here")

        self.robot.perception.toggle(["template_matching"])
        rospy.sleep(5.0)
        self.robot.perception.toggle([])

        object_answers = self.robot.reasoner.query(Conjunction(  Compound("goal", Compound("serve", "Drink")),
                                           Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                           Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo"))))

        if object_answers:
            self.robot.speech.speak("Hey, I found the drink!")
            return "found"
        else:
            self.robot.speech.speak("Did not find the drink!")
            #self.robot.speech.speak("Did not find anything, keep looking")
            return "looking"

class LookForPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["looking" , "found", "not_found"])
        self.robot = robot

    def execute(self, userdata=None):
        # find out who we need to return the drink to
        return_result = self.robot.reasoner.query(Compound("current_person", "Person"))        
        if not return_result:
            self.robot.speech.speak("That's horrible, I forgot who I should bring the drink to!")
            return_drink_result = self.robot.reasoner.query(Compound("goal", Compound("serve", "Drink")))
            serving_drink = str(return_drink_result[0]["Drink"])  
            self.robot.speech.speak("I'm coming to the meeting point to hand over the " + str(serving_drink))
            return "not_found"

        serving_person = str(return_result[0]["Person"])


        # Move to the next waypoint in the party room
        goal_answers = self.robot.reasoner.query(Conjunction(  
                                                    Compound("=", "Waypoint", Compound("party_room", "W")),
                                                    Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                    Compound("not", Compound("visited", "Waypoint"))
                                                            ))

        if not goal_answers:
            self.robot.speech.speak(str(serving_person) +", I have been looking everywhere but I cannot find you.")
            return "not_found"

        # for now, take the first goal found
        goal_answer = goal_answers[0]

        self.robot.speech.speak(str(serving_person) + ", I'm on my way!", language="us", personality="kyle", voice="default", mood="excited")

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypoint_name = goal_answer["Waypoint"]

        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        ## If nav_result is unreachable DO NOT stop looking, there are more options, return not_found when list of Waypoints is empty
        if nav_result == "unreachable":                    
            return "looking"
        elif nav_result == "preempted":
            return "looking"

        self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        # we made it to the new goal. Let's have a look to see whether we can find the person here
        self.robot.speech.speak("Let me see who I can find here...")
        self.robot.head.set_pan_tilt(tilt=-0.2)
        self.robot.spindle.reset()

        self.response_start = self.robot.perception.toggle(["face_segmentation"])
        rospy.loginfo("error_code = {0}".format(self.response_start.error_code))
        rospy.loginfo("error_msg = {0}".format(self.response_start.error_msg))
        if self.response_start.error_code == 0:
            rospy.loginfo("Face segmentation has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Face segmentation failed to start")
            self.robot.speech.speak("I was not able to start face segmentation.")
        rospy.sleep(5.0)
        self.robot.perception.toggle([])

        person_result = self.robot.reasoner.query(
                                            Conjunction(  
                                                Compound( "property_expected", "ObjectID", "class_label", "face"),
                                                Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo"))))
        if not person_result:
            self.robot.speech.speak("No one here. Face segmentation did not find a person here")
            return "not_found"
        self.robot.speech.speak("Hi there, human. Please look into my eyes, so I can recognize you.")


        self.robot.perception.toggle(["face_recognition", "face_segmentation"])
        rospy.sleep(5.0)
        self.robot.perception.toggle([])

        person_result = self.robot.reasoner.query(
                                            Conjunction(  
                                                Compound( "property_expected", "ObjectID", "class_label", "face"),
                                                Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                                Compound( "property", "ObjectID", "name", Compound("discrete", "DomainSize", "NamePMF"))))

        if len(person_result) > 0:
            name_pmf = person_result[0]["NamePMF"]

            if len(person_result) > 1:
                rospy.logwarn("Multiple faces detected, only checking the first one!")

            name=None
            name_prob=0
            for name_possibility in name_pmf:
                print name_possibility
                prob = float(name_possibility[0])
                if prob > 0.1 and prob > name_prob:
                    name = str(name_possibility[1][0])
                    #print "Updated most probable name to " + str(name)
                    name_prob = prob

            if not name:
                self.robot.speech.speak("I don't know who you are.")
                return "looking"  

            if name != serving_person:
                self.robot.speech.speak("Hello " + str(name) + "! You are not the one I should return this drink to. Moving on!")
                return "looking"      

            if name:
                self.robot.speech.speak("Hello " + str(name)) 
                return "found"       
        else:
            rospy.logwarn("No person names received from world model") 

        return "not_found"

## Class not build in yet, this will be used when person can not be found but drink is still in gripper
class HandoverToKnownHuman(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["done"])

        if grasp_arm == "left":
            arm = robot.leftArm
        if grasp_arm == "right":
            arm = robot.rightArm

        with self:
            smach.StateMachine.add( 'PRESENT_DRINK',
                                    Say(robot, ["Here's your drink", "Here you go!"]),
                                    transitions={"spoken":"POSE"})

            smach.StateMachine.add( 'POSE',
                                    Handover_pose(arm, robot),
                                    transitions={   'succeeded':'PLEASE_TAKE',
                                                    'failed':'PLEASE_TAKE'})
            
            smach.StateMachine.add( 'PLEASE_TAKE',
                                    Say(robot, ["Please hold the drink, I'm going to let it go.", "Please take the drink, i'll let it go"]),
                                    transitions={"spoken":"OPEN_GRIPPER"})

            smach.StateMachine.add( "OPEN_GRIPPER", 
                                    SetGripper(robot, arm, gripperstate=0, drop_from_frame="/grippoint_left"), #open
                                    transitions={   'succeeded':'CLOSE_AFTER_DROP',
                                                    'failed':'CLOSE_AFTER_DROP'})
            smach.StateMachine.add( 'CLOSE_AFTER_DROP',
                                    SetGripper(robot, arm, gripperstate=1), #close
                                    transitions={   'succeeded':'RESET_ARM',
                                                    'failed':'RESET_ARM'})
            smach.StateMachine.add('RESET_ARM', 
                                    ArmToPose(robot, arm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), 
                                    transitions={   'done':'SAY_ENJOY',
                                                    'failed':'SAY_ENJOY'})
            smach.StateMachine.add( 'SAY_ENJOY',
                                    Say(robot, ["Enjoy your drink!", "I hope your thirsty, enjoy!"]),
                                    transitions={"spoken":"done"})
                                    
class HandoverToUnknownHuman(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["done"])

        if grasp_arm == "left":
            arm = robot.leftArm
        if grasp_arm == "right":
            arm = robot.rightArm

        with self:
            smach.StateMachine.add('GOTO_MEETING_POINT',
                                    GotoMeetingPoint(robot),
                                    transitions={   "found":"PRESENT_DRINK", 
                                                    "not_found":"PRESENT_DRINK", 
                                                    "no_goal":"PRESENT_DRINK",  
                                                    "all_unreachable":"PRESENT_DRINK"})

            smach.StateMachine.add( 'PRESENT_DRINK',
                                    Say(robot, ["Here's your drink", "Here you go!"]),
                                    transitions={"spoken":"POSE"})

            smach.StateMachine.add( 'POSE',
                                    Handover_pose(arm, robot),
                                    transitions={   'succeeded':'PLEASE_TAKE',
                                                    'failed':'PLEASE_TAKE'})
            
            smach.StateMachine.add( 'PLEASE_TAKE',
                                    Say(robot, ["Please hold the drink, I'm going to let it go.", "Please take the drink, i'll let it go"]),
                                    transitions={"spoken":"OPEN_GRIPPER"})

            smach.StateMachine.add( "OPEN_GRIPPER", 
                                    SetGripper(robot, arm, gripperstate=0, drop_from_frame="/grippoint_left"), #open
                                    transitions={   'succeeded':'CLOSE_AFTER_DROP',
                                                    'failed':'CLOSE_AFTER_DROP'})
            smach.StateMachine.add( 'CLOSE_AFTER_DROP',
                                    SetGripper(robot, arm, gripperstate=1), #close
                                    transitions={   'succeeded':'RESET_ARM',
                                                    'failed':'RESET_ARM'})
            smach.StateMachine.add('RESET_ARM', 
                                    ArmToPose(robot, arm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), 
                                    transitions={   'done':'SAY_ENJOY',
                                                    'failed':'SAY_ENJOY'})
            smach.StateMachine.add( 'SAY_ENJOY',
                                    Say(robot, ["Enjoy your drink!", "I hope your thirsty, enjoy!"]),
                                    transitions={"spoken":"done"})

class CocktailParty(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])

        # Queries:
        #query_meeting_point = Compound("waypoint", Compound("meeting_point", "Object"), Compound("pose_2d", "X", "Y", "Phi"))
        query_party_room = Compound("waypoint", "party_room", Compound("pose_2d", "X", "Y", "Phi"))
        query_grabpoint = Conjunction(  Compound("goal", Compound("serve", "Drink")),
                                           Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                           Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                           Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))

        if grasp_arm == "left":
            arm = robot.leftArm
        if grasp_arm == "right":
            arm = robot.rightArm

        with self:

            smach.StateMachine.add( "START_CHALLENGE",
                                    StartChallengeRobust(robot, "initial"), #query_meeting_point
                                    transitions={   "Done":"GOTO_MEETING_POINT", 
                                                    "Aborted":"Aborted", 
                                                    "Failed":"SAY_FAILED"})

            smach.StateMachine.add('GOTO_MEETING_POINT',
                                    GotoMeetingPoint(robot),
                                    transitions={   "found":"ITERATE_PERSONS", 
                                                    "not_found":"ITERATE_PERSONS", 
                                                    "no_goal":"ITERATE_PERSONS",  
                                                    "all_unreachable":"ITERATE_PERSONS"})

            persons_iterator = smach.Iterator(  outcomes=['served', 'not_served'], 
                                                it=lambda: range(1),
                                                it_label="person_index",
                                                input_keys=[],
                                                output_keys=[],
                                                exhausted_outcome='Done')
            
            with persons_iterator:
                single = smach.StateMachine(outcomes=['served', 'not_served'])
                with single:
                    #TODO: Retract visited(X) befor elooking for a person back again.
                    smach.StateMachine.add( "RETRACT_VISITED",
                                            Retract_facts(robot, [Compound("visited", "X")]),
                                            transitions={"retracted":"WAIT_FOR_PERSON"})

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

                    smach.StateMachine.add( "TAKE_ORDER",
                                            Ask_drink(robot),
                                            transitions={   "done":"LOOK_FOR_DRINK",
                                                            "failed":"TAKE_ORDER"})
                    
                    # smach.StateMachine.add('TAKE_ORDER', 
                    #                         Timedout_QuestionMachine(
                    #                                 robot=robot,
                    #                                 default_option = "coke", 
                    #                                 sentence = "What would you like to drink?", 
                    #                                 options = { "coke":Compound("goal", Compound("serve", "coke")),
                    #                                             "fanta":Compound("goal", Compound("serve", "fanta"))
                    #                                           }),
                    #                         transitions={   'answered':'LOOK_FOR_DRINK',
                    #                                         'not_answered':'TAKE_ORDER'})
                       
                    smach.StateMachine.add( 'LOOK_FOR_DRINK',
                                            LookForDrink(robot),
                                            transitions={   "looking":"LOOK_FOR_DRINK",
                                                            "found":'PICKUP_DRINK',
                                                            "not_found":'SAY_DRINK_NOT_FOUND'})
                    
                    smach.StateMachine.add( 'SAY_DRINK_NOT_FOUND',
                                            Say(robot, ["I could not find the drink you wanted.", 
                                                        "I looked really hard, but I couldn't find your drink."]),
                                            transitions={   'spoken':'GOTO_INITIAL_FAIL' }) 

                    smach.StateMachine.add( 'PICKUP_DRINK',
                                            GrabMachine(arm, robot, query_grabpoint),
                                            transitions={   "succeeded":"RETRACT_VISITED_2",
                                                            "failed":'LOOK_FOR_DRINK' }) 
                    # NOW IF FAILED, THE ROBOT INFINITLY LOOPS TRYING TO GRAPS
                    smach.StateMachine.add( 'HUMAN_HANDOVER',
                                            Human_handover(arm,robot),
                                            transitions={   'succeeded':'GOTO_INITIAL_FAIL',
                                                            'failed':'GOTO_INITIAL_FAIL' })

                    smach.StateMachine.add( "RETRACT_VISITED_2",
                                            Retract_facts(robot, [Compound("visited", "X")]),
                                            transitions={"retracted":"LOOK_FOR_PERSON"})           

                    smach.StateMachine.add( 'LOOK_FOR_PERSON',
                                            LookForPerson(robot),
                                            transitions={   "looking":"LOOK_FOR_PERSON",
                                                            "found":'HANDOVER_DRINK',
                                                            "not_found":'HANDOVER_DRINK_UNKNOWN_PERSON'})

                    smach.StateMachine.add( 'SAY_PERSON_NOT_FOUND',
                                            Say(robot, ["I could not find you. Going to the meeting point", 
                                                        "I can't find you. I really don't like fluids, so I'm coming to the meeting point.",
                                                        "I could not find you. Going to the meeting point"]),
                                            transitions={   'spoken':'HANDOVER_DRINK_UNKNOWN_PERSON' }) #GOTO_INITIAL_FAIL

                    smach.StateMachine.add( 'HANDOVER_DRINK_UNKNOWN_PERSON',
                                            HandoverToUnknownHuman(robot),
                                            transitions={"done":"GOTO_INITIAL_FAIL"})

                    smach.StateMachine.add( 'HANDOVER_DRINK',
                                            HandoverToKnownHuman(robot),
                                            transitions={"done":"GOTO_INITIAL_SUCCESS"})

                    smach.StateMachine.add( "GOTO_INITIAL_SUCCESS",
                                            NavigateGeneric(robot, goal_query=query_party_room),
                                            transitions={   "arrived":"served", 
                                                            "unreachable":"served", 
                                                            "preempted":"not_served", 
                                                            "goal_not_defined":"served"})

                    smach.StateMachine.add( "GOTO_INITIAL_FAIL",
                                            NavigateGeneric(robot, goal_query=query_party_room),
                                            transitions={   "arrived":"not_served", 
                                                            "unreachable":"not_served", 
                                                            "preempted":"not_served", 
                                                            "goal_not_defined":"not_served"})

                persons_iterator.set_contained_state('SINGLE_COCKTAIL_SM', 
                                                          single, 
                                                          loop_outcomes=['served', 'not_served'])

            smach.StateMachine.add( 'ITERATE_PERSONS', 
                                    persons_iterator, 
                                    transitions={'served':'EXIT',
                                                'not_served':'EXIT',
                                                'Done':"EXIT"})

            smach.StateMachine.add( "EXIT",
                                    NavigateGeneric(robot, goal_name="initial"),
                                    transitions={"arrived":"FINISH", 
                                                 "unreachable":"FINISH", 
                                                 "preempted":"FINISH", 
                                                 "goal_not_defined":"FINISH"})

            smach.StateMachine.add( 'FINISH', Finish(robot),
                                    transitions={'stop':'Done'})

            smach.StateMachine.add("SAY_FAILED", 
                                    Say(robot, "I could not accomplish my task, sorry about that, please forgive me."),
                                    transitions={   "spoken":"EXIT"})
 
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
    amigo.reasoner.query(Compound("retractall", Compound("type", "X", "Y")))

    amigo.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
    #amigo.reasoner.query(Compound("load_database", "challenge_cocktail_party", 'prolog/objects.pl'))

    amigo.reasoner.assertz(Compound("challenge", "cocktailparty"))

    initial_state = None
    #initial_state= "LOOK_FOR_DRINK"

    if initial_state == "LOOK_FOR_DRINK":
        amigo.reasoner.query(Compound("assert", Compound("goal", Compound("serve", "coke"))))
        amigo.reasoner.query(Compound("assert", Compound("current_person", "maxima")))

    machine = CocktailParty(amigo)
    
    if initial_state != None:
        machine.set_initial_state([initial_state])

    introserver = smach_ros.IntrospectionServer('server_name', machine, '/SM_ROOT_PRIMARY')
    introserver.start()
    try:
        machine.execute()
    except Exception, e:
        amigo.speech.speak(e)
    finally:
        introserver.stop()
