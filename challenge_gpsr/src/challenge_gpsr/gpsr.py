#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_gpsr')
import rospy
#import robot_parts.speech
from std_msgs.msg import String
import geometry_msgs
import smach
import sys

from robot_skills.amigo import Amigo
import robot_smach_states as states
import robot_skills.util.msg_constructors as msgs

from robot_smach_states.util.startup import startup
from robot_smach_states.util.designators import *
from robot_skills.util import msg_constructors as geom
import ed.msg

from datetime import datetime
from datetime import date

from robot_smach_states.util.geometry_helpers import *
from cb_planner_msgs_srvs.msg import PositionConstraint

#import data
from robocup_knowledge import load_knowledge
data = load_knowledge('challenge_gpsr')

global ROBOT_NAME_SPECIAL
LOCATION_NR_IN_ROOM = 0
global LOC_ROOM
LOC_ROOM = "cabinet"
# global HUMAN 
# HUMAN = ed.msg.EntityInfo

###########################
# Created by: Erik Geerts #
###########################

#######################
##### TODO LIST!! #####
#### NA EINDHOVEN  ####
#######################
# - dropoff points in eindhoven definieren.
# - remove timeout of 5 minutes -> DID YOU SAY SOMETHING, IN ANY CASE, I DID NOT HEAR YOU!

class InitializeWorldModel(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.ed.configure_kinect_segmentation(continuous=False)
        self.robot.ed.configure_perception(continuous=False)
        self.robot.ed.reset()

        return "done"

class QueryFirstAnswerDesignator(Designator):
    def __init__(self, robot, reasoner_query):
        super(QueryFirstAnswerDesignator, self).__init__(resolve_type=str)
        self.robot = robot
        self.reasoner_query = reasoner_query

    def resolve(self):
        first_answer = self.robot.reasoner.query_first_answer(self.reasoner_query)
        if not first_answer:
            return None
        print "first_answer is:", str(first_answer)
        return str(first_answer)

class ObjectGraspDesignator(Designator):
    def __init__(self, robot):
        super(ObjectGraspDesignator, self).__init__(resolve_type=ed.msg.EntityInfo)
        self.robot = robot
        #self.object_to_find = object_to_find

    def resolve(self):
        object_type = str(self.robot.reasoner.query_first_answer("action_info('2','2_vb_take_object_loc',A,_)"))
        #object_type = object_to_find.resolve()
        ## FOR TESTING:
        #object_type = "cola"
        has_type = lambda entity: entity.type == object_type
        location = str(self.robot.reasoner.query_first_answer("action_info('2','2_vb_take_object_loc',_,A)"))
        #location = str(self.robot.reasoner.query_first_answer(self.query))
        ## FOR TESTING:
        #location = "hallway_couch"
        grab_item_designator = EdEntityDesignator(QueryFirstAnswerDesignator(self.robot, "action_info('3','3_place_location',A)"))

        #self.robot.ed

        print "test1"
        print grab_item_designator.resolve()
        print "test2"
        return grab_item_designator.resolve()

class PlaceLocDesignator(Designator):
    def __init__(self, robot):
        super(PlaceLocDesignator, self).__init__(resolve_type=ed.msg.EntityInfo)
        self.robot = robot

    def resolve(self):
        loc_entity = robot.ed.get_entity(id=QueryFirstAnswerDesignator(robot, "action_info('3','3_place_location',A)"))

        print "loc_entity in PlaceLocDesignator: \n"
        print loc_entity.resolve()
        return loc_entity.resolve()

class PossibleHumanFlagsDesignator(Designator):
    def __init__(self, robot, room):
        super(PossibleHumanFlagsDesignator, self).__init__(resolve_type=ed.msg.EntityInfo)
        self.robot = robot
        self.room = room

    def resolve(self):
        
        possible_humans =  self.robot.ed.get_closest_possible_person_entity(room=self.room.resolve())
        
        return possible_humans


### EXAMPLE SENTENCES stated in new conceptual rulebook.
# Go to the bedroom, find a person and tell the time (missing object-interaction part).
# Go to the kitchen, find a person and follow her (missing object-interaction part).
# Go to the dinner-table, grasp the crackers, and take them to the TV.
# Go to the shelf, count the drinks and report to me.

# Take this object and bring it to Susan at the hall.
# Bring a coke to the person in the living room and answer him a question.
# Offer a drink to the person at the door.


class Ask_action(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "failed"])
        self.robot = robot

    def execute(self, userdata):
        self.robot.head.look_at_standing_person()

        self.robot.speech.speak("What can I do for you?")

        res = self.robot.ears.recognize(spec=data.spec, choices=data.choices, time_out=rospy.Duration(30))
        self.robot.head.cancel_goal()
        if not res:
            self.robot.speech.speak("My ears are not working properly, can i get a restart?.")
            return "failed"
        try:
            if res.result:
                say_result_filter_me = self.replace_word(res.result,"me","you")
                say_result = self.replace_word(say_result_filter_me,"your","my")
                self.robot.speech.speak("Okay I will {0}".format(say_result))
                print say_result
                self.save_action(res)

            else:
                self.robot.speech.speak("Sorry, could you please repeat?")
                return "failed"
        except KeyError:
            print "[what_did_you_say] Received question is not in map. THIS SHOULD NEVER HAPPEN!"
            return "failed"

        return "done"

    def replace_word(self,string,word_in,word_out):
        try:
            if string[:(len(word_in)+1)] == (word_in+" "):
                string = string.replace(string[:len(word_in)],word_out)

            if string[(len(string)-len(word_in)-1):] == (" "+word_in):
                string = string.replace(string[(len(string)-len(word_in)):],word_out)

            string = string.replace(" "+word_in+" "," "+word_out+" ")

        except KeyError:
            print "[gpsr] Received action is to short."

        return string

    def save_action(self,res):
        
        self.robot.reasoner.query("retractall(action_info(A,B,C))")
        self.robot.reasoner.query("retractall(current_action(A))")
        self.robot.reasoner.query("retractall(action_info(A,B))")

        self.robot.reasoner.assertz("action_info('complete_action','"+self.add_underscores(str(res.result))+"')")

        whole_action = self.robot.reasoner.query("action_info('complete_action',B)")
        print "whole_action = ", whole_action

        for choice_key, choice_value in res.choices.iteritems():
            print "choice_key = ", self.add_underscores(str(choice_key))
            print "choice_value = '",self.add_underscores(str(choice_value)),"'"

            if not choice_key[:1].find("1"):
                #print " 1 = ", choice_key[:1]             
                self.robot.reasoner.assertz("action_info('1','"+self.add_underscores(str(choice_key))+"','"+self.add_underscores(str(choice_value))+"')")
            if not choice_key[:1].find("2") : 
                #print " 2 = ", choice_key[:1] 
                self.robot.reasoner.assertz("action_info('2','"+self.add_underscores(str(choice_key))+"','"+self.add_underscores(str(choice_value))+"')")  
            if not choice_key[:1].find("3"):
                #print " 3 = ", choice_key[:1]   
                self.robot.reasoner.assertz("action_info('3','"+self.add_underscores(str(choice_key))+"','"+self.add_underscores(str(choice_value))+"')")


    #   todo: 
    #       - First get for first action the simple action (for now only navigating to location, room or object, later also direct grabbing object)
    #       - Then get action 2
    #       - Then action 3. (mainly dropoff, report, follow, answer question (including tell time))
    #
    def add_underscores(self, string):
        return str(string.replace(" ","_"))

class Query_specific_action(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["navigate_room", "navigate_location", "take_object_loc", "look_object_loc", "find_person", "answer_question", "answer_special", "return_to_operator", "return_to_person", "place_item","no_action"])
        self.robot = robot

    def execute(self, userdata):

        action_nr = self.robot.reasoner.query("current_action(A)")
        print action_nr
        if action_nr:
            #print action_nr[0]['A']
            current_action = int(action_nr[0]['A'])

        else:
            self.robot.reasoner.assertz("current_action('1')")
            current_action = int("1")
        
        #current_action = 2 ## for testing
        print "current action = ", current_action
        
        if current_action == 1:
            action_1 = self.robot.reasoner.query("action_info('1',A,B)")
            for x in action_1:
                for choice_key, choice_value in x.iteritems():
                    print "action 1: choice_key = ", str(choice_key)
                    print "action 1: choice_value = ", str(choice_value)

                    if str(choice_value) == "1_room":
                        room=str(self.robot.reasoner.query_first_answer("action_info('1','1_room',A)"))
                        print "room =", room
                        self.robot.reasoner.query("retractall(action_info('1','1_locations_rooms',A))")
                        self.robot.reasoner.assertz("action_info('1','1_locations_rooms',"+str(room)+")")
                        print self.robot.reasoner.query_first_answer("action_info('1','1_locations_rooms',A)")
                        return "navigate_room"

                    if str(choice_value) == "1_location":
                        location=str(self.robot.reasoner.query_first_answer("action_info('1','"+str(choice_value)+"',A)"))
                        self.robot.reasoner.query("retractall(action_info('1','"+str(choice_value)+"',A))")
                        #self.robot.reasoner.assertz("action_info('1','1_location','"+"gpsr_"+str(location)+"')")
                        self.robot.reasoner.assertz("action_info('1','1_location','"+str(location)+"')")
                        print self.robot.reasoner.query_first_answer("action_info('1','1_location',A)")
                        return "navigate_location"


        elif current_action == 2:
            action_2 = self.robot.reasoner.query("action_info('2',A,B)")
            for x in action_2:
                for choice_key, choice_value in x.iteritems():
                    
                    print "action 2: choice_key = ", str(choice_key)
                    print "action 2: choice_value = ", str(choice_value)

                    if str(choice_value) == "2_vb_take":
                        take_object = str(self.robot.reasoner.query_first_answer("action_info('2','2_object',A)"))
                        location = str(self.robot.reasoner.query_first_answer("action_info('1','1_location',A)"))
                        print "take_object =", take_object
                        print "location =", location
                        self.robot.reasoner.query("retractall(action_info('2','2_vb_take_object_loc',A,B))")
                        self.robot.reasoner.assertz("action_info('2','2_vb_take_object_loc',"+str(take_object)+","+str(location)+")")
                        print self.robot.reasoner.query_first_answer("action_info('2','2_vb_take_object_loc',A,B)")
                        return "take_object_loc"

                    if str(choice_value) == "2_vb_find":

                        #First check if the word 'person' can be found in the action:
                        if str(self.robot.reasoner.query_first_answer("action_info('complete_action',A)")).find('person')>-1:
                            print "person is found"
                            
                            room = self.robot.reasoner.query_first_answer("action_info('1','1_locations_rooms',A)")
                            print "room =", room
                            self.robot.reasoner.query("retractall(action_info('2','2_look_person_loc',A))")
                            self.robot.reasoner.assertz("action_info('2','2_look_person_loc',"+str(room)+")")
                            print self.robot.reasoner.query_first_answer("action_info('2','2_look_person_loc',A)")

                            return "find_person"
                        else:
                            room = self.robot.reasoner.query_first_answer("action_info('1','1_locations_rooms',A)")
                            print "room =", room
                            take_object = str(self.robot.reasoner.query_first_answer("action_info('2','2_object',A)"))
                            print "take_object =", take_object
                            self.robot.reasoner.query("retractall(action_info('2','2_look_object_loc',A,B))")
                            self.robot.reasoner.assertz("action_info('2','2_look_object_loc',"+str(take_object)+","+str(room)+")")
                            print self.robot.reasoner.query_first_answer("action_info('2','2_look_object_loc',A,B)")
                            return "look_object_loc"


        elif current_action == 3:
            action_3 = self.robot.reasoner.query("action_info('3',A,B)")
            for x in action_3:
                for choice_key, choice_value in x.iteritems():
                    
                    print "action 3: choice_key = ", str(choice_key)
                    print "action 3: choice_value = ", str(choice_value)

                    if str(choice_value) == "3_question":
                        return "answer_question"

                    if str(choice_value) == "3_vb_speak":
                        return "answer_special"

                    if str(choice_value) == "3_person_me":
                        return "return_to_operator"

                    if str(choice_value) == "3_place_location":
                        return "place_item"

                    if str(choice_value) == "3_person":
                        return "return_to_person"


        # print answers[0]

        # print answers[0]['A']

        # answers = self.robot.reasoner.query("retractall(action_info(_,_))")
        # print "test"
        # print self.robot.reasoner.query("action_info(A,B)")
        return "no_action"

class Finished_goal(smach.State):
    # Checks how many tasks have been done and if another task is needed
    # Does this check with the database in the reasoner
    def __init__(self,robot):
        smach.State.__init__(self, outcomes=["new_task", "tasks_completed"])

        self.robot = robot

    def execute(self, userdata):


        action_nr = self.robot.reasoner.query("current_action(A)")
        print action_nr
        if action_nr:
            #print "finish"
            #print int(action_nr[0]['A'])
            #print str(int(action_nr[0]['A'])+1)
            #before_action_nr = self.robot.reasoner.query("current_action(A)")
            #print "query before = ", before_action_nr
            self.robot.reasoner.query("retractall(current_action(_))")
            self.robot.reasoner.assertz("current_action('"+str(int(action_nr[0]['A'])+1)+"')")
            action_nr = self.robot.reasoner.query("current_action(A)")
            current_action = int(action_nr[0]['A'])
            #print "query after = ", current_action

        else:
            print "[gpsr] current_action not found. This should not happen."

        if current_action == 4:
            return "tasks_completed"
        return "new_task"

class HearQuestion(smach.State):
    def __init__(self, robot, time_out=rospy.Duration(10)):
        smach.State.__init__(self, outcomes=["answered", "error", "failed"])
        self.robot = robot
        self.time_out = time_out

    def execute(self, userdata):
        self.robot.head.look_at_standing_person()

        res = self.robot.ears.recognize(spec=data.spec_questions, choices=data.choices_questions, time_out=self.time_out)

        if not res:
            self.robot.speech.speak("My ears are not working properly, can i get a restart?.")
            return "error"

        if res:
            if "question" in res.choices:
                rospy.loginfo("Question was: '%s'?"%res.result)
                self.robot.speech.speak("The answer is %s"%data.choice_answer_mapping[res.choices['question']])
            else:
                self.robot.speech.speak("Sorry, I do not understand your question")
                return "failed"

        self.robot.head.cancel_goal()
        return "answered"

class SpeakSpecial(smach.State):
    def __init__(self, robot, time_out=rospy.Duration(10)):
        smach.State.__init__(self, outcomes=["answered", "failed"])
        self.robot = robot
        self.time_out = time_out

    def execute(self, userdata):
        self.robot.head.look_at_standing_person()

        say_type = str(self.robot.reasoner.query_first_answer("action_info('3','3_name_time_date',A)"))

        if say_type == 'your_name':
            self.robot.speech.speak("My name is %s" % ROBOT_NAME_SPECIAL)
            self.robot.head.cancel_goal()
            return "answered"
        if say_type == 'the_name_of_your_team':
            self.robot.speech.speak("My team's name is Tech United")
            self.robot.head.cancel_goal()
            return "answered"

        ## TIME SPECIALS
        if (say_type == 'the_time' or say_type == "what_time_is_it" or say_type == "what_time_it_is"):
            time="It is %s" % datetime.now().strftime("%I %M %p")
            self.robot.speech.speak(time)
            self.robot.head.cancel_goal()
            return "answered"

        if (say_type == 'what_day_is_today' or say_type == 'the day of the week'):
            today = "It is %s" % datetime.now().strftime("%A")
            self.robot.speech.speak(today)
            self.robot.head.cancel_goal()
            return "answered"

        if (say_type == 'the_date'):
            month = datetime.now().strftime("%B")
            day_nr = datetime.now().strftime("%d")
            year = datetime.now().strftime("%Y")

            the_date = "Today it is %s %s of the year %s" % (month, day_nr, year)
            #print the_date
            self.robot.speech.speak(the_date)
            self.robot.head.cancel_goal()
            return "answered"

        if (say_type == 'the_day_of_the_month'):
            month = datetime.now().strftime("%B")
            day_nr = datetime.now().strftime("%d")
            the_day_of_the_month = "Today it is %s %s" % (month, day_nr)
            #print the_date
            self.robot.speech.speak(the_day_of_the_month)
            self.robot.head.cancel_goal()
            return "answered"

        if (say_type == 'what_day_is_tomorrow'):
            today = int(datetime.now().strftime("%w"))
            week_list = ["Monday", "Tuesday","Wednesday","Thursday","Friday","Saturday","Sunday"]
            tomorrow = week_list[today]
            the_day_of_the_month = "Tomorrow it is %s" % (tomorrow)
            #print the_day_of_the_month
            self.robot.speech.speak(the_day_of_the_month)
            self.robot.head.cancel_goal()
            return "answered"
        
        self.robot.speech.speak("Something went wrong, I'm sorry", block=False)
        return "failed"



class FindObjectInRoom(smach.StateMachine):
    """Initialize, wait for the door to be opened and drive inside"""

    def __init__(self, robot, room, object_to_find):
        smach.StateMachine.__init__(self, outcomes=["Object_found", "Object_not_found", "No_locations", "Failed"])

        with self:

            ## room is known -> locations to search for object found in knowledge file. 
            ## One by one these objects need to be checked for the object that needs to be picked up.
            ## If found, then pickup.

            #Class ChooseLocation in final amigo file. 

            # smach.StateMachine.add("SAY_FIND_OBJECT_IN_ROOM",
            #                    states.Say(robot,"Let's   FindObjectInRoom", block=True),
            #                    transitions={'spoken':'CHECK_FOR_LOCATIONS'})

            @smach.cb_interface(outcomes=['location_found','no_location'])
            def get_location_room(userdata):
                answer = room.resolve()
                #print "! Room = ", answer
                #print "BEFORE LOCATION_NR_IN_ROOM = ", LOCATION_NR_IN_ROOM

                try:
                    if len(data.rooms_detailed[answer]) > LOCATION_NR_IN_ROOM:
                        LOC_ROOM = data.rooms_detailed[answer][LOCATION_NR_IN_ROOM]
                    
                        global LOCATION_NR_IN_ROOM 
                        LOCATION_NR_IN_ROOM += 1
                        #print "AFTER LOCATION_NR_IN_ROOM = ", LOCATION_NR_IN_ROOM
                        #print "! LOC_ROOM = ", LOC_ROOM
                        robot.reasoner.query("retractall(room_loc(B))")
                        robot.reasoner.assertz("room_loc("+str(LOC_ROOM)+")")
                        print "room_loc = ", robot.reasoner.query_first_answer("room_loc(A)")

                        return 'location_found'
                    else:
                        return 'no_location'

                except KeyError:
                    print "[find_loc] No loc found anymore, can happen"
                    return "no_location"

            smach.StateMachine.add( "CHECK_FOR_LOCATIONS",
                                    smach.CBState(get_location_room),
                                    transitions={'location_found':'NAV_TO_LOC',
                                                 'no_location':'No_locations'})

            smach.StateMachine.add('NAV_TO_LOC',
                                    states.NavigateToSymbolic(robot, 
                                        {EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "room_loc(A)")) : "in_front_of" }, 
                                        EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "room_loc(A)"))),
                                    transitions={   'arrived'           :   'LOOKAT_LOC',
                                                    'unreachable'       :   'NAV_TO_LOC_RETRY',
                                                    'goal_not_defined'  :   'NAV_TO_LOC_RETRY'})

            smach.StateMachine.add('NAV_TO_LOC_RETRY',
                                        states.NavigateToObserve(robot, EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "room_loc(A)")), radius=0.5),
                                        transitions={   'arrived':'LOOKAT_LOC',
                                                        'unreachable':'Object_not_found',
                                                        'goal_not_defined':'Object_not_found'})
            
            smach.StateMachine.add( "LOOKAT_LOC",
                                         states.LookOnTopOfEntity(robot, EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "room_loc(A)")), waittime=5.0),
                                         transitions={  'succeeded'         :'CHECK_FOR_OBJECT',
                                                        'failed'            :'CHECK_FOR_OBJECT'})

            @smach.cb_interface(outcomes=['object_found','object_not_found'])
            def check_for_object(userdata):
                answer = object_to_find.resolve()
                print "! Object_type_to_find = ", answer

                try:
                    object_type = object_to_find.resolve()
                    # has_type = lambda entity: entity.type == object_type


                    #     #print "aaaaa, test has_type = ", object_type
                    location = str(QueryFirstAnswerDesignator(robot, "room_loc(A)").resolve())
                    print "location = ", location
                    location_ent = robot.ed.get_entity(id=location, parse=False)
                    #print "location_ent = ", location_ent
                    #     #print "aaaaa, test location = ", location
                    #     grab_item_designator = EdEntityDesignator(robot, center_point=geom.PointStamped(frame_id="/"+location), radius=2.0,
                    #                                                             criteriafuncs=[has_type], debug=False)

                    #     #print "test1 \n"
                    #     #print grab_item_designator.resolve()
                    #     #print "\n test2"
                    #     if grab_item_designator.resolve():
                    #         return 'object_found'
                    #     else:
                    #         return 'object_not_found'

                    ''' Enable kinect segmentation plugin (only one image frame) '''
                    entity_ids = robot.ed.segment_kinect(max_sensor_range=2)

                    print "entity_ids = ", entity_ids

                    ''' Get all entities that are returned by the segmentation and are on top of the shelf '''
                    id_list = [] # List with entities that are flagged with 'perception'                
                    for entity_id in entity_ids:
                        e = robot.ed.get_entity(entity_id)

                        if e and onTopOff(e, location_ent):
                            id_list.append(e.id)

                    print "id_list = ", id_list

                    ''' Try to classify the objects on the shelf '''
                    print "data.objects_known_recognize =", data.objects_known_recognize

                    entity_types = robot.ed.classify(ids=id_list, types=data.objects_known_recognize)

                    ''' Check all entities that were flagged to see if they have received a 'type' it_label
                    if so: recite them and lock them '''

                    correct_object_type_ids_list = []
                    not_correct_object_type_ids_list = []
                    for i in range(0, len(id_list)):
                        e_id = id_list[i]
                        print "e_id = ", e_id
                        e_type = entity_types[i]
                        print "e_type = ", e_type
                        if e_type == object_type:
                            correct_object_type_ids_list.append(e_id)
                        elif e_type:
                            not_correct_object_type_ids_list.append(e_id)

                    print "correct_object_type_ids_list = ", correct_object_type_ids_list
                    
                    if len(correct_object_type_ids_list) > 0:
                        return 'object_found'
                    else:
                        return 'object_not_found'

                except KeyError:
                    print "[find_loc] Keyerror at checking entities in snapshot. Should not happen!"
                    return "object_not_found"


            smach.StateMachine.add( "CHECK_FOR_OBJECT",
                                    smach.CBState(check_for_object),
                                    transitions={'object_found':'SAY_FOUND_OBJECT',
                                                 'object_not_found':'CHECK_IF_LOCATIONS_LEFT'})


            # smach.StateMachine.add('NAV_TO_LOC',
            #                         states.NavigateToSymbolic(robot, 
            #                             {EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "room_loc(A)")) : "in_front_of_2" }, 
            #                             EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "room_loc(A)"))),
            #                         transitions={   'arrived'           :   'LOOKAT_LOC',
            #                                         'unreachable'       :   'CHECK_IF_LOCATIONS_LEFT',
            #                                         'goal_not_defined'  :   'CHECK_IF_LOCATIONS_LEFT'})


            @smach.cb_interface(outcomes=['ok'])
            def dynamic_say(userdata):
                try:
                    object_type = object_to_find.resolve()
                    if ((str(QueryFirstAnswerDesignator(robot, "room_loc(A)").resolve()) == "bookcase/shelf1") or 
                    (str(QueryFirstAnswerDesignator(robot, "room_loc(A)").resolve()) == "bookcase/shelf2") or
                    (str(QueryFirstAnswerDesignator(robot, "room_loc(A)").resolve()) == "bookcase/shelf3") or
                    (str(QueryFirstAnswerDesignator(robot, "room_loc(A)").resolve()) == "bookcase/shelf4") or
                    (str(QueryFirstAnswerDesignator(robot, "room_loc(A)").resolve()) == "bookcase/shelf5") or
                    (str(QueryFirstAnswerDesignator(robot, "room_loc(A)").resolve()) == "bookcase/shelf6") or
                    (str(QueryFirstAnswerDesignator(robot, "room_loc(A)").resolve()) == "bookcase/shelf7") or
                    (str(QueryFirstAnswerDesignator(robot, "room_loc(A)").resolve()) == "bookcase/shelf8")):
                        robot.speech.speak("I found the " + str(object_type) + ". It is on the bookcase")
                    else:
                        robot.speech.speak("I found the " + str(object_type) + ". It is on the " + str(QueryFirstAnswerDesignator(robot, "room_loc(A)").resolve()))
                    return 'ok'

                except KeyError:
                    print "[say] problem should not happen"
                    return "ok"

            smach.StateMachine.add("SAY_FOUND_OBJECT",
                               smach.CBState(dynamic_say),
                               transitions={'ok':'Object_found'})

            ## HACK: IF IT IS THE LAST LOCATION IN THE ROOM AND NO OBJECT HAS BEEN FOUND, JUST SAY THAT THE OBJECT IS THERE.
            @smach.cb_interface(outcomes=['location_found','no_location'])
            def check_locations_in_room_left(userdata):
                answer = room.resolve()

                try:
                    if len(data.rooms_detailed[answer]) > LOCATION_NR_IN_ROOM:
                        return 'location_found'
                    else:
                        return 'no_location'

                except KeyError:
                    print "[find_loc] No loc found anymore, can happen"
                    return "no_location"

            smach.StateMachine.add('NAV_TO_LOC_POS2',
                                    states.NavigateToSymbolic(robot, 
                                        {EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "room_loc(A)")) : "in_front_of_pos2" }, 
                                        EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "room_loc(A)"))),
                                    transitions={   'arrived'           :   'LOOKAT_LOC_POS2',
                                                    'unreachable'       :   'CHECK_IF_LOCATIONS_LEFT',
                                                    'goal_not_defined'  :   'CHECK_IF_LOCATIONS_LEFT'})

            smach.StateMachine.add( "LOOKAT_LOC_POS2",
                                         states.LookOnTopOfEntity(robot, EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "room_loc(A)")), waittime=5.0),
                                         transitions={  'succeeded'         :'CHECK_FOR_OBJECT_POS2',
                                                        'failed'            :'CHECK_FOR_OBJECT_POS2'})

            smach.StateMachine.add( "CHECK_FOR_OBJECT_POS2",
                                    smach.CBState(check_for_object),
                                    transitions={'object_found':'SAY_FOUND_OBJECT',
                                                 'object_not_found':'CHECK_IF_LOCATIONS_LEFT'})

            smach.StateMachine.add( "CHECK_IF_LOCATIONS_LEFT",
                                    smach.CBState(check_locations_in_room_left),
                                    transitions={'location_found':'Object_not_found',
                                                 'no_location':'SAY_FOUND_OBJECT'})


class FindAndGoToPerson(smach.StateMachine):
    """Initialize, wait for the door to be opened and drive inside"""

    def __init__(self, robot, room):
        smach.StateMachine.__init__(self, outcomes=["Found", "Not_found", "Failed"])
        with self:

            ## room is known -> locations to search for object found in knowledge file. 
            ## One by one these objects need to be checked for the object that needs to be picked up.
            ## If found, then pickup.

            #Class ChooseLocation in final amigo file. 

            @smach.cb_interface(outcomes=['found','not_found'])
            def check_person(userdata):
                try:
                    rospy.sleep(2) #sleep is build in to make sure that amigo is standing still and has time to update entities.
                    possible_human = PossibleHumanFlagsDesignator(robot,room).resolve()
                    if possible_human:
                        print "! human = ", possible_human

                        # global HUMAN
                        # HUMAN = possible_human

                        robot.speech.speak("I found a person")
                        return 'found'
                    else:
                        return 'not_found'

                except KeyError:
                    print "[FindPerson] problem should not happen"
                    return "not_found"

            smach.StateMachine.add("CHECK_FOR_PERSON",
                               smach.CBState(check_person),
                               transitions={'found':'GO_TO_PERSON',
                                            'not_found':'DRIVE_TO_CENTER_ROOM'})

            smach.StateMachine.add('GO_TO_PERSON',
                                    states.NavigateToObserve(robot, PossibleHumanFlagsDesignator(robot,room), radius=0.7),
                                    transitions={   'arrived':'LOOK_AT_PERSON_FOUND',
                                                    'unreachable':'LOOK_AT_PERSON_NOT_FOUND',
                                                    'goal_not_defined':'LOOK_AT_PERSON_NOT_FOUND'})

            smach.StateMachine.add('LOOK_AT_PERSON_FOUND',
                                    states.LookAtEntity(robot, PossibleHumanFlagsDesignator(robot,room), waittime=1.0),
                                    transitions={   'succeeded':'SAY_HELLO',
                                                    'failed':'SAY_HELLO'})

            smach.StateMachine.add('LOOK_AT_PERSON_NOT_FOUND',
                                    states.LookAtEntity(robot, PossibleHumanFlagsDesignator(robot,room), waittime=1.0),
                                    transitions={   'succeeded':'SAY_SORRY',
                                                    'failed':'SAY_SORRY'})
            smach.StateMachine.add("SAY_HELLO",
                               states.Say(robot,"Hello there!", block=True),
                               transitions={'spoken':'Found'})

            smach.StateMachine.add("SAY_SORRY",
                               states.Say(robot,"Sorry, I was not able to reach you. Please come to me.", block=True),
                               transitions={'spoken':'Failed'})


            smach.StateMachine.add('DRIVE_TO_CENTER_ROOM',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id_designator=room), radius=0.2),
                                    transitions={   'arrived':'CHECK_FOR_PERSON_1',
                                                    'unreachable':'DRIVE_TO_CENTER_ROOM_BACKUP',
                                                    'goal_not_defined':'DRIVE_TO_CENTER_ROOM_BACKUP'})

            smach.StateMachine.add('DRIVE_TO_CENTER_ROOM_BACKUP',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id_designator=room), radius=0.4),
                                    transitions={   'arrived':'CHECK_FOR_PERSON_1',
                                                    'unreachable':'CHECK_FOR_PERSON_1',
                                                    'goal_not_defined':'CHECK_FOR_PERSON_1'})

            smach.StateMachine.add("CHECK_FOR_PERSON_1",
                               smach.CBState(check_person),
                               transitions={'found':'GO_TO_PERSON',
                                            'not_found':'TURN_90_DEGREES_1'})

            smach.StateMachine.add('TURN_90_DEGREES_1',
                                    Turn_90_degrees(robot),
                                    transitions={   'turned':'CHECK_FOR_PERSON_2'})

            smach.StateMachine.add("CHECK_FOR_PERSON_2",
                               smach.CBState(check_person),
                               transitions={'found':'GO_TO_PERSON',
                                            'not_found':'TURN_90_DEGREES_2'})

            smach.StateMachine.add('TURN_90_DEGREES_2',
                                    Turn_90_degrees(robot),
                                    transitions={   'turned':'CHECK_FOR_PERSON_3'})

            smach.StateMachine.add("CHECK_FOR_PERSON_3",
                               smach.CBState(check_person),
                               transitions={'found':'GO_TO_PERSON',
                                            'not_found':'TURN_90_DEGREES_3'})

            smach.StateMachine.add('TURN_90_DEGREES_3',
                                    Turn_90_degrees(robot),
                                    transitions={   'turned':'CHECK_FOR_PERSON_4'})

            smach.StateMachine.add("CHECK_FOR_PERSON_4",
                               smach.CBState(check_person),
                               transitions={'found':'GO_TO_PERSON',
                                            'not_found':'SAY_NOT_FOUND'})

            smach.StateMachine.add("SAY_NOT_FOUND",
                               states.Say(robot,"Sorry, I was not able to find you. Please come to me.", block=True),
                               transitions={'spoken':'Not_found'})

class Turn_90_degrees(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["turned"])
        self.robot = robot

    def execute(self, userdata):

        vth = 1.0
        th = 3.1415 / 2 # turns 90 degrees
        self.robot.base.force_drive(0, 0, vth, th / vth)

        return "turned"

class InspectLocationAndGrab(smach.State):

    def __init__(self, robot): #, unknown = False):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot

    def execute(self, userdata):

        object_type = str(self.robot.reasoner.query_first_answer("action_info('2','2_vb_take_object_loc',A,_)"))
        print "object_type = ", object_type
        location = str(self.robot.reasoner.query_first_answer("action_info('2','2_vb_take_object_loc',_,A)"))
        print "location = ", location
        location_ent = self.robot.ed.get_entity(id=location, parse=False)

        ''' Enable kinect segmentation plugin (only one image frame) '''
        entity_ids = self.robot.ed.segment_kinect(max_sensor_range=2)

        print "entity_ids = ", entity_ids

        ''' Get all entities that are returned by the segmentation and are on top of the shelf '''
        id_list = [] # List with entities that are flagged with 'perception'                
        for entity_id in entity_ids:
            e = self.robot.ed.get_entity(entity_id)

            if e and onTopOff(e, location_ent):
                id_list.append(e.id)

        print "id_list = ", id_list

        ''' Try to classify the objects on the shelf '''
        print "data.objects_known_recognize =", data.objects_known_recognize

        entity_types = self.robot.ed.classify(ids=id_list, types=data.objects_known_recognize)

        ''' Check all entities that were flagged to see if they have received a 'type' it_label
        if so: recite them and lock them '''

        correct_object_type_ids_list = []
        not_correct_object_type_ids_list = []
        for i in range(0, len(id_list)):
            e_id = id_list[i]
            print "e_id = ", e_id
            e_type = entity_types[i]
            print "e_type = ", e_type
            if e_type == object_type:
                correct_object_type_ids_list.append(e_id)
            elif e_type:
                not_correct_object_type_ids_list.append(e_id)

        print "correct_object_type_ids_list = ", correct_object_type_ids_list
        print "not_correct_object_type_ids_list = ", not_correct_object_type_ids_list

        # TODO: Get closest entity to amigo, this item should be easier to grab.
        
        # # Taken from world_model_ed.py 
        # # Sort by distance
        # try:
        #     entities = sorted(entities, key=lambda entity: hypot(center_point.x - entity.pose.position.x, center_point.y - entity.pose.position.y))
        # except:
        #     print "Failed to sort entities"
        #     return None

        # return entities[0]
        

        # Grab correct item if correct item is seen, otherwise take one from other type. In GPSR there is a big chance that the desired object type is on the location.
        left_arm = ArmDesignator(self.robot.arms, self.robot.leftArm)

        if len(correct_object_type_ids_list)>0:
            for i in range(0, len(correct_object_type_ids_list)):
                grabstate = states.Grab(self.robot, EdEntityDesignator(self.robot,id=correct_object_type_ids_list[i]), left_arm)
                result = grabstate.execute()
                rospy.loginfo("Amigo attempts to grasp an object that is classified as the desired object type")
                if result == 'done':
                    global ITEM 
                    ITEM = correct_object_type_ids_list[i]
                    return 'succeeded'

        if len(not_correct_object_type_ids_list)>0:

            for i in range(0, len(not_correct_object_type_ids_list)):
                grabstate = states.Grab(self.robot, EdEntityDesignator(self.robot,id=not_correct_object_type_ids_list[i]), left_arm)
                result = grabstate.execute()
                rospy.loginfo("Amigo attempts to grasp an object that is not classified as the desired object type")
                if result == 'done':
                    global ITEM 
                    ITEM = not_correct_object_type_ids_list[i]
                    return 'succeeded'

        return 'failed'





        # for i in range(0, len(id_list)):
        #     e_id = id_list[i]
        #     print "e_id = ", e_id
        #     e_type = entity_types[i]
        #     print "e_type = ", e_type
            
        #     if e_type == object_type:
        #         self.robot.speech.speak("I have seen a {0}".format(object_type), block=True)
        #         print "ed_id = ", e_id

        #         # In the gpsr I assume that there will only be one coke, no multiple cokes, therefore, directly grab the coke that is seen.
        #         # In other cases, one could check which object is the closest and grab that item.

        #         left_arm = ArmDesignator(self.robot.arms, self.robot.leftArm)

        #         grabstate = states.Grab(self.robot, EdEntityDesignator(self.robot,id=e_id), left_arm) #UnoccupiedArmDesignator(self.robot.arms, self.robot.leftArm))
        #         result = grabstate.execute()

        #         if result == 'done':
        #             global ITEM 
        #             ITEM = e_id

        #             #self.robot.reasoner.query("retractall(grabbed_item('item_id',A))")
        #             #self.robot.reasoner.assertz("grabbed_item('item_id','"+str(e_id)+"'')")
        #             return 'succeeded'
        #         else:
        #             return 'failed'

        # return 'failed'

class EmptySpotDesignator(Designator):
    """Designates an empty spot on the empty placement-shelve.
    It does this by queying ED for entities that occupy some space.
        If the result is no entities, then we found an open spot."""
    def __init__(self, robot, place_location_designator):
        super(EmptySpotDesignator, self).__init__(resolve_type=gm.PoseStamped)
        self.robot = robot
        self.place_location_designator = place_location_designator
        self._edge_distance = 0.1                   # Distance to table edge
        self._spacing = 0.15

    def resolve(self):
        place_location = self.place_location_designator.resolve()

        # points_of_interest = []
        points_of_interest = self.determinePointsOfInterest(place_location)

        def is_poi_occupied(poi):
            entities_at_poi = self.robot.ed.get_entities(center_point=poi, radius=self._spacing)
            return not any(entities_at_poi)

        open_POIs = filter(is_poi_occupied, points_of_interest)

        def distance_to_poi_area(poi):
            #Derived from navigate_to_place
            radius = math.hypot(self.robot.grasp_offset.x, self.robot.grasp_offset.y)
            x = poi.point.x
            y = poi.point.y
            ro = "(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, radius+0.075)
            ri = "(x-%f)^2+(y-%f)^2 > %f^2"%(x, y, radius-0.075)
            pos_constraint = PositionConstraint(constraint=ri+" and "+ro, frame="/map")

            plan_to_poi = self.robot.base.global_planner.getPlan(pos_constraint)

            distance = 10**10 #Just a really really big number for empty plans so they seem far away and are thus unfavorable
            if plan_to_poi:
                distance = len(plan_to_poi)
            print "Distance: %s"%distance
            return distance

        if any(open_POIs):
            best_poi = min(open_POIs, key=distance_to_poi_area)
            placement = geom.PoseStamped(pointstamped=best_poi)
            rospy.loginfo("Placement = {0}".format(placement).replace('\n', ' '))
            return placement
        else:
            rospy.logerr("Could not find an empty spot")
            return None

    def determinePointsOfInterest(self, e):

        points = []

        x = e.pose.position.x
        y = e.pose.position.y

        if len(e.convex_hull) == 0:
            rospy.logerr('Entity: {0} has an empty convex hull'.format(e.id))
            return []

        ''' Convert convex hull to map frame '''
        center_pose = poseMsgToKdlFrame(e.pose)
        ch = []
        for point in e.convex_hull:
            p = pointMsgToKdlVector(point)
            p = center_pose * p
            ch.append(p)

        ''' Loop over hulls '''
        ch.append(ch[0])
        for i in xrange(len(ch) - 1):
                dx = ch[i+1].x() - ch[i].x()
                dy = ch[i+1].y() - ch[i].y()
                length = math.hypot(dx, dy)

                d = self._edge_distance
                while d < (length-self._edge_distance):

                    ''' Point on edge '''
                    xs = ch[i].x() + d/length*dx
                    ys = ch[i].y() + d/length*dy

                    ''' Shift point inwards and fill message'''
                    ps = geom.PointStamped()
                    ps.header.frame_id = "/map"
                    ps.point.x = xs - dy/length * self._edge_distance
                    ps.point.y = ys + dx/length * self._edge_distance
                    ps.point.z = e.pose.position.z + e.z_max
                    points.append(ps)

                    # ToDo: check if still within hull???
                    d += self._spacing

        return points

class PlaceGrabbed(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot

    def execute(self, userdata):

        #item_id = str(self.robot.reasoner.query_first_answer("grabbed_item('item_id',A)"))
        #print "item_id = ", item_id
        print "item_id at place grabbed = ", ITEM
        place_location = str(self.robot.reasoner.query_first_answer("action_info('3','3_place_location',A)"))
        print "place_location = ", place_location

        place_pose_ent = EdEntityDesignator(self.robot,id=place_location)
        place_position = EmptySpotDesignator(self.robot, place_pose_ent)
        arm_with_item_designator = ArmDesignator(self.robot.arms, self.robot.leftArm)

        placestate = states.Place(self.robot, item_to_place=EdEntityDesignator(self.robot,id=ITEM), place_pose=place_position ,arm=arm_with_item_designator)
        result = placestate.execute()

        if result == 'done':
            return 'succeeded'
        else:
            return 'failed'

        return 'failed'




########################
##### STATEMACHINE #####
########################


def setup_statemachine(robot):

    robot.reasoner.load_database("challenge_gpsr","prolog/prolog_data.pl")
    robot.reasoner.query("retractall(current_action(_))")
    robot.reasoner.query("retractall(action_info(_,_,_))")

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    empty_arm_designator = UnoccupiedArmDesignator(robot.arms, robot.leftArm)
    arm_with_item_designator = ArmDesignator(robot.arms, robot.arms['left'])

    with sm:

        ######################################################
        ##################### INITIALIZE #####################             
        ######################################################

        smach.StateMachine.add("INIT_WM",
                               InitializeWorldModel(robot), 
                               transitions={'done'                      :'START_CHALLENGE_ROBUST'})

        # Start challenge via StartChallengeRobust
        smach.StateMachine.add( "START_CHALLENGE_ROBUST",
                                    states.StartChallengeRobust(robot, data.starting_point, use_entry_points = True),
                                    transitions={   "Done":"GO_TO_MEETING_WAYPOINT",
                                                    "Aborted":"GO_TO_MEETING_WAYPOINT",
                                                    "Failed":"GO_TO_MEETING_WAYPOINT"})   # There is no transition to Failed in StartChallengeRobust (28 May)


        smach.StateMachine.add('GO_TO_MEETING_WAYPOINT',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=data.meeting_point), radius=0.2),
                                    transitions={   'arrived':'INTRODUCE_SHORT',
                                                    'unreachable':'GO_TO_MEETING_WAYPOINT_BACKUP',
                                                    'goal_not_defined':'GO_TO_MEETING_WAYPOINT_BACKUP'})

        smach.StateMachine.add('GO_TO_MEETING_WAYPOINT_BACKUP',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=data.meeting_point), radius=0.6),
                                    transitions={   'arrived':'INTRODUCE_SHORT',
                                                    'unreachable':'INTRODUCE_SHORT_FAILED',
                                                    'goal_not_defined':'INTRODUCE_SHORT_FAILED'})

        ######################################################
        #################### INSTRUCTIONS ####################             
        ######################################################


        smach.StateMachine.add("INTRODUCE_SHORT",
                               states.Say(robot,"Hi! I will just wait here and wonder if I can do something for you", block=False),
                               transitions={'spoken':'ASK_ACTION'})

        smach.StateMachine.add("INTRODUCE_SHORT_FAILED",
                               states.Say(robot,"Hi! I could not reach the meeting point, but I will just wait here and wonder if I can do something for you", block=False),
                               transitions={'spoken':'ASK_ACTION'})

        smach.StateMachine.add("ASK_ACTION",
                                Ask_action(robot),
                                transitions={'done':'QUERY_SPECIFIC_ACTION',
                                             'failed':'ASK_ACTION'})

        smach.StateMachine.add("QUERY_SPECIFIC_ACTION",
                                Query_specific_action(robot),
                                transitions={   'navigate_room':'1_ACTION_NAVIGATE_TO_ROOM',
                                                'navigate_location':'1_ACTION_NAVIGATE_TO_LOCATION',
                                                'take_object_loc':'2_INSPECT_AND_GRAB', #amigo should be at a location and now try to search for object
                                                'look_object_loc':'2_FIND_ITEM', # If you are in a room, amigo should explore all locations in room.
                                                'find_person':'2_FIND_PERSON',
                                                #'count_objects':'FINISHED_TASK',
                                                'answer_question':'3_SAY_QUESTION_1',
                                                'answer_special':'3_SPEAK_SPECIAL',
                                                'return_to_operator':'3_NAVIGATE_TO_OPERATOR',
                                                'return_to_person':'3_ACTION_NAVIGATE_TO_ROOM',
                                                'place_item':'3_NAV_TO_LOC_PLACE',
                                                'no_action':'SAY_GO_TO_EXIT'})


        #################################
        ######### ACTION PART 1 #########
        #################################

        ## Navigate to specific location

        smach.StateMachine.add('1_ACTION_NAVIGATE_TO_LOCATION',
                                    states.NavigateToSymbolic(robot, 
                                        {EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_location',A)")) : "in_front_of" }, 
                                        EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_location',A)"))),
                                    transitions={   'arrived'           :   '1_LOOKAT_LOCATION',
                                                    'unreachable'       :   '1_ACTION_NAVIGATE_TO_LOCATION_RETRY',
                                                    'goal_not_defined'  :   '1_ACTION_NAVIGATE_TO_LOCATION_RETRY'})

        smach.StateMachine.add('1_ACTION_NAVIGATE_TO_LOCATION_RETRY',
                                    states.NavigateToSymbolic(robot, 
                                        {EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_location',A)")) : "in_front_of" }, 
                                        EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_location',A)"))),
                                    transitions={   'arrived'           :   '1_LOOKAT_LOCATION',
                                                    'unreachable'       :   'SAY_NOT_ARRIVED',
                                                    'goal_not_defined'  :   'SAY_NOT_ARRIVED'})


        # smach.StateMachine.add('1_ACTION_NAVIGATE_TO_LOCATION',
        #                             states.NavigateToObserve(robot, EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_location',A)")), radius=0.5),
        #                             transitions={   'arrived':'1_LOOKAT_LOCATION',
        #                                             'unreachable':'1_ACTION_NAVIGATE_TO_LOCATION_RETRY',
        #                                             'goal_not_defined':'1_ACTION_NAVIGATE_TO_LOCATION_RETRY'})

        # smach.StateMachine.add('1_ACTION_NAVIGATE_TO_LOCATION_RETRY',
        #                             states.NavigateToObserve(robot, EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_location',A)")), radius=0.8),
        #                             transitions={   'arrived':'1_LOOKAT_LOCATION',
        #                                             'unreachable':'SAY_NOT_ARRIVED',
        #                                             'goal_not_defined':'SAY_NOT_ARRIVED'})
        
        # smach.StateMachine.add( "1_LOOKAT_LOCATION",
        #                              states.LookAtEntity(robot, EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_location',A)")), waittime=5.0),
        #                              transitions={  'succeeded'         :'FINISHED_TASK',
        #                                             'failed'            :'FINISHED_TASK'})

        smach.StateMachine.add( "1_LOOKAT_LOCATION",
                                         states.LookOnTopOfEntity(robot, EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_location',A)")), waittime=5.0),
                                     transitions={  'succeeded'         :'FINISHED_TASK',
                                                    'failed'            :'FINISHED_TASK'})

        ## Navigate to a room

        smach.StateMachine.add('1_ACTION_NAVIGATE_TO_ROOM',
                                states.NavigateToSymbolic(robot, 
                                    {EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_locations_rooms',A)")) : "in" }, 
                                    EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_locations_rooms',A)"))),
                                transitions={   'arrived'           :   'FINISHED_TASK',
                                                'unreachable'       :   '1_ACTION_NAVIGATE_TO_ROOM_RETRY',
                                                'goal_not_defined'  :   '1_ACTION_NAVIGATE_TO_ROOM_RETRY'})

        smach.StateMachine.add('1_ACTION_NAVIGATE_TO_ROOM_RETRY',
                                states.NavigateToSymbolic(robot, 
                                    {EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_locations_rooms',A)")) : "in" }, 
                                    EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_locations_rooms',A)"))),
                                transitions={   'arrived'           :   'FINISHED_TASK',
                                                'unreachable'       :   'SAY_NOT_ARRIVED',
                                                'goal_not_defined'  :   'SAY_NOT_ARRIVED'})  

        smach.StateMachine.add( 'SAY_NOT_ARRIVED',
                                states.Say(robot, ["I have not arrived at the desired location, I'm sorry."], block=True),
                                transitions={'spoken':'FINISHED_TASK'})

        #################################
        ######### ACTION PART 2 #########
        #################################

        # grab_designator_locked = LockingDesignator(ObjectGraspDesignator(robot))
        # smach.StateMachine.add( "2_LOCK_GRAB_ITEM",
        #                             states.LockDesignator(grab_designator_locked), #TODO maybe: unlock this if you ever want to pickup something else
        #                             transitions={   'locked'              :'2_GRAB_ITEM'})

        smach.StateMachine.add( "2_INSPECT_AND_GRAB",
                                    InspectLocationAndGrab(robot),
                                    transitions={   'succeeded'              :'FINISHED_TASK',
                                                    'failed'                 :'2_SAY_NOT_GRABBED'})

        smach.StateMachine.add( '2_SAY_NOT_GRABBED',
                                states.Say(robot, ["I was not able to grasp the item, I'm sorry."], block=False),
                                transitions={'spoken':'SAY_GO_TO_EXIT'})

        # smach.StateMachine.add( "2_GRAB_ITEM",
        #                             states.Grab(robot, ObjectGraspDesignator(robot), empty_arm_designator),
        #                             transitions={   'done'              :'FINISHED_TASK',
        #                                             'failed'            :'FINISHED_TASK'})

        smach.StateMachine.add( "2_FIND_ITEM",
                                    FindObjectInRoom(robot, QueryFirstAnswerDesignator(robot, "action_info('2','2_look_object_loc',_,A)"),QueryFirstAnswerDesignator(robot, "action_info('2','2_look_object_loc',A,_)")),
                                    transitions={   'Object_found'      :'FINISHED_TASK',
                                                    'Object_not_found'  :'2_FIND_ITEM',
                                                    'No_locations'      :'FINISHED_TASK',
                                                    'Failed'            :'FINISHED_TASK'})

        smach.StateMachine.add( "2_FIND_PERSON",
                                    FindAndGoToPerson(robot, QueryFirstAnswerDesignator(robot, "action_info('2','2_look_person_loc',A)")),
                                    transitions={   'Found'              :'FINISHED_TASK',
                                                    'Not_found'           :'FINISHED_TASK',
                                                    'Failed'            :'FINISHED_TASK'})
        

        #################################
        ######### ACTION PART 3 #########
        #################################

        ###### ACTION ANSWER QUESTION ######


        smach.StateMachine.add('3_SAY_QUESTION_1', 
                                states.Say(robot, "What question do you have for me?", block=True), 
                                transitions={ 'spoken' :'3_HEAR_QUESTION_1'})

        smach.StateMachine.add('3_HEAR_QUESTION_1', 
                                HearQuestion(robot), 
                                transitions={ 'answered' :'SAY_GO_TO_EXIT',
                                              'error':'3_SAY_QUESTION_2',
                                              'failed':'3_SAY_QUESTION_2'})

        smach.StateMachine.add('3_SAY_QUESTION_2', 
                                states.Say(robot, "Could you repeat your question?", block=True), 
                                transitions={ 'spoken' :'3_HEAR_QUESTION_2'})

        smach.StateMachine.add('3_HEAR_QUESTION_2', 
                                HearQuestion(robot), 
                                transitions={ 'answered' :'FINISHED_TASK',
                                              'error':'FINISHED_TASK',
                                              'failed':'FINISHED_TASK'})

        ###### ACTION SPEAK SPECIAL ######


        smach.StateMachine.add('3_SPEAK_SPECIAL', 
                                SpeakSpecial(robot), 
                                transitions={ 'answered':'FINISHED_TASK',
                                              'failed':'FINISHED_TASK'})

        ###### ACTION HANDOVER OBJECT TO OPERATOR ######

        smach.StateMachine.add('3_NAVIGATE_TO_OPERATOR',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=data.meeting_point), radius=0.2),
                                    transitions={   'arrived':'3_SAY_HANDOVER',
                                                    'unreachable':'3_NAVIGATE_TO_OPERATOR_RETRY',
                                                    'goal_not_defined':'3_NAVIGATE_TO_OPERATOR_RETRY'})

        smach.StateMachine.add('3_NAVIGATE_TO_OPERATOR_RETRY',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=data.meeting_point), radius=0.6),
                                    transitions={   'arrived':'3_SAY_HANDOVER',
                                                    'unreachable':'3_SAY_NOT_ARRIVED',
                                                    'goal_not_defined':'3_SAY_NOT_ARRIVED'})

        smach.StateMachine.add( '3_SAY_HANDOVER',
                                states.Say(robot, ["Here it is!"], block=True),
                                transitions={'spoken':'3_HANDOVER_TO_OPERATOR'})

        smach.StateMachine.add( '3_SAY_NOT_ARRIVED',
                                states.Say(robot, ["I have not arrived at the desired location, I'm sorry, but please take the item from me."], block=True),
                                transitions={'spoken':'3_HANDOVER_TO_OPERATOR'})


        smach.StateMachine.add('3_HANDOVER_TO_OPERATOR',
                                   states.HandoverToHuman(robot, arm_with_item_designator),
                                   transitions={   'succeeded'          :'3_RESET_ARMS',
                                                    'failed'            :'3_RESET_ARMS'})

        smach.StateMachine.add("3_RESET_ARMS",
                                    states.ResetArms(robot),
                                    transitions={   'done'            :'FINISHED_TASK'})


        ###### ACTION PLACE OBJECT ON LOCATION ######


        smach.StateMachine.add('3_NAV_TO_LOC_PLACE',
                                    states.NavigateToSymbolic(robot, 
                                        {EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('3','3_place_location',A)")) : "in_front_of" }, 
                                        EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('3','3_place_location',A)"))),
                                    transitions={   'arrived'           :   '3_PLACE_ITEM', #3_PLACE_ITEM
                                                    'unreachable'       :   '3_PLACE_ITEM',
                                                    'goal_not_defined'  :   '3_PLACE_ITEM'})

        # place_loc_designator = PlaceLocDesignator(robot)

        # # smach.StateMachine.add( "3_PLACE_ITEM",
        # #                             states.Place(robot, item_to_place=grab_designator_locked, place_pose=place_loc_designator, arm=arm_with_item_designator),
        # #                             transitions={   'done'              :'FINISHED_TASK',
        # #                                             'failed'            :'3_SAY_FAILED_PLACING'})

        smach.StateMachine.add( "3_PLACE_ITEM",
                                     PlaceGrabbed(robot),
                                     transitions={   'succeeded'              :'FINISHED_TASK',
                                                     'failed'            :'3_SAY_FAILED_PLACING'})

        smach.StateMachine.add( '3_SAY_FAILED_PLACING',
                                states.Say(robot, ["I was not able to place the item. I'm sorry, but please take the item from me."], block=True),
                                transitions={'spoken':'3_HANDOVER_TO_OPERATOR'})

        ###### ACTION HANDOVER OBJECT TO PERSON IN ROOM ######

        smach.StateMachine.add('3_ACTION_NAVIGATE_TO_ROOM',
                                states.NavigateToSymbolic(robot, 
                                    {EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('3','3_room',A)")) : "in" }, 
                                    EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('3','3_room',A)"))),
                                transitions={   'arrived'           :   '3_FIND_PERSON',
                                                'unreachable'       :   '3_FIND_PERSON',
                                                'goal_not_defined'  :   '3_FIND_PERSON'})

        smach.StateMachine.add( "3_FIND_PERSON",
                                    FindAndGoToPerson(robot, QueryFirstAnswerDesignator(robot, "action_info('3','3_room',A)")),
                                    transitions={   'Found'             :'3_SAY_HANDOVER',
                                                    'Not_found'         :'3_SAY_NOT_ARRIVED',
                                                    'Failed'            :'FINISHED_TASK'})


        ###############################
        ######### FOR TESTING #########
        ###############################

        smach.StateMachine.add('GO_TO_INITIAL_POINT',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=data.meeting_point), radius=0.2),
                                    transitions={   'arrived':'FINISHED_TASK',
                                                    'unreachable':'GO_TO_INITIAL_POINT_BACKUP',
                                                    'goal_not_defined':'GO_TO_INITIAL_POINT_BACKUP'})

        smach.StateMachine.add('GO_TO_INITIAL_POINT_BACKUP',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=data.meeting_point), radius=0.6),
                                    transitions={   'arrived':'FINISHED_TASK',
                                                    'unreachable':'FINISHED_TASK',
                                                    'goal_not_defined':'FINISHED_TASK'})

        ###################################
        ######### CHECK NEXT TASK #########
        ###################################

        smach.StateMachine.add("FINISHED_TASK",
                                Finished_goal(robot),
                                transitions={'new_task':'QUERY_SPECIFIC_ACTION',
                                              'tasks_completed':'SAY_GO_TO_EXIT'})

        ###############################
        ######### ACTION EXIT #########
        ###############################

        smach.StateMachine.add( 'SAY_GO_TO_EXIT',
                                states.Say(robot, ["I will go to the exit now."], block=False),
                                transitions={'spoken':'GO_TO_EXIT'})

        smach.StateMachine.add('GO_TO_EXIT',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id=data.gpsr_exit), radius = 0.4),
                                    transitions={   'arrived':'SAY_GOODBYE',
                                                    'unreachable':'SAY_GOODBYE',
                                                    'goal_not_defined':'SAY_GOODBYE'})

        smach.StateMachine.add( 'SAY_GOODBYE',
                                states.Say(robot, ["Goodbye"], block=True),
                                transitions={'spoken':'Done'})

    return sm



def test_find_person(robot,room):    
    findperson = FindAndGoToPerson(robot, Designator(room))
    findperson.execute(None)

if __name__ == "__main__":
    rospy.init_node('gpsr_exec')
    rospy.loginfo("----------------------------------------------------------")
    rospy.loginfo("----------------------- GPSR 2015 ------------------------")
    rospy.loginfo("----------------------------------------------------------")
    
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
        ROBOT_NAME_SPECIAL = robot_name
    else:
        print "[CHALLENGE GPSR] Please provide robot name as argument."
        exit(1)

    rospy.sleep(1)
    states.util.startup(setup_statemachine, robot_name=robot_name)
