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
from robot_smach_states import Grab

#import data
from robocup_knowledge import load_knowledge
data = load_knowledge('challenge_gpsr')
#common_kb = load_knowledge('challenge_gpsr')

###########################
# Created by: Erik Geerts #
###########################

#######################
##### TODO LIST!! #####
#### NA EINDHOVEN  ####
#######################
# - dropoff points in eindhoven definieren.
# - remove timeout of 5 minutes -> DID YOU SAY SOMETHING, IN ANY CASE, I DID NOT HEAR YOU!



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

        res = self.robot.ears.recognize(spec=data.spec, choices=data.choices, time_out = rospy.Duration(30))
        self.robot.head.cancel_goal()
        if not res:
            self.robot.speech.speak("My ears are not working properly, can i get a restart?.")
            return "failed"
        try:
            if res.result:
                say_result = self.replace_word(res.result,"me","you")
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
        smach.State.__init__(self, outcomes=["navigate_room", "navigate_location", "grasp_object", "answer_question" "count_objects", "find_person", "test"]) #outcomes=["action_get", "action_transport","action_point","action_find","action_navigate","action_leave","error"])
        self.robot = robot

    def execute(self, userdata):

        action_nr = self.robot.reasoner.query("current_action(A)")
        print action_nr
        if action_nr:
            #print action_nr[0]['A']
            current_action = int(action_nr[0]['A'])+1

        else:
            self.robot.reasoner.assertz("current_action('1')")
            current_action = int("1")
        
        #rospy.logwarn("HACK TO GO DIRECTLY TO ACTION 2!")
        #current_action = "2"

        print current_action

        if current_action == 1:
            action_1 = self.robot.reasoner.query("action_info('1',A,B)")
            for x in action_1:
                for choice_key, choice_value in x.iteritems():
                    print "action 1: choice_key = ", str(choice_key)
                    print "action 1: choice_value = ", str(choice_value)

                    # if str(choice_value) == "1_locations_rooms":
                    #     room=str(self.robot.reasoner.query_first_answer("action_info('1','1_locations_rooms',A)"))
                    #     self.robot.reasoner.query("retractall(action_info('1','1_locations_rooms',A))")
                    #     self.robot.reasoner.assertz("action_info('1','1_locations_rooms',"+"room_"+str(room)+")")
                    #     print "a"
                    #     print self.robot.reasoner.query_first_answer("action_info('1','1_locations_rooms',A)")
                    #     return "navigate_room"

                    # if (str(choice_value) == "1_locations_aeuoi") or (str(choice_value) == "1_locations_rest") :
                    #     location=str(self.robot.reasoner.query_first_answer("action_info('1','"+str(choice_value)+"',A)"))
                    #     self.robot.reasoner.query("retractall(action_info('1','"+str(choice_value)+"',A))")
                    #     self.robot.reasoner.assertz("action_info('1','1_location','"+str(location)+"')")
                    #     print "b"
                    #     print self.robot.reasoner.query_first_answer("action_info('1','1_location',A)")
                    #     return "navigate_location"

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
                        self.robot.reasoner.assertz("action_info('1','1_location','"+"gpsr_"+str(location)+"')")
                        print self.robot.reasoner.query_first_answer("action_info('1','1_location',A)")
                        return "navigate_location"


        elif current_action == 2:
            action_2 = self.robot.reasoner.query("action_info('2',A,B)")
            for x in action_2:
                for choice_key, choice_value in x.iteritems():
                    
                    print "action 2: choice_key = ", str(choice_key)
                    print "action 2: choice_value = ", str(choice_value)

                    if (str(choice_value) == "2_action") or (str(choice_value) == "2_action_count") or (str(choice_value) == "2_action_person"):
                        action_to_be_executed = str(self.robot.reasoner.query_first_answer("action_info('2','"+str(choice_value)+"',B)"))
                        self.robot.reasoner.query("retractall(action_info('2','"+str(choice_value)+"',A))")
                        self.robot.reasoner.assertz("action_info('2','2_action','"+str(action_to_be_executed)+"')")


                    if (str(choice_value) == "2_objects_plural") or (str(choice_value) == "2_objects_types_plural") or (str(choice_value) == "2_objects_aeuoi") or (str(choice_value) == "2_objects_rest_singular") :
                        object_action=str(self.robot.reasoner.query_first_answer("action_info('2','"+str(choice_value)+"',A)"))
                        self.robot.reasoner.query("retractall(action_info('2','"+str(choice_value)+"',A))")
                        self.robot.reasoner.assertz("action_info('2','2_object','"+str(object_action)+"')")

                    if (str(choice_value) == "2_person_men") or (str(choice_value) == "2_person_women") or (str(choice_value) == "2_person_me") or (str(choice_value) == "2_person_general") :
                        person_action=str(self.robot.reasoner.query_first_answer("action_info('2','"+str(choice_value)+"',A)"))
                        self.robot.reasoner.query("retractall(action_info('2','"+str(choice_value)+"',A))")
                        self.robot.reasoner.assertz("action_info('2','2_person','"+str(person_action)+"')")
            
            planned_action_2 = self.robot.reasoner.query_first_answer("action_info('2','2_action',A)")
            planned_person_2 = self.robot.reasoner.query_first_answer("action_info('2','2_person',A)")
            planned_object_2 = self.robot.reasoner.query_first_answer("action_info('2','2_object',A)")

            print "planned_action_2 = ", planned_action_2
            print "planned_person_2 = ", planned_person_2
            print "planned_object_2 = ", planned_object_2

            if planned_action_2 == "count":
                return "count_objects"

            elif planned_action_2 == "find":
                if planned_person_2:
                    return "find_person"

                elif planned_object_2:
                    return "grasp_object"
            
            elif (planned_action_2 == "grasp") or (planned_action_2 == "get") or (planned_action_2 == "take"):
                return "grasp_object"

            


        elif current_action == 3:
            print self.robot.reasoner.query("action_info('3',A,B)")



        # print answers[0]

        # print answers[0]['A']

        # answers = self.robot.reasoner.query("retractall(action_info(_,_))")
        # print "test"
        # print self.robot.reasoner.query("action_info(A,B)")
        return "test"

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
            print "finish"
            print int(action_nr[0]['A'])
            print str(int(action_nr[0]['A'])+1)
            self.robot.reasoner.query("retractall(current_action(_))")
            self.robot.reasoner.assertz("current_action("+str(int(action_nr[0]['A'])+1)+")")
            action_nr = self.robot.reasoner.query("current_action('A')")
            print "query after = ", action_nr

        else:
            print "[gpsr] current_action not found. This should not happen."

        #rospy.sleep(2)
        return "new_task"

class HearQuestion(smach.State):
    def __init__(self, robot, time_out=rospy.Duration(10)):
        smach.State.__init__(self, outcomes=["answered", "error", "failed"])
        self.robot = robot
        self.time_out = time_out

    def execute(self, userdata):
        self.robot.head.look_at_standing_person()

        res = self.robot.ears.recognize(spec=data.spec_question, choices=data.choices_question, time_out=self.time_out)

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

########################
##### STATEMACHINE #####
########################

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

class ObjectTypeDesignator(Designator):
    def __init__(self, robot):
        super(ObjectTypeDesignator, self).__init__(resolve_type=ed.msg.EntityInfo)
        self.robot = robot

    def resolve(self):
        object_type = str(self.robot.reasoner.query_first_answer("action_info('2','2_object',A)"))               
        ## FOR TESTING:
        #object_type = "cola"
        has_type = lambda entity: entity.type == object_type
        location = str(self.robot.reasoner.query_first_answer("action_info('1','1_location',A)"))
        ## FOR TESTING:
        #location = "hallway_couch"
        grab_item_designator = EdEntityDesignator(self.robot, center_point=geom.PointStamped(frame_id="/"+location), radius=2.0,
                                                                criteriafuncs=[has_type], debug=False)

        return grab_item_designator.resolve()

# class PersonTypeDesignator(Designator):
#     def __init__(self, robot):
#         super(ObjectTypeDesignator, self).__init__(resolve_type=ed.msg.EntityInfo)
#         self.robot = robot

#     def resolve(self):
#         has_type = lambda entity: entity.type == "person"
#         location = str(self.robot.reasoner.query_first_answer("action_info('1','1_locations_rooms',A)"))
#         ## FOR TESTING:
#         #location = "hallway_couch"
#         grab_item_designator = EdEntityDesignator(self.robot, center_point=geom.PointStamped(frame_id="/"+location), radius=2.0,
#                                                                 criteriafuncs=[has_type], debug=False)

#         return grab_item_designator.resolve()

def setup_statemachine(robot):

    robot.reasoner.load_database("challenge_gpsr","prolog/prolog_data.pl")
    robot.reasoner.query("retractall(current_action(_))")
    robot.reasoner.query("retractall(action_info(_,_,_))")

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:

        ######################################################
        ##################### INITIALIZE #####################             
        ######################################################

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
                               states.Say(robot,"Hi! I could not reach the meeint point, but I will just wait here and wonder if I can do something for you", block=False),
                               transitions={'spoken':'ASK_ACTION'})

        smach.StateMachine.add("ASK_ACTION",
                                Ask_action(robot),
                                transitions={'done':'QUERY_SPECIFIC_ACTION',
                                             'failed':'ASK_ACTION'})

        smach.StateMachine.add("QUERY_SPECIFIC_ACTION",
                                Query_specific_action(robot),
                                transitions={   'navigate_room':'1_ACTION_NAVIGATE_TO_ROOM',
                                                'navigate_location':'1_ACTION_NAVIGATE_TO_LOCATION',
                                                'grasp_object':'GRAB_ITEM', # TODO: assuming for now that you navigate directly to a location. 
                                                                                      # If you are in a room, amigo should explore all locations in room.
                                                'find_person':'FINISHED_TASK',
                                                'count_objects':'FINISHED_TASK',
                                                'answer_question':'SAY_QUESTION_1',
                                                'test':'GO_TO_INITIAL_POINT'})

        #                                         # 'action_get':'SUB_SM_GET',
        #                                         # 'action_transport':'SUB_SM_TRANSPORT',
        #                                         # 'action_point':'SUB_SM_POINT',
        #                                         # 'action_find':'SUB_SM_FIND',
        #                                         # 'action_navigate':'SUB_SM_NAVIGATE',
        #                                         # 'action_leave':'SUB_SM_LEAVE',
        #                                         # 'error':'FINISHED_TASK'})

        # #In case goal is given via speech interpreter:
        # smach.StateMachine.add("FAILED_TASK",
        #                         Failed_goal(robot),
        #                         transitions={'new_task':'RESET_REASONER'})


        smach.StateMachine.add('1_ACTION_NAVIGATE_TO_LOCATION',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_location',A)")), radius=0.3),
                                    transitions={   'arrived':'SAY_ARRIVED',
                                                    'unreachable':'SAY_NOT_ARRIVED',
                                                    'goal_not_defined':'SAY_NOT_ARRIVED'})


        # smach.StateMachine.add('1_ACTION_NAVIGATE_TO_LOCATION',
        #                         states.NavigateToSymbolic(robot, 
        #                             {EdEntityDesignator(robot, id="hallway_couch") : "in_front_of" }, 
        #                             EdEntityDesignator(robot, id="hallway_couch")),
        #                         transitions={   'arrived'           :   'QUERY_SPECIFIC_ACTION',
        #                                         'unreachable'       :   'QUERY_SPECIFIC_ACTION',
        #                                         'goal_not_defined'  :   'QUERY_SPECIFIC_ACTION'})

        smach.StateMachine.add('1_ACTION_NAVIGATE_TO_ROOM',
                                states.NavigateToSymbolic(robot, 
                                    {EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_locations_rooms',A)")) : "in" }, 
                                    EdEntityDesignator(robot, id_designator=QueryFirstAnswerDesignator(robot, "action_info('1','1_locations_rooms',A)"))),
                                transitions={   'arrived'           :   'SAY_ARRIVED',
                                                'unreachable'       :   'SAY_NOT_ARRIVED',
                                                'goal_not_defined'  :   'SAY_NOT_ARRIVED'}) 

        smach.StateMachine.add( "GRAB_ITEM",
                                    Grab(robot, ObjectTypeDesignator(robot), UnoccupiedArmDesignator(robot.arms, robot.leftArm)),
                                    transitions={   'done'              :'GO_TO_INITIAL_POINT',
                                                    'failed'            :'GO_TO_INITIAL_POINT'})

        smach.StateMachine.add( 'SAY_ARRIVED',
                                states.Say(robot, ["I have arrived at the desired location."], block=True),
                                transitions={'spoken':'SAY_GO_BACK'})

        smach.StateMachine.add( 'SAY_NOT_ARRIVED',
                                states.Say(robot, ["I have not arrived at the desired location, I'm sorry."], block=True),
                                transitions={'spoken':'SAY_GO_BACK'})

        smach.StateMachine.add( 'SAY_GO_BACK',
                                states.Say(robot, ["I can not do the last two actions, I will go to the exit now."], block=False),
                                transitions={'spoken':'GO_TO_EXIT'})

        ####################################
        ###### ACTION ANSWER QUESTION ######
        ####################################

        smach.StateMachine.add('SAY_QUESTION_1', 
                                states.Say(robot, "What can I do for you?", block=True), 
                                transitions={ 'spoken' :'HEAR_QUESTION_1'})

        smach.StateMachine.add('HEAR_QUESTION_1', 
                                HearQuestion(robot), 
                                transitions={ 'answered' :'SAY_2',
                                              'error':'SAY_QUESTION_2',
                                              'failed':'SAY_QUESTION_2'})

        smach.StateMachine.add('SAY_QUESTION_2', 
                                states.Say(robot, "Could you repeat your question?", block=True), 
                                transitions={ 'spoken' :'HEAR_QUESTION_2'})

        smach.StateMachine.add('HEAR_QUESTION_2', 
                                HearQuestion(robot), 
                                transitions={ 'answered' :'SAY_2',
                                              'error':..,
                                              'failed':})





        smach.StateMachine.add('GO_TO_EXIT',
                                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id='exit_gpsr'), radius = 0.4),
                                    transitions={   'arrived':'SAY_GOODBYE',
                                                    'unreachable':'SAY_GOODBYE',
                                                    'goal_not_defined':'SAY_GOODBYE'})

        smach.StateMachine.add( 'SAY_GOODBYE',
                                states.Say(robot, ["Goodbye"], block=True),
                                transitions={'spoken':'Done'})

        # smach.StateMachine.add( "FIND_PERSON",
        #                             Grab(robot, ObjectTypeDesignator(robot), UnoccupiedArmDesignator(robot.arms, robot.leftArm)),
        #                             transitions={   'done'              :'GO_TO_INITIAL_POINT',
        #                                             'failed'            :'GO_TO_INITIAL_POINT'})


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

        smach.StateMachine.add("FINISHED_TASK",
                                Finished_goal(robot),
                                transitions={'new_task':'ASK_ACTION',
                                              'tasks_completed':'GO_TO_EXIT'})

        # smach.StateMachine.add('GO_TO_EXIT',
        #                             states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="exit"), radius=0.7),
        #                             transitions={   'arrived':'Done',
        #                                             'unreachable':'Done',
        #                                             'goal_not_defined':'Done'})

    return sm

if __name__ == "__main__":
    rospy.init_node('gpsr_exec')
    rospy.loginfo("----------------------------------------------------------")
    rospy.loginfo("----------------------- GPSR 2015 ------------------------")
    rospy.loginfo("----------------------------------------------------------")
    
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE GPSR] Please provide robot name as argument."
        exit(1)

    rospy.sleep(5)
    states.util.startup(setup_statemachine, robot_name=robot_name)
