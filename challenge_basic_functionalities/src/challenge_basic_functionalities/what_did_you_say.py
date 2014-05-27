#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_basic_functionalities')
import rospy
#import robot_parts.speech
from std_msgs.msg import String
import geometry_msgs
import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states
import robot_skills.util.msg_constructors as msgs

from robot_smach_states.util.startup import startup

from speech_interpreter.srv import AskUser # for speech_to_text only

from pein_srvs.srv import StartStopWithROIArray

from psi import *

###########################
# Created by: Erik Geerts #
###########################

# # Hardcoded question room {'livingroom','bedroom' or 'kitchen'}
# room = 'living_room'

class AskQuestions(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self.robot = robot
        self.ask_user_service_questions = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata=None):

        rospy.loginfo("-- Possible questions for now: ----------------------------------------------------------")
        rospy.loginfo("-- What time is it? ---------------------------------------------------------------------")
        rospy.loginfo("-- What is the answer to the ultimate question about life the universe and everything? --")
        rospy.loginfo("-- What is the capital of poland?--------------------------------------------------------")
        rospy.loginfo("-- What is the oldest most widely used drug on earth?------------------------------------")
        rospy.loginfo("-- What is your name?--------------------------------------------------------------------")
        rospy.loginfo("-- What is your team's name?-------------------------------------------------------------")
        rospy.loginfo("-- Which country grows the most potatoes?------------------------------------------------")
        rospy.loginfo("-- Which country grew the first orange?--------------------------------------------------")
        rospy.loginfo("-- Which fish can hold objects in its tail?----------------------------------------------")
        rospy.loginfo("-- How many countries are in europe?-----------------------------------------------------")

        # Here you can define how many times you want to try to listen and want the maximum duration is to listen to operator.
        self.response = self.ask_user_service_questions("questions_grammar", 10 , rospy.Duration(60))

        if self.response.keys[0] == "answer":
            return "succeeded"
        else:
            return "failed"

class WaitForPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['detected','waiting','timed_out','failed'])
        self.robot = robot

        self.counter = 0
    
    def execute(self, userdata=None):

        if self.counter > 1:
            self.robot.head.look_up(tilt_vel=0.75)
            return 'timed_out'

        self.robot.spindle.reset()
        self.robot.head.set_pan_tilt(tilt=-0.2)
        
        if self.counter == 0:
            rospy.sleep(1.5)        

        query_detect_person = Conjunction(Compound("property_expected", "ObjectID", "class_label", "face"),
                                          Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                          Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        self.response_start = self.robot.perception.toggle(['face_segmentation'])

        if self.response_start.error_code == 0:
            rospy.loginfo("Face segmentation has started correctly")
        elif self.response_start.error_code == 1:
            rospy.logerr("Face segmentation failed to start")
            self.robot.head.look_up(tilt_vel=0.75)
            return "failed"

        wait_machine = states.Wait_query_true(self.robot, query_detect_person, 12)
        wait_result = wait_machine.execute()

        rospy.loginfo("Face segmentation will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Face segmentation is stopped")
        elif self.response_stop.error_code == 1:
            self.robot.lights.set_color(1, 0, 0)
            rospy.sleep(0.1)  
            self.robot.lights.set_color(0, 0, 1)
            rospy.loginfo("Failed stopping face segmentation")

        if wait_result == "timed_out":
            self.counter = self.counter + 1
            return "waiting"

        elif wait_result == "query_true":
            answers = self.robot.reasoner.query(query_detect_person)
            possible_locations = [( float(answer["X"]), 
                                    float(answer["Y"]), 
                                    float(answer["Z"])) for answer in answers]
            x,y,z = possible_locations[0]

            if z > 1.5:
                self.robot.spindle.high()
                rospy.logdebug("Spindle should come up now!")

            lookat_point = msgs.PointStamped(x,y,z)
            rospy.loginfo("AMIGO should look at person now. (x = {0}, y = {1}, z = {2})".format(x,y,z))
            self.robot.head.send_goal(lookat_point,timeout=0)

            return 'detected'

class DriveToFindPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["arrived", "failed", "no_waypoint"])

        self.robot = robot

    def execute(self, userdata=None):
        
        self.robot.spindle.reset()
        self.robot.head.reset_position()

        # NAVIGATE TO LOCATION TO DETECT PEOPLE

        self.robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X"))) 

        navigate_room = Conjunction(  Compound("=", "Waypoint", Compound("look_person", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint")),
                                                 Compound("not", Compound("unreachable", "Waypoint")))

        goal_answers = self.robot.reasoner.query(navigate_room)

        print "goal answers = ", goal_answers

        self.robot.reasoner.query(Compound("assert", Compound("current_exploration_target", "Waypoint"))) 

        if not goal_answers:
            return "no_waypoint"

        goal_answer = goal_answers[0]
        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))

        waypoint_name = goal_answer["Waypoint"]
            

        nav = states.NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        if nav_result == "unreachable" or nav_result == "preempted":
            return "failed"
        else:
            return "arrived"



class CheckForPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["found", "not_found"])

        self.robot = robot

    def execute(self, userdata=None):

        person_query = Conjunction( Compound( "property_expected", "ObjectID", "class_label", "person"),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        person_result = self.robot.reasoner.query(person_query)
        
        if not person_result:
            self.robot.speech.speak("No one here.")
            return "not_found"

        print "person_result = ", person_result

        if len(person_result) > 1:
            self.robot.speech.speak("I see some people!",block=False)
        else:
            self.robot.speech.speak("I found someone!",block=False)

        self.robot.reasoner.assertz(Compound("question_person", Compound("point_3d", Sequence(person_result[0]["X"], person_result[0]["Y"], 1.5))))

        return "found"    


# # this class drives to a point and then checks for persons. asserts that location as visited.
# class LookingForPerson(smach.State):
#     def __init__(self, robot):
#         smach.State.__init__(self, outcomes=["found", "looking", "not_found"])

#         self.robot = robot

#     def execute(self, userdata=None):
        
#         self.robot.spindle.reset()
#         self.robot.head.reset_position()

#         self.robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X"))) 

#         navigate_room = Conjunction(  Compound("=", "Waypoint", Compound("look_person", "W")),
#                                                  Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
#                                                  Compound("not", Compound("visited", "Waypoint")),
#                                                  Compound("not", Compound("unreachable", "Waypoint")))

#         goal_answers = self.robot.reasoner.query(navigate_room)

#         print "goal answers = ", goal_answers

#         self.robot.reasoner.query(Compound("assert", Compound("current_exploration_target", "Waypoint"))) 

#         if not goal_answers:
#             return "not_found"

#         goal_answer = goal_answers[0]
#         goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))

#         waypoint_name = goal_answer["Waypoint"]
            

#         nav = states.NavigateGeneric(self.robot, goal_pose_2d=goal)
#         nav_result = nav.execute()

#         self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

#         if nav_result == "unreachable" or nav_result == "preempted":
#             return "looking"

#         self.robot.head.set_pan_tilt(tilt=0.0)
#         self.robot.spindle.reset()

#         # we made it to the new goal. Let's have a look to see whether we can find the person here
#         #self.robot.speech.speak("Let me see who I can find here...")
#         rospy.sleep(1.5)
        
#         self.response_start = self.robot.perception.toggle(["face_segmentation"])
#         if self.response_start.error_code == 0:
#             rospy.loginfo("Face segmentation has started correctly")
#         elif self.response_start.error_code == 1:
#             rospy.loginfo("Face segmentation failed to start")
#             self.robot.speech.speak("I was not able to start face segmentation.")
#             return 'looking'
#         rospy.sleep(2)

#         rospy.loginfo("Face segmentation will be stopped now")
#         self.response_stop = self.robot.perception.toggle([])
        
#         if self.response_stop.error_code == 0:
#             rospy.loginfo("Face segmentation is stopped")
#         elif self.response_stop.error_code == 1:
#             rospy.loginfo("Failed stopping face segmentation")

#         person_query = Conjunction(  
#                                     Compound( "property_expected", "ObjectID", "class_label", "face"),
#                                     Compound( "property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

#         person_result = self.robot.reasoner.query(person_query)
        
#         print "person_result = ", person_result

#         if not person_result:
#             self.robot.speech.speak("No one here.")
#             return "looking"

#         if len(person_result) > 1:
#             self.robot.speech.speak("I see some people!",block=False)
#         else:
#             self.robot.speech.speak("I found someone!",block=False)

#         self.robot.reasoner.assertz(Compound("question_person", Compound("point_3d", Sequence(person_result[0]["X"], person_result[0]["Y"], person_result[0]["Z"]))))

#         return "found"    


class WhatDidYouSay(smach.StateMachine):

    def __init__(self, robot=None):
        smach.StateMachine.__init__(self, outcomes=["Done"])
        self.robot = robot

        rospy.loginfo("-------------------- WHAT DIT YOU SAY ---------------------")
        rospy.loginfo("-----------------------------------------------------------")
        rospy.loginfo("------------ MAKE SURE YOU ADDED THE SENTENCES ------------")
        rospy.loginfo("-------------- speech_interpreter.pl in map ---------------")
        rospy.loginfo("----------- tue_reasoner/tue_knowledge/prolog -------------")
        rospy.loginfo("-------- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -----------")
        rospy.loginfo("-----------------------------------------------------------")
            
        with self:

            smach.StateMachine.add("SAY_LOOK_EYES",
                                    states.Say(robot,"I will try to find you", block=False),
                                    transitions={'spoken':'DRIVE_TO_FIND_PERSON_LOC'})

            smach.StateMachine.add( "DRIVE_TO_FIND_PERSON_LOC",
                                DriveToFindPerson(robot),
                                transitions={   'arrived':'RESET_HEAD_SPINDLE',
                                                'failed':'DRIVE_TO_FIND_PERSON_LOC',
                                                'no_waypoint':'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE'})

            smach.StateMachine.add("RESET_HEAD_SPINDLE",
                                states.ResetHeadSpindle(robot),
                                transitions={   'done':'RESET_REASONER'})

            smach.StateMachine.add("RESET_REASONER",
                                states.ResetReasoner(robot),
                                transitions={   'done':'PEOPLE_DETECTION'})

            smach.StateMachine.add("PEOPLE_DETECTION",
                                    states.PeopleDetectorTorsoLaser(robot, time=4, room='living_room'),
                                transitions={   'done':'CHECK_FOR_PERSON',
                                                'failed':'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE'})

            smach.StateMachine.add( "CHECK_FOR_PERSON",
                                CheckForPerson(robot),
                                transitions={   'found':'NAVIGATE_TO_PERSON',
                                                'not_found':'DRIVE_TO_FIND_PERSON_LOC'})

            question_person_query = Compound("question_person", Compound("point_3d",Sequence("X","Y","Z")))

            smach.StateMachine.add( "NAVIGATE_TO_PERSON",
                                #states.NavigateGeneric(robot, lookat_query=general_person_query, xy_dist_to_goal_tuple=(1.0,0)),
                                states.NavigateGeneric(robot, lookat_query=question_person_query, xy_dist_to_goal_tuple=(0.8,0)),
                                transitions={   "arrived":"LOOK_AT_PERSON",
                                                "unreachable":'SAY_PERSON_UNREACHABLE',
                                                "preempted":'SAY_PERSON_UNREACHABLE',
                                                "goal_not_defined":'SAY_PERSON_UNREACHABLE'})

            smach.StateMachine.add( "SAY_PERSON_UNREACHABLE",
                                states.Say(robot,"I failed going to the person", block=False),
                                transitions={'spoken':'GO_TO_LAST_EXPLORATION_POINT'})

            query_last_exploration_location = Conjunction(Compound("current_exploration_target", "Waypoint"),
                                                  Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))
            
            smach.StateMachine.add( "GO_TO_LAST_EXPLORATION_POINT",
                                states.NavigateGeneric(robot, goal_query=query_last_exploration_location),
                                transitions={   "arrived":"SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE",
                                                    "unreachable":'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE',
                                                    "preempted":'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE',
                                                    "goal_not_defined":'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE'})

            smach.StateMachine.add('LOOK_AT_PERSON',
                                states.LookAtPoint(robot, lookat_query=question_person_query),
                                transitions={   'looking':'SAY_FIRST_QUESTION',
                                                'no_point_found':'SAY_FIRST_QUESTION',
                                                'abort':'SAY_FIRST_QUESTION'})


            smach.StateMachine.add( "SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE",
                                    states.Say(robot,"I will ask my questions from here, Please step in front of me", block=True),
                                    transitions={'spoken':'WAIT_FOR_PERSON'})



            # smach.StateMachine.add("STEP_IN_FRONT",
            #                         states.Say(robot,"Please step in front of me so that you can ask some questions"),
            #                         transitions={'spoken':'WAIT_FOR_PERSON'})

            smach.StateMachine.add("WAIT_FOR_PERSON",
                                    WaitForPerson(robot),
                                    transitions={'detected':'SAY_FIRST_QUESTION',
                                                 'waiting':'SAY_FIRST_QUESTION',
                                                 'timed_out':'SAY_FIRST_QUESTION',
                                                 'failed':'SAY_FIRST_QUESTION'})

            smach.StateMachine.add("SAY_FIRST_QUESTION",
                                    states.Say(robot,"Hello! What is your first question?"),
                                    transitions={'spoken':'ASK_FIRST_QUESTION'})

            smach.StateMachine.add("ASK_FIRST_QUESTION",
                                    AskQuestions(robot),
                                    transitions={'succeeded':'SAY_SECOND_QUESTION',
                                                 'failed':'ASK_FIRST_QUESTION'})

            smach.StateMachine.add("SAY_SECOND_QUESTION",
                                    states.Say(robot,"What is your second question?"),
                                    transitions={'spoken':'ASK_SECOND_QUESTION'})

            smach.StateMachine.add("ASK_SECOND_QUESTION",
                                    AskQuestions(robot),
                                    transitions={'succeeded':'SAY_THIRD_QUESTION',
                                                 'failed':'ASK_SECOND_QUESTION'})

            smach.StateMachine.add("SAY_THIRD_QUESTION",
                                    states.Say(robot,"What is your third question?"),
                                    transitions={'spoken':'ASK_THIRD_QUESTION'})

            smach.StateMachine.add("ASK_THIRD_QUESTION",
                                    AskQuestions(robot),
                                    transitions={'succeeded':'Done',
                                                 'failed':'ASK_THIRD_QUESTION'})

if __name__ == "__main__":
    rospy.init_node('what_did_you_say_exec')
    startup(WhatDidYouSay)
