#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_cocktail_party')
import rospy
import os
import roslib.packages as p
import smach_ros
import smach
import random

from speech_interpreter.srv import AskUser

from robot_skills.amigo import Amigo

from std_msgs.msg import Empty

from robot_smach_states import *
from psi import Compound, Sequence, Conjunction, Term
import robot_skills.util.msg_constructors as msgs
from geometry_msgs.msg import PoseStamped, Pose

import robot_smach_states.util.transformations as transformations

from pein_srvs.srv import SetObjects


################################# SETUP VARIABLES ######################################


MIN_SIMULTANEOUS_ORDERS = 2
MAX_SIMULTANEOUS_ORDERS = 3
TOTAL_ORDERS = 3


#########################################################################################

class DeleteModels(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["done"])

        self.robot = robot
        self.set_objects = rospy.ServiceProxy('/pein/set_object_models',SetObjects)

    def execute(self, userdata=None):

        folder = p.get_pkg_dir('pein_face_recognition')+'/models'
        #folder = '/path/to/folder'
        for the_file in os.listdir(folder):
            file_path = os.path.join(folder, the_file)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
            except Exception, e:
                print e
        rospy.loginfo("Deleted models from {0}".format(folder))
        return 'done'


#########################################################################################

class DetectWavingPeople(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["detected", "not_detected", "error"])
        
        self.robot = robot

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: DetectWavingPeople\n")

        self.robot.head.reset_position()
        self.robot.speech.speak("Ladies and gentlemen, please wave to call me and place an order.")

        # turn head to one side to start the swipping the room
        self.robot.head.set_pan_tilt(pan=-1.1, tilt=0.0)
        rospy.sleep(3)
        

        # Turn ON Human Tracking
        self.response_start = self.robot.perception.toggle(['human_tracking'])

        if self.response_start.error_code == 0:
            rospy.loginfo("human_tracking turned on")
        elif self.response_start.error_code == 1:
            rospy.loginfo("human_tracking failed to start")
            self.robot.speech.speak("I was not able to start human tracking.")
            return "error"

        # self.robot.head.set_pan_tilt(pan=-1.2, pan_vel=0.1)
        # rospy.sleep(3)
        self.robot.head.set_pan_tilt(pan=0.0, pan_vel=0.1, tilt=0.0)
        rospy.sleep(3)
        
        self.robot.head.set_pan_tilt(pan=1.1, pan_vel=0.1, tilt=0.0)
        rospy.sleep(5)

        # Turn OFF Human Tracking
        self.response_stop = self.robot.perception.toggle([])

        if self.response_stop.error_code == 0:
            rospy.loginfo("human_tracking turned off")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("human_tracking failed to shutdown")
            self.robot.speech.speak("I was not able to stop human tracking.")
            return "error"

        rospy.sleep(1)

        self.robot.head.reset_position()

        # compose person query
        peopleFoundQ = Conjunction( Compound("property_expected", "ObjectID", "class_label", "validated_person"),
                                    Compound("not", Compound("visited", "ObjectID")))

        # get results from the query
        peopleFound = self.robot.reasoner.query(peopleFoundQ)
        
        if peopleFound:
            self.robot.speech.speak("Someone is calling me, I will be with you soon!", block=False)
            return 'detected'
        else:
            self.robot.speech.speak("No one called me. Forever alone.", block=False)
            return 'not_detected'

#########################################################################################

class DetectPeople(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["detected", "not_detected", "error"])
        
        self.robot = robot

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: DetectPeople\n")

        self.robot.head.reset_position()
        
        # turn head to one side to start the swipping the room
        self.robot.head.set_pan_tilt(pan=-0.8, tilt=0.0)
        rospy.sleep(3)
        
        # Turn ON Human Tracking
        self.response_start = self.robot.perception.toggle(['human_tracking'])

        if self.response_start.error_code == 0:
            rospy.loginfo("human_tracking turned on")
        elif self.response_start.error_code == 1:
            rospy.loginfo("human_tracking failed to start")
            self.robot.speech.speak("I was not able to start human tracking.")
            return "error"

        # TODO CAN I USE TIMEOUTS INSTEAD OF SLEEPS?
        self.robot.head.set_pan_tilt(pan=0.0, pan_vel=0.1, tilt=0.0)
        rospy.sleep(3)
        
        self.robot.head.set_pan_tilt(pan=0.8, pan_vel=0.1, tilt=0.0)
        rospy.sleep(5)

        # Turn OFF Human Tracking
        self.response_stop = self.robot.perception.toggle([])

        if self.response_stop.error_code == 0:
            rospy.loginfo("human_tracking turned off")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("human_tracking failed to shutdown")
            self.robot.speech.speak("I was not able to stop human tracking.")
            return "error"

        rospy.sleep(1)
        
        self.robot.head.reset_position()

        # compose person query
        qPeopleFound = Conjunction( Compound("property_expected", "ObjectID", "class_label", "validated_person"),
                                    # Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                    Compound("not", Compound("approached", "ObjectID")))

        # get results from the query
        peopleFoundRes = self.robot.reasoner.query(qPeopleFound)
        
        if peopleFoundRes:
            rospy.loginfo("\t\t[Cocktail Party] Found {0} person(s)\n".format(len(peopleFoundRes)))
            self.robot.speech.speak("I think i saw someone here.", block=False)
            return 'detected'
        else:
            self.robot.speech.speak("There's no one here.", block=False)
            return 'not_detected'

#########################################################################################

class WaitForPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['waiting', 'unknown_person'],
                                input_keys=['waitIndex_in'])
        
        self.robot = robot

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: WaitForPerson\n")

        waitedCount = userdata.waitIndex_in

        if waitedCount == 3:
            # Reset the counter for waiting
            waitedCount = 0;
            self.robot.speech.speak("I was not able to detect a person, assuming someone is in front of me!")
            return "unknown_person"
        else:
            rospy.loginfo("Waited for {0} times!!!".format(waitedCount))

        # reset robo pose
        self.robot.spindle.reset()
        self.robot.reasoner.reset()
        self.robot.head.set_pan_tilt(tilt=-0.2)
        
        self.robot.speech.speak("Ladies and gentlemen, please step in front of me to order your drink.", block=False)

        # prepare query for detected person
        query_detect_person = Conjunction(Compound('property_expected', 'ObjectID', 'class_label', 'human_face'),
                                          Compound('property_expected', 'ObjectID', 'position', Compound('in_front_of', 'amigo')),
                                          Compound('property_expected', 'ObjectID', 'position', Sequence('X','Y','Z')))

        # start face segmentation node
        self.response_start = self.robot.perception.toggle(['human_tracking'])

        if self.response_start.error_code == 0:
            rospy.loginfo("Human Tracking has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Human Tracking failed to start")
            self.robot.speech.speak("I was not able to start Human Tracking.")

            return "waiting"

        # wait until the query detected person is true, or 10 second timeout
        wait_machine = Wait_query_true(self.robot, query_detect_person, 10)
        wait_result = wait_machine.execute()

        # turn off face segmentation
        rospy.loginfo("Human Tracking will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Human Tracking is stopped")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping Human Tracking")

        # if the query timed out...
        if wait_result == "timed_out":
            self.robot.speech.speak("Please, don't keep me waiting.", block=False)
            waitedCount += 1
            return "waiting"
        elif wait_result == "preempted":
            self.robot.speech.speak("Waiting for person was preemted.", block=False)
            return "waiting"
        # if the query succeeded
        elif wait_result == "query_true":
            # answers = self.robot.reasoner.query(query_detect_person)
            # # get the person location
            # possible_locations = [( float(answer["X"]), 
            #                         float(answer["Y"]), 
            #                         float(answer["Z"])) for answer in answers]
            return "unknown_person"


#########################################################################################

class ConfirmPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['found', 'not_found', 'error'])
        
        self.robot = robot

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: ConfirmPerson\n")

        # reset robo pose
        self.robot.spindle.reset()
        self.robot.head.reset_position()

        self.robot.speech.speak("Let me make sure there's someone here.", block=False)

        # prepare query for detected person
        detectPersonQ = Conjunction(      Compound('property_expected', 'ObjectID', 'class_label', 'face'),
                                          Compound('property_expected', 'ObjectID', 'position', Compound('in_front_of', 'amigo')),
                                          Compound('property_expected', 'ObjectID', 'position', Sequence('X','Y','Z')),
                                          # Compound('not', Compound('confirmed', 'Waypoint'))
                                          )

        # start face segmentation node
        self.response_start = self.robot.perception.toggle(['face_segmentation'])

        if self.response_start.error_code == 0:
            rospy.loginfo("face_segmentation turned on")
        elif self.response_start.error_code == 1:
            rospy.logwarn("face_segmentation failed to start")
            self.robot.speech.speak("I was not able to start face segmentation.")

        # wait until the query detected person is true, or 10 second timeout
        wait_machine = Wait_query_true(self.robot, detectPersonQ, 8)
        wait_result = wait_machine.execute()

        # turn off face segmentation
        rospy.loginfo("Face Segmentation will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Face Segmentation is stopped")
        elif self.response_stop.error_code == 1:
            rospy.logwarn("Failed stopping Face Segmentation")

        # if the query timed out...
        if wait_result == 'timed_out':
            self.robot.speech.speak("I guess i was wrong.", block=False)
            return "not_found"
        elif wait_result == 'preempted':
            self.robot.speech.speak("Waiting for person was preemted.", block=False)
            return "not_found"
        # if the query succeeded
        elif wait_result == 'query_true':
            rospy.loginfo("\t\t[Cocktail Party] Person found!\n")
            
            result = self.robot.reasoner.query(detectPersonQ)

            if result:
                x,y,z = result[0]["X"],result[0]["Y"],result[0]["Z"]

                lookat_point = msgs.PointStamped(x,y,z)

                rospy.loginfo("\t\t[Cocktail Party] Looking at person's face at ({0}, {1}, {2})\n".format(x, y, z))
                self.robot.head.send_goal(lookat_point, timeout=0)
            else: 
                rospy.logwarn("\t\t[Cocktail Party] No face found in front of amigo\n")
            
            objectID = result[0]["ObjectID"]
            # self.robot.reasoner.query(Compound("assert", Compound("confirmed", objectID)))
            return "found"


#########################################################################################


class LearnPersonName(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["learned", "failed"],
                                output_keys=['personName_out'])

        self.robot = robot
        self.ask_user_service_get_learn_person_name = rospy.ServiceProxy('interpreter/ask_user', AskUser)
        self.person_learn_failed = 0

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: LearnPersonName\n")
        
        # reset pose
        # self.robot.spindle.high()
        # self.robot.head.reset_position()

        # ask the name of the user (within 3 tries and within 60 seconds an answer is received)
        self.response = self.ask_user_service_get_learn_person_name("name", 3 , rospy.Duration(60))
            
        # test if the name is allowed / exists
        for x in range(0,len(self.response.keys)):
            if self.response.keys[x] == "answer":
                response_answer = self.response.values[x]

        # if no answer was found / unsupported name
        if response_answer == "no_answer" or response_answer == "wrong_answer":
            if self.person_learn_failed == 2:
                self.robot.speech.speak("I will call you David")
                response_answer = "david"
            if self.person_learn_failed == 1:
                self.robot.speech.speak("I will call you Michael")
                response_answer = "michael"
            if self.person_learn_failed == 0:
                self.robot.speech.speak("I will call you Joseph")
                response_answer = "joseph"

            self.person_learn_failed += 1
        else:
             self.person_learn_failed = 0

        userdata.personName_out = str(response_answer)
        self.robot.speech.speak("Hello " + str(response_answer) + "!")

        # Person's name successfully learned
        return "learned"


#########################################################################################

class AskDrink(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["done", "failed"],
                                input_keys=['personName_in'])

        self.robot = robot
        self.ask_user_service_get_drink = rospy.ServiceProxy('interpreter/ask_user', AskUser)
        self.person_learn_failed = 0
        self.drink_learn_failed = 0

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: AskDrink\n")

        # ask the name of the drink
        self.response = self.ask_user_service_get_drink("drink_cocktail", 3 , rospy.Duration(60))  # This means that within 3 tries and within 60 seconds an answer is received. 
            
        # determine if the answer is allowed
        for x in range(0,len(self.response.keys)):
            if self.response.keys[x] == "answer":
                response_answer = self.response.values[x]

        # if the answer is not allwed just assume another possible drink
        if response_answer == "no_answer" or  response_answer == "wrong_answer":
            if self.drink_learn_failed == 2:
                self.robot.speech.speak("I will just bring you a seven up")
                response_answer = "seven_up"
                self.drink_learn_failed = 3
            elif self.drink_learn_failed == 1:
                self.robot.speech.speak("I will just bring you a milk")
                response_answer = "milk"
                self.drink_learn_failed = 2
            elif self.drink_learn_failed == 0:
                self.robot.speech.speak("I will just bring you a coke")
                response_answer = "coke"
                self.drink_learn_failed = 1
            else:
                self.robot.speech.speak("I will just bring you a coke")
                response_answer = "coke"

        # get Amigo's current location and rotation
        amigoPose = self.robot.base.location
        pos, rot = amigoPose.pose.position, amigoPose.pose.orientation
        rotation = transformations.euler_z_from_quaternion(rot)

        locX = amigoPose.pose.position.x
        locY = amigoPose.pose.position.y
        locZ = amigoPose.pose.position.z
        locPhi = rotation

        uniqueID = "{0}_{1}".format(userdata.personName_in, response_answer)
        lastLocID = "{0}_{1}_{2}".format(locX, locY, locZ)
        
        # save the requested drink
        # assert( goal( serve, UniqueID, Person, Drink, Person Last Known Location, Carrying the drink))
        self.robot.reasoner.query(  Compound('assert', 
                                    Compound('goal',
                                    Compound('serve', uniqueID, userdata.personName_in, response_answer, Compound('pose_2d', locX, locY, locZ)))))

        self.robot.reasoner.query(  Compound('assert', 
                                    Compound('waypoint', Compound('last_known_location', lastLocID), Sequence(locX, locY, locZ))))

        rospy.loginfo("\t\t[Cocktail Party] I'm getting a {0} for {1}".format(response_answer,userdata.personName_in))

        return "done"


#########################################################################################


class LearnPersonFace(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["face_learned" , "learn_failed"],
                                input_keys=['personName_in'])

        self.robot = robot

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: LearnPersonFace\n")

        self.robot.speech.speak("Now " + userdata.personName_in + ", let me look at you.", block=False)

        # learn the face of the person
        learn_machine = Learn_Person(self.robot, userdata.personName_in)
        learn_result = learn_machine.execute()

        # TO CHECK IF OUTCOME IS face_learned or learn_failed and ACT adequatly!
        if learn_result == 'face_learned':
            rospy.loginfo("Face learning succeeded")
        elif learn_result == 'learn_failed':
            rospy.logwarn("Failed learning face, WHAT TO DO!? Just continue to the next state and ask drink.")
        return learn_result


#########################################################################################


class NavToLookout(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["unreachable" , "arrived", "visited_all"])
        self.robot = robot

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: NavToLookout\n")

        # import ipdb; ipdb.set_trace()

        # get the waypoint of where to search, party_room_lookout
        goal_answers = self.robot.reasoner.query(Conjunction(  Compound("=", "Waypoint", Compound("party_room_lookout", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint"))))

        # if there is no location associated with lookout points say it
        if not goal_answers:
            self.robot.speech.speak("I visited all the lookout points", block=False)
            rospy.loginfo("\t\t[Cocktail Party] Visited all lookout points\n")
            return "visited_all"

        self.robot.speech.speak("Going to the next lookout point", block=False)

        # for now, take the first goal found
        goal_answer = goal_answers[0]

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypointName = goal_answer["Waypoint"]

        # navigate to the waypoint queried
        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        # assert that this location has been visited
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypointName)))

        # If nav_result is unreachable DO NOT stop looking, there are more options, return not_found when list of Waypoints is empty
        if nav_result == "unreachable":                    
            return "unreachable"
        elif nav_result == "preempted":
            return "unreachable"

        # we made it to the new goal. Let's have a look to see whether we can find the object here
        return "arrived"

#########################################################################################

class NavToWavingPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['going' , 'arrived', 'visited_all', 'max_orders'])
        
        global MAX_SIMULTANEOUS_ORDERS;

        self.robot = robot

    def execute(self, userdata=None):
        rospy.loginfo('\t\t[Cocktail Party] Entered State: NavToWavingPerson\n')

        # query the number of current orders
        goals = self.robot.reasoner.query(Compound("goal", "X"))
        
        if len(goals) >= MAX_SIMULTANEOUS_ORDERS:
            rospy.loginfo("\t\t[Cocktail Party] Maximum number of simultaneous orders reached ({0})\n". format(len(goals)))
            return 'max_orders'

        # compose person query
        qPeopleWaving = Conjunction(Compound('property_expected', 'ObjectID', 'class_label', 'validated_person'),
                                    Compound('property_expected', 'ObjectID', 'position', Sequence('X','Y','Z')),
                                    Compound('not', Compound('visited', 'ObjectID')),
                                    # Compound('not', Compound('ordered', 'ObjectID'))
                                    )

        # get results from the query
        goal_answers = self.robot.reasoner.query(qPeopleWaving)

        # if there is no location associated with lookout points
        if not goal_answers:
            rospy.loginfo("\t\t[Cocktail Party] Visited all the persons\n")
            return "visited_all"

        rospy.loginfo("\t\t[Cocktail Party] I found {0} person(s)\n". format(len(goal_answers)))

        # for now, take the first goal found
        goal_answer = goal_answers[0]

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Z"]))
        waypointName = goal_answer["ObjectID"]

        # Use the lookat query
        nav = NavigateGeneric(self.robot, lookat_query = qPeopleWaving)
        nav_result = nav.execute()

        # assert that this location has been visited
        self.robot.reasoner.query(Compound('assert', Compound('visited', waypointName)))
        # self.robot.reasoner.query(Compound('assert', Compound('ordered', waypointName)))

        # If nav_result is unreachable DO NOT stop looking, there are more options, return not_found when list of Waypoints is empty
        if nav_result == 'unreachable':                    
            return 'going'
        elif nav_result == 'preempted':
            return 'going'

        # we made it to the new goal. Let's have a look to see whether we can find the object here
        return 'arrived'

#########################################################################################

class NavToLastKnowLoc(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["unreachable" , "arrived", "visited_all"])
        
        self.robot = robot
        self.resetVisited = 0

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: NavToLastKnowLoc\n")

        qGoals = Conjunction(Compound("=", "Waypoint",  Compound("last_known_location", "ID")),
                                                        Compound("waypoint", "Waypoint", Sequence("X", "Y", "Z")),
                                                        Compound("not", Compound("visited", "Waypoint")))

        goals = self.robot.reasoner.query(qGoals)
        
        # reset visited pepople if all the last know locations have been visited
        if not goals:
            rospy.loginfo("\t\t[Cocktail Party] Visited all the last know locations\n")
            
            if self.resetVisited == 0:
                rospy.loginfo("\t\t[Cocktail Party] Reseting approached people and detections\n")
                self.robot.reasoner.reset()
                amigo.reasoner.query(Compound('retractall', Compound('approached', 'X')))
                self.resetVisited = 1

            return 'visited_all'
        else:    
            self.resetVisited = 0

            
            # self.robot.speech.speak("I'm going to {0}'s last know location.".format(goal_answer["Person"]))
            self.robot.speech.speak("Going to the place I last saw people.", block=False)

            # take the first goal found
            goal_answer = goals[0]
            waypointName = goal_answer["Waypoint"]
            # waypointName =  str(goal_answer["ObjectID"]) +  str(goal_answer["X"]) + str(goal_answer["Y"]) + str(goal_answer["Z"])

             # Use the lookat query
            nav = NavigateGeneric(self.robot, lookat_query = qGoals)
            nav_result = nav.execute()

            # assert that this location has been visited
            self.robot.reasoner.query(Compound("assert", Compound("visited", waypointName)))

            # If nav_result is unreachable DO NOT stop looking, there are more options, return not_found when list of Waypoints is empty
            if nav_result == "unreachable":                    
                return "unreachable"
            elif nav_result == "preempted":
                return "unreachable"

            # we made it to the new goal
            return "arrived"


#########################################################################################


class ServedStatus(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["complete", "incomplete"])
        global TOTAL_ORDERS;

        self.robot = robot

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: ServedStatus\n")

        # query the number of people served so far
        answer = self.robot.reasoner.query(Compound("people_served_count", "X"))
        nServed = float(answer[0]["X"])
        
        rospy.loginfo("\t\t[Cocktail Party] I already served {0} person(s)\n". format(nServed))

        # if there were enough people served, finish the challenge
        if nServed < TOTAL_ORDERS:
            # retract most of the facts
            self.robot.reasoner.query(Compound('retractall', Compound('goal', 'X')))
            self.robot.reasoner.query(Compound('retractall', Compound('carrying', 'X')))
            # self.robot.reasoner.query(Compound('retractall', Compound('ordered', 'X')))
            self.robot.reasoner.query(Compound('retractall', Compound('approached', 'X')))
            self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
            self.robot.reasoner.query(Compound('retract', Compound("waypoint", Compound("last_known_location", "X"), "Y")))

            self.robot.reasoner.reset()

            rospy.loginfo("\t\t[Cocktail Party] Need to serve {0} more person(s) to finish the challenge\n". format(TOTAL_ORDERS-nServed))
            self.robot.speech.speak("I still need to serve more people.", block=False)
            return "incomplete"
        else:
            return "complete"


#########################################################################################


class CheckPendingOrders(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["enough_orders", "insuficient_orders"])

        global MIN_SIMULTANEOUS_ORDERS;
        global TOTAL_ORDERS

        self.robot = robot

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: PendingOrders\n")

        # query the number of current orders
        answer = self.robot.reasoner.query(Compound("goal", "X"))
        
        if answer:
            nOrders = len(answer)
            rospy.loginfo("\t\t[Cocktail Party] I have {0} pending orders\n". format(nOrders))
        else: 
            nOrders = 0
            rospy.logwarn("\t\t[Cocktail Party] Could not query the number of goals! Setting it to 0.\n")
        
        # query the number of people served so far
        peopleServedCount = self.robot.reasoner.query(Compound("people_served_count", "X"))
        peopleServedCount = float(peopleServedCount[0]["X"])

        # if there are enough requests start serving, or if there are enough request to 
        #   complete the total number of people needed to serve
        if nOrders < MIN_SIMULTANEOUS_ORDERS and nOrders < TOTAL_ORDERS - peopleServedCount:
            self.robot.speech.speak("I need more requests before getting the drinks", block=False)
            return "insuficient_orders"
        else:
            # retract the people already served, maybe they want something again
            self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
            # self.robot.reasoner.query(Compound('retractall', Compound('ordered', 'X')))
            self.robot.reasoner.query(Compound('retractall', Compound('approached', 'X')))
            
            # self.robot.speech.speak("I will now get your drinks.", block=False)
            self.robot.reasoner.reset()
            return "enough_orders"  


#########################################################################################


class LookForDrinks(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["looking" , "found", "visited_all", "done"])

        self.robot = robot

    def execute(self, userdata = None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: LookForDrinks\n")

        # query requested drink, that haven't been picked up
        goals = self.robot.reasoner.query(Conjunction(  Compound("goal", Compound("serve", "ObjectID", "Person", "Drink", "LastKnowPose")), 
                                                        Compound("not", Compound("carrying", Compound("drink", "Drink", "CarryingLoc")))))

        # if there is no drink requested, return not found
        if not goals:
            self.robot.speech.speak("I picked up all the drinks requested", block=False)
            self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
            return "done"

        drinkNames = set([str(answer["Drink"]) for answer in goals])
        orderedDrinks = " and ".join(drinkNames) #str(goals[0]["Drink"])  

        # get the waypoint of where to search, storage_room
        storageRoomWaypts = self.robot.reasoner.query(Conjunction(  Compound("=", "Waypoint", Compound("storage_room", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint"))))

        # if there are no more unvisited locations, give up
        if not storageRoomWaypts:
            amigo.reasoner.reset()
            self.robot.speech.speak("I want to find the drinks, but I don't know where to go!", block=False)
            return "visited_all"

        # say different phrases randomly
        lookedIdx =  random.randint(0, 4)
        if lookedIdx == 1:
            self.robot.speech.speak("I'm on the move, looking for your " + orderedDrinks, block = False)
        elif lookedIdx == 2:
            self.robot.speech.speak("Still on the move looking for your " + orderedDrinks, block = False)
        else:
            self.robot.speech.speak("I think I know the location of your " + orderedDrinks, block = False)

        #take the first goal found
        goal = (float(storageRoomWaypts[0]["X"]), float(storageRoomWaypts[0]["Y"]), float(storageRoomWaypts[0]["Phi"]))
        waypointName = storageRoomWaypts[0]["Waypoint"]

        # navigate to the waypoint queried
        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        navResult = nav.execute()

        # assert that this location has been visited
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypointName)))

        # If navResult is unreachable DO NOT stop looking, there are more options, return not_found when list of Waypoints is empty
        if navResult == "unreachable" or navResult == "preempted": 
            return "looking"

        # we made it to the new goal. Let's have a look to see whether we can find the object here

        # look to points of interest, if there is any, look at it
        pointsInterest = self.robot.reasoner.query(Compound("point_of_interest", waypointName, Compound("point_3d", "X", "Y", "Z")))

        # If point of interest where found, take the first one
        if pointsInterest:
            point = pointsInterest[0]
            self.robot.head.send_goal(msgs.PointStamped(float(point["X"]), float(point["Y"]), float(point["Z"]), "/map"))

        self.robot.speech.speak("Let's see what I can find here", block=False)

        # query for detect object, finishes when something found or timeout!
        query_detect_object = Conjunction(Compound("goal", Compound("serve", "UniqueID", "Person", "Drink", "LastKnowPose")),
                                          Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                          Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")))

        # start object template matching
        self.response_start = self.robot.perception.toggle(["object_segmentation"])
 
        if self.response_start.error_code == 0:
            rospy.loginfo("Object segmentation has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Object segmentation failed to start")
            self.robot.speech.speak("I was not able to start object recognition.")
            return "visited_all"

        wait_machine = Wait_query_true(self.robot, query_detect_object, 7)
        wait_result = wait_machine.execute()

        rospy.loginfo("Object recogition will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Object segmentation is stopped")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping Object segmentation")

        # interpret results wait machine
        if wait_result == "timed_out":
            self.robot.speech.speak("Did not find your " + orderedDrinks, block=False)
            return "looking"
        elif wait_result == "preempted":
            self.robot.speech.speak("Finding drink was preempted.", block=False)
            return "looking"
        elif wait_result == "query_true":
            self.robot.speech.speak("Hey, I found something!", block=False)
            return "found"


#########################################################################################

class PreparePickup(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=[  'pickup_left', 
                                            'pickup_right', 
                                            'pickup_basket', 
                                            'grabbed_all',
                                            'no_requested_drinks_here'])

        # initializations
        self.robot = robot
        self.armCount = 0

    def execute(self, userdata = None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: PreparePickup\n")

        carryingRes = self.robot.reasoner.query(Compound("carrying", 
                                                Compound("drink", "Drink", "CarryingLoc")))

        # location already carrying drinks
        carryingNames = set([str(answer["CarryingLoc"]) for answer in carryingRes])

        # find which arms are available for carrying (default "right_arm")
        carryingLoc = "right_arm"

        if not "basket" in carryingNames:
            carryingLoc = "basket"
        elif not "right_arm" in carryingNames:
            carryingLoc = "right_arm"
        elif not "left_arm" in carryingNames:
            carryingLoc = "left_arm"

        # get the drinks requested that were seen in this storage location AND are NOT already in the robot's posession
        drinksReqHere = Conjunction(Compound("goal", Compound("serve", "UniqueID", "Person", "Drink", "LastKnowPose")),
                                    Compound("property_expected", "ObjectID", "class_label", "Drink"),
                                    # Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                    Compound("property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")),
                                    Compound("not", Compound("carrying", Compound("drink", "Drink", "CarryingLoc"))))

        drinksReqRes = self.robot.reasoner.query(drinksReqHere)

        if not drinksReqRes:
            # get the drinks requested that are NOT already in the robot's posession
            incompleteReq = self.robot.reasoner.query(  Conjunction(Compound("goal", Compound("serve", "UniqueID", "Person", "Drink", "LastKnowPose")),
                                                                    Compound("not", Compound("carrying", Compound("drink", "Drink", "CarryingLoc")))))

            if not incompleteReq:
                self.robot.speech.speak("I finished picking up all the drinks", block=False)
                self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
                self.robot.reasoner.reset()
                return 'grabbed_all'
            else:
                # there are still drinks requested, but they cannot be found here, so lets go to another storage
                self.robot.speech.speak("I still need to find drinks", block=False)
                return 'no_requested_drinks_here'
        else:
            rospy.loginfo("\t\t[Cocktail Party] Still have to pick up {0} orders\n".format(len(drinksReqRes)))

            drinkName = drinksReqRes[0]["Drink"]
            rospy.loginfo("\t\t[Cocktail Party] Preparing to pickup the {0}, with the {1}\n".format(drinkName, carryingLoc))

            # retract previous goals
            self.robot.reasoner.query(  Compound('retractall', Compound('grab_goal', 'X')))

            # add new goal
            self.robot.reasoner.query(  Compound("assert", 
                                        Compound("grab_goal",
                                        Compound("grab", drinkName, carryingLoc))))

            if carryingLoc == "left_arm":
                return 'pickup_left'
            elif carryingLoc  == "right_arm":
                return 'pickup_right'
            else:
                return 'pickup_basket'


#########################################################################################

class ResetSearchedLocations(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['done'])

        # initializations
        self.robot = robot
        self.resetCount = 0

    def execute(self, userdata = None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: ResetSearchedLocations\n")

        if self.resetCount == 0:
            self.robot.speech.speak("I could not pick up the drink you wanted. I'll try again.", block=False)
            self.resetCount+=1;

            self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
        else:
            self.resetCount=0;
            self.robot.speech.speak("I still could not pickup the drink. I give up.", block=False)


        return 'done'

#########################################################################################


class AssertPickup(smach.State):
    def __init__(self, robot, query):
        smach.State.__init__(   self, 
                                outcomes=['done'])

        # initializations
        self.robot = robot
        self.qPickedUpDrink = query

    def execute(self, userdata = None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: AssertPickup\n")

        res = self.robot.reasoner.query(self.qPickedUpDrink)

        # assert that this drink has been grabbed
        self.robot.reasoner.query(  Compound("assert", 
                                    Compound("carrying", 
                                    Compound("drink", res[0]["Drink"], res[0]["CarryingLoc"]))))

        rospy.loginfo("\t\t[Cocktail Party] Asserted pickup of a {0} with the {1}\n".format(res[0]["Drink"], res[0]["CarryingLoc"]))

        return 'done'


#########################################################################################


class PrepareDelivery(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=[   'handover_left', 
                                                'handover_right',
                                                'handover_basket',
                                                'unknown', 
                                                'not_correct',
                                                'no_people',
                                                'error'],
                                    output_keys=[   'person_out ',
                                                    'drink_out'])
        self.robot = robot

    def execute(self, userdata):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: PrepareDelivery\n")

        self.robot.head.reset_position()
        self.robot.spindle.reset()
        self.robot.spindle.high()

        self.robot.speech.speak("Let me se if i can find the correct person here.", block=False)

        # # prepare query for detected faces
        # facesDetectedQuery = Conjunction(   Compound( "property_expected", "ObjectID", "class_label", "human_face"),
        #                                     Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
        #                                     Compound( "property", "ObjectID", "name", Compound("discrete", "DomainSize", "NamePMF")))

        # # start face segmentation node
        # self.response_start = self.robot.perception.toggle(['face_recognition'])

        # if self.response_start.error_code == 0:
        #     rospy.loginfo("Face Recognition has started correctly")
        # elif self.response_start.error_code == 1:
        #     rospy.loginfo("Face Recognition failed to start")
        #     self.robot.speech.speak("I was not able to start Face Recognition.")
        #     return "error"

        # # wait until the query detected person is true, or 10 second timeout
        # wait_machine = Wait_query_true(self.robot, facesDetectedQuery, 10)
        # wait_result = wait_machine.execute()

        # # turn off face segmentation
        # rospy.loginfo("Face Recognition will be stopped now")
        # self.response_stop = self.robot.perception.toggle([])
        
        # if self.response_stop.error_code == 0:
        #     rospy.loginfo("Face Recognition is stopped")
        # elif self.response_stop.error_code == 1:
        #     rospy.loginfo("Failed stopping Face Recognition")

        # # if the query timed out...
        # if wait_result == "timed_out":
        #     self.robot.speech.speak("I thought there was someone here, but I'm mistaken.", block=False)
        #     rospy.loginfo("\t\t[Cocktail Party] No person names received from world model\n") 
        #     return 'no_people'
        # elif wait_result == "preempted":
        #     self.robot.speech.speak("Waiting for person was preemted.", block=False)
        #     return "error"
        # # if the query succeeded
        # elif wait_result == "query_true":
        #     facesDetected = self.robot.reasoner.query(facesDetectedQuery)

        #     if facesDetected:
        #         if len(facesDetected) > 1:
        #             rospy.logwarn("Multiple faces detected, only checking the first one!")

        #         name_pmf = facesDetected[0]["NamePMF"]

        #         # initialize variables
        #         personName = None
        #         name_prob = 0

        #         # try to get the name of the person detected
        #         for name_possibility in name_pmf:
        #             print name_possibility
        #             prob = float(name_possibility[0])
        #             if prob > 0.1856 and prob > name_prob:
        #                 personName = str(name_possibility[1][0])
        #                 name_prob = prob

        #         # If the person is not recognized
        #         if not personName:
        #             rospy.loginfo("\t\t[Cocktail Party]Couldn't identify the person\n")
        #             self.robot.speech.speak("I don't know who you are.", block=False)
        #             return "unknown"
        #         else:
        #             # get requests
        #             requests = self.robot.reasoner.query(Compound("goal", Compound("serve", "ObjectID", "Person", "Drink", "LastKnowPose")))

        #             # if there are no drinks requested, return error
        #             if not requests:
        #                 self.robot.speech.speak("I forgot who I had to serve", block=False)
        #                 return "error"

        #             personNames = set([str(answer["Person"]) for answer in requests])

        #             for name_possibility in personNames:
        #                 # if the person if recognized but not the one we are searching for
        #                 if personName == name_possibility:
                            
        #                     # TODO: I CAN MERGE THIS 2 QUERIES IN ONE CANT I?
        #                     drinkNameRes = self.robot.reasoner.query(   Compound("goal", 
        #                                                                 Compound("serve", "ObjectID", name_possibility, "Drink", "LastKnowPose")))
        #                     drinkName = str(drinkNameRes[0]["Drink"])

        #                     carryingRes = self.robot.reasoner.query(Compound("carrying", 
        #                                                             Compound("drink", drinkName, "CarryingLoc")))

        #                     if not carryingRes:
        #                         self.robot.speech.speak("I don't have any drinks for you", block=False)
        #                         rospy.logwarn("\t\t[Cocktail Party] Query about carried objects is empty!\n") 
        #                         return 'error'

        #                     person_out = personName
        #                     drink_out = drinkName

        #                     self.robot.reasoner.query(  Compound("retractall", 
        #                                                 Compound("delivering", "X")))

        #                     self.robot.reasoner.query(  Compound("assert", 
        #                                                 Compound("delivering",
        #                                                 Compound("delivery", personName, drinkName))))

        #                     carryingLoc = str(carryingRes[0]["CarryingLoc"])

        #                     self.robot.speech.speak("Hello " + personName + ", I have the " + drinkName + " your requested.", block=False)
                            
        #                     rospy.loginfo("\t\t[Cocktail Party] Delivering a {0} to {1}, carried in the {2}\n".format(
        #                         drinkName, personName, carryingLoc))

        #                     if carryingLoc == "left_arm":
        #                         return 'handover_left'
        #                     elif carryingLoc == "right_arm":
        #                         return 'handover_right'
        #                     elif carryingLoc == "basket":
        #                         return 'handover_basket'
        #                     else:
        #                         self.robot.speech.speak("Oh no, I forgot where i was carrying {0}'s order ".format(personName), block=False)
        #                         rospy.logwarn("\t\t[Cocktail Party] The CarryingLoc is not valid (carryingLoc=[{0}]).\n".format(carryingLoc)) 
        #                         return 'error'



        # perform face recognition on the person found
        self.response_start = self.robot.perception.toggle(["face_recognition"])

        if self.response_start.error_code == 0:
            rospy.loginfo("Face recognition has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Face recognition failed to start")
            self.robot.speech.speak("I was not able to start face recognition.")
            return 'error'
        
        # sleep while we wait for results on the recognition
        rospy.sleep(6.0)

        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Face recognition is stopped")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping face recognition")

        # Interpret face regnition results
        facesDetected = self.robot.reasoner.query( Conjunction( Compound( "property_expected", "ObjectID", "class_label", "face"),
                                                                Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                                                Compound( "property", "ObjectID", "name", Compound("discrete", "DomainSize", "NamePMF"))))
        # if there were people found...
        if facesDetected:
            if len(facesDetected) > 1:
                rospy.logwarn("Multiple faces detected, only checking the first one!")

            name_pmf = facesDetected[0]["NamePMF"]

            # initialize variables
            personName = None
            name_prob = 0

            # try to get the name of the person detected
            for name_possibility in name_pmf:
                print name_possibility
                prob = float(name_possibility[0])
                if prob > 0.1856 and prob > name_prob:
                    personName = str(name_possibility[1][0])
                    name_prob = prob

            # If the person is not recognized
            if not personName:
                rospy.loginfo("\t\t[Cocktail Party]Couldn't identify the person\n")
                self.robot.speech.speak("I don't know who you are.", block=False)
                return "unknown"
            else:
                # get requests
                requests = self.robot.reasoner.query(Compound("goal", Compound("serve", "ObjectID", "Person", "Drink", "LastKnowPose")))

                # if there are no drinks requested, return error
                if not requests:
                    self.robot.speech.speak("I forgot who I had to serve", block=False)
                    return "error"

                personNames = set([str(answer["Person"]) for answer in requests])

                for name_possibility in personNames:
                    # if the person is recognized and is the one we are searching for
                    if personName == name_possibility:
                        
                        # TODO: I CAN MERGE THIS 2 QUERIES IN ONE CANT I?
                        drinkNameRes = self.robot.reasoner.query(   Compound("goal", 
                                                                    Compound("serve", "ObjectID", name_possibility, "Drink", "LastKnowPose")))
                        drinkName = str(drinkNameRes[0]["Drink"])

                        carryingRes = self.robot.reasoner.query(Compound("carrying", 
                                                                Compound("drink", drinkName, "CarryingLoc")))

                        if not carryingRes:
                            self.robot.speech.speak("I don't have any drinks for you", block=False)
                            rospy.logwarn("\t\t[Cocktail Party] Query about carried objects is empty!\n") 
                            return 'error'

                        person_out = personName
                        drink_out = drinkName

                        self.robot.reasoner.query(  Compound("retractall", 
                                                    Compound("delivering", "X")))

                        self.robot.reasoner.query(  Compound("assert", 
                                                    Compound("delivering",
                                                    Compound("delivery", personName, drinkName))))

                        carryingLoc = str(carryingRes[0]["CarryingLoc"])

                        self.robot.speech.speak("Hello " + personName + ", I have the " + drinkName + " your requested.", block=False)
                        
                        rospy.loginfo("\t\t[Cocktail Party] Delivering a {0} to {1}, carried in the {2}\n".format(
                            drinkName, personName, carryingLoc))

                        if carryingLoc == "left_arm":
                            return 'handover_left'
                        elif carryingLoc == "right_arm":
                            return 'handover_right'
                        elif carryingLoc == "basket":
                            return 'handover_basket'
                        else:
                            self.robot.speech.speak("Oh no, I forgot where i was carrying {0}'s order ".format(personName), block=False)
                            rospy.logwarn("\t\t[Cocktail Party] The CarryingLoc is not valid (carryingLoc=[{0}]).\n".format(carryingLoc)) 
                            return 'error'

                self.robot.speech.speak("You are not the person I'm looking for", block=False)
                return 'not_correct'

        # if there were no people found...
        else:
            self.robot.speech.speak("I thought there was someone here, but I'm mistaken.", block=False)
            rospy.loginfo("\t\t[Cocktail Party] No person names received from world model\n") 
            return 'no_people'


#########################################################################################


class NavToDetectedPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=[   'unreachable', 
                                                'arrived',
                                                'visited_all'])
        self.robot = robot

    def execute(self, userdata):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: NavToDetectedPerson\n")

        person_query = Conjunction(Compound("property_expected", "ObjectID", "class_label", "validated_person"),
                                    Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")),
                                    Compound("not", Compound("visited", "ObjectID"),
                                    # Compound("not", Compound("approached", "ObjectID"))
                                    ))

        # get results from the query
        queryRes = self.robot.reasoner.query(person_query)

        # if there is no location associated with lookout points say it
        if not queryRes:
            rospy.loginfo("\t\t[Cocktail Party] Visited all the persons\n")
            return "visited_all"
        else:
            rospy.loginfo("\t\t[Cocktail Party] I found {0} person(s)\n". format(len(queryRes)))
            
            waypointName = queryRes[0]["ObjectID"]

            # Use the lookat query
            nav = NavigateGeneric(self.robot, lookat_query = person_query)
            nav_result = nav.execute()

            # assert that this location has been visited
            self.robot.reasoner.query(Compound("assert", Compound("visited", waypointName)))
            # self.robot.reasoner.query(Compound("assert", Compound("approached", waypointName)))

            if nav_result == "unreachable":                    
                return "unreachable"
            elif nav_result == "preempted":
                return "unreachable"

            return "arrived"

#########################################################################################


class HandoverDrinkLeft(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["done"])

        arm = robot.leftArm

        with self:
            smach.StateMachine.add( 'PRESENT_DRINK',
                                    Say(robot, ["I'm going to hand over your drink now", 
                                                "Here you go! Handing over your drink"], block=False),
                                    transitions={"spoken":"POSE"})

            smach.StateMachine.add( 'POSE',
                                    Handover_pose(arm, robot),
                                    transitions={   'succeeded':'PLEASE_TAKE',
                                                    'failed':'PLEASE_TAKE'})
            
            smach.StateMachine.add( 'PLEASE_TAKE',
                                    Say(robot, ["Please hold it, I'm going to let go.", 
                                                "Please grab the drink, I'm releasing"]),
                                    transitions={"spoken":"OPEN_GRIPPER"})

            smach.StateMachine.add( "OPEN_GRIPPER", 
                                    SetGripper(robot, arm, gripperstate=0, drop_from_frame="/amigo/grippoint_left"), #open
                                    transitions={   'succeeded':'RESET_ARM',
                                                    'failed':'RESET_ARM'})
            smach.StateMachine.add('RESET_ARM', 
                                    ArmToPose(robot, arm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), 
                                    transitions={   'done':'RESET_TORSO',
                                                    'failed':'RESET_TORSO'})
            smach.StateMachine.add('RESET_TORSO',
                                    ResetTorso(robot),
                                    transitions={   'succeeded':'CLOSE_AFTER_DROP',
                                                    'failed'   :'CLOSE_AFTER_DROP'})

            smach.StateMachine.add( 'CLOSE_AFTER_DROP',
                                    SetGripper(robot, arm, gripperstate=1), #close
                                    transitions={   'succeeded':'SAY_ENJOY',
                                                    'failed':'SAY_ENJOY'})

            smach.StateMachine.add( 'SAY_ENJOY',
                                    Say(robot, ["Enjoy your drink!", "I hope your're thirsty, enjoy!"], block=False),
                                    transitions={"spoken":"done"})


#########################################################################################
    

class HandoverDrinkRight(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["done"])

        arm = robot.rightArm

        with self:
            smach.StateMachine.add( 'PRESENT_DRINK',
                                    Say(robot, ["I'm going to hand over your drink now", 
                                                "Here you go! Handing over your drink"], block=False),
                                    transitions={"spoken":"POSE"})

            smach.StateMachine.add( 'POSE',
                                    Handover_pose(arm, robot),
                                    transitions={   'succeeded':'PLEASE_TAKE',
                                                    'failed':'PLEASE_TAKE'})
            
            smach.StateMachine.add( 'PLEASE_TAKE',
                                    Say(robot, ["Please hold it, I'm going to let go.", 
                                                "Please grab the drink, I'm releasing"]),
                                    transitions={"spoken":"OPEN_GRIPPER"})

            smach.StateMachine.add( "OPEN_GRIPPER", 
                                    SetGripper(robot, arm, gripperstate=0, drop_from_frame="/amigo/grippoint_left"), #open
                                    transitions={   'succeeded':'RESET_ARM',
                                                    'failed':'RESET_ARM'})
            smach.StateMachine.add('RESET_ARM', 
                                    ArmToPose(robot, arm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), 
                                    transitions={   'done':'RESET_TORSO',
                                                    'failed':'RESET_TORSO'})
            smach.StateMachine.add('RESET_TORSO',
                                    ResetTorso(robot),
                                    transitions={   'succeeded':'CLOSE_AFTER_DROP',
                                                    'failed'   :'CLOSE_AFTER_DROP'})

            smach.StateMachine.add( 'CLOSE_AFTER_DROP',
                                    SetGripper(robot, arm, gripperstate=1), #close
                                    transitions={   'succeeded':'SAY_ENJOY',
                                                    'failed':'SAY_ENJOY'})
            smach.StateMachine.add( 'SAY_ENJOY',
                                    Say(robot, ["Enjoy your drink!", "I hope you're thirsty, enjoy!"], block=False),
                                    transitions={"spoken":"done"})


#########################################################################################


class RetractServedPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                input_keys=['person_in', 'drink_in'],
                                outcomes=['done', 'all_served', 'failed'])
        # initializations
        self.robot = robot

    def execute(self, userdata):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: RetractServedPerson\n")

        # personName2 = str(userdata.person_in)
        # drinkName2 = str(userdata.drink_in)
        # rospy.loginfo("\t\t[Cocktail Party] Name: {0}, drink: {1}\n".format(personName2, drinkName2))
        # TODO PASS THE VARIABLES THROUGH USERDATA AND NOT QUERY (Not working, not sure why)

        deliveryRes = self.robot.reasoner.query(Compound("delivering",
                                                Compound("delivery", "Person", "Drink")))
    
        if not deliveryRes:
            rospy.logwarn("\t\t[Cocktail Party] No information on this delivery.\n")
            return 'failed'
        else:
            personName = deliveryRes[0]["Person"]
            drinkName = deliveryRes[0]["Drink"]

        if personName and drinkName:
            # retract from drinks being carried
            retractDrink = self.robot.reasoner.query(   Compound( "retract", 
                                                        Compound( "carrying", 
                                                        Compound( "drink", drinkName, "Arm"))))

            # retract from persons being served
            retractPerson = self.robot.reasoner.query(  Compound( "retract", 
                                                        Compound( "goal", 
                                                        Compound( "serve", "ObjectID", personName, drinkName, "LastKnowPose"))))

            rospy.loginfo("\t\t[Cocktail Party] Retracted {0} drink and {1} person.\n".format(len(retractDrink), len(retractPerson)))

            ############################################################
            # update the number of served people
            servedCount = self.robot.reasoner.query(Compound("people_served_count", "Counter"))

            nServed = float(servedCount[0]["Counter"]) + 1.0

            rospy.loginfo("\t\t[Cocktail Party]Hurray! Another drink was served (total served: {0})\n".format(float(servedCount[0]["Counter"])))

            self.robot.reasoner.query(Compound("retractall", Compound("people_served_count", "X")))
            self.robot.reasoner.query(Compound("assert",Compound("people_served_count", nServed)))
            ############################################################

            carrying = self.robot.reasoner.query(Compound("carrying", Compound( "drink", "Drink", "Arm")))

            if not carrying:
                self.robot.speech.speak("I finished delivering every drink.", block=False)
                return 'all_served'
            else:
                rospy.loginfo("\t\t[Cocktail Party] Still have to deliver {0} drink(s)\n".format(len(carrying)))
                return 'done'

        else:
            rospy.logwarn("\t\t[Cocktail Party] Unable to retract the person and drink served\n")
            return 'failed'


########################################################################################


class DropInBasket(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['done'])

        # initializations
        self.robot = robot

    def execute(self, userdata = None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: DropInBasket\n")

        self.robot.spindle.high()

        self.robot.leftArm.send_joint_goal(-0.1, -0.4, 0.2, 1.6, 0.0, 0.5, 0.0, timeout=5.0)

        self.robot.leftArm.send_joint_goal(-0.3, -0.2, 0.7, 1.8, 0.0, 0.3, 0.6, timeout=5.0)

        self.robot.spindle.send_goal(0.2, timeout= 5.0)

        self.robot.leftArm.send_gripper_goal_open(timeout=5.0)

        self.robot.spindle.high()

        self.robot.leftArm.send_gripper_goal_close(timeout=5.0)

        self.robot.leftArm.reset_arm()

        self.robot.spindle.reset()

        return 'done'


#########################################################################################


class FocusOnFace(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, 
                                    outcomes=['loop', 'preempted', 'error'])

        self.robot = robot
        self.firstTime = 0;

    def execute(self, userdata=None):
        # rospy.loginfo("\t\t[Cocktail Party] Entered State: FocusOnFace\n")

        # turn on human tracking only once
        if self.firstTime == 0:
            self.robot.head.reset_position()
            self.robot.spindle.reset()

            self.response_start = self.robot.perception.toggle(['face_segmentation'])

            if self.response_start.error_code == 0:
                rospy.loginfo("face_segmentation turned on")
            elif self.response_start.error_code == 1:
                rospy.logwarn("face_segmentation failed to start")
                self.robot.speech.speak("I was not able to start face segmentation.")

            self.firstTime = 1

        # prepare query for detected faces
        qFaces = Conjunction(   Compound("property_expected", "ObjectID", "class_label", "face"),
                                Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        res = self.robot.reasoner.query(qFaces)

        if res:
            x,y,z = res[0]["X"],res[0]["Y"],res[0]["Z"]

            lookat_point = msgs.PointStamped(x,y,z)

            # rospy.loginfo("\t\t[Cocktail Party] Looking at person's face ({0}, {1}, {2})\n".format(x, y, z))
            self.robot.head.send_goal(lookat_point, timeout=0)
        # else: 
            # rospy.loginfo("\t\t[Cocktail Party] No face found in front of amigo\n")

        # get the preempted flag value
        answer = self.robot.reasoner.query(Compound("preempt_head_focus", "X"))

        if answer:
            preempt = float(answer[0]["X"])
        else:
            rospy.logwarn("\t\t[Cocktail Party] Unable to query preempt_head_focus, assuming its 0\n".format(x, y, z))
            preempt = 0

        # loop or preempt focus on face
        if preempt == 0:
            rospy.sleep(0.3)
            return 'loop'
        else:
            rospy.loginfo("\t\t[Cocktail Party] FocusOnFace behaviour preempted\n")
            
            # turn off face_segmentation
            self.response_stop = self.robot.perception.toggle([''])

            if self.response_stop.error_code == 0:
                rospy.loginfo("face_segmentation turned off")
            elif self.response_stop.error_code == 1:
                rospy.logwarn("face_segmentation failed to shutdown")
                self.robot.speech.speak("I was not able to stop face segmentation.")

            # reset flags
            self.robot.reasoner.query(Compound("retractall", Compound("preempt_head_focus", "X")))
            self.robot.reasoner.query(Compound("assert",Compound("preempt_head_focus", "0")))
            self.firstTime = 0

            # give it one second for the face focus to preempt, to avoind conflicts between human_tracking and face_recognition
            # rospy.sleep(1)

            return 'preempted'


#########################################################################################


class FocusOnFaceLoop(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['done'])

        with self:

            smach.StateMachine.add( 'FOCUS_ON_FACE',
                                    FocusOnFace(robot),
                                    transitions={   'loop':'FOCUS_ON_FACE',
                                                    'error':'FOCUS_ON_FACE',
                                                    'preempted':'done'})


#########################################################################################


class PreemptFaceFocus(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])

        self.robot = robot

    def execute(self, userdata):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: PreemptFaceFocus!\n")

        self.robot.reasoner.query(Compound("retractall", Compound("preempt_head_focus", "X")))
        self.robot.reasoner.query(Compound("assert",Compound("preempt_head_focus", "1")))
        
        rospy.sleep(2)

        return 'done'


#########################################################################################


class HandoverUndeliveredDrinks(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['done'])

        self.robot = robot

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: HandoverUndeliveredDrinks\n")

        self.robot.speech.speak("Someone please remove these drinks from my hands.")

        # retract from drinks being carried
        amigo.reasoner.query(Compound('retractall', Compound('carrying', 'X')))
        amigo.reasoner.query(Compound('retractall', Compound('goal', 'X')))

        rospy.sleep(10)

        return 'done'


#########################################################################################


class HearContinue(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['continue', 'no_continue', 'force_continue'])

        self.robot = robot
        self.hearCount = 0
        self.ask_user_service_continue = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: HearContinue\n")

        if self.hearCount > 0 and self.hearCount < 4:
            self.robot.speech.speak(["I did not hear you say continue, i'm still waiting.",
                                     "Did you already say continue? Please say it again."], block=False)
            self.hearCount += 1
        else:
            self.hearCount = 0
            rospy.loginfo("\t\t[Cocktail Party] Forcing continue after several tries\n")
            return 'force_continue'

        self.response = self.ask_user_service_continue("continue", 4, rospy.Duration(15))

        for x in range(0,len(self.response.keys)):
            if self.response.keys[x] == 'answer':
                if self.response.values[x] == 'true':
                    return 'continue'
                else: 
                    return 'no_continue'

        rospy.logwarn("answer was not found in response of interpreter. Should not happen!!")
        return 'no_continue'


#########################################################################################


class TakeNewOrder(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["done"])

        with self:

            # Learn the persons name
            smach.StateMachine.add( 'LEARN_PERSON_NAME',
                                    LearnPersonName(robot),
                                    remapping={     'personName_out':'personName'},
                                    transitions={   'learned':'PREEMPT_FACE_FOCUS',
                                                    'failed':'PREEMPT_FACE_FOCUS'})

            smach.StateMachine.add( 'PREEMPT_FACE_FOCUS',
                                    PreemptFaceFocus(robot),
                                    transitions={   'done':'LEARN_PERSON_FACE'})

            # Learn the persons face
            smach.StateMachine.add( 'LEARN_PERSON_FACE',
                                    LearnPersonFace(robot),
                                    remapping={     'personName_in':'personName'},
                                    transitions={   'face_learned':'TAKE_ORDER',
                                                    'learn_failed':'LEARN_PERSON_FACE'})

            # Take the persons order
            smach.StateMachine.add( 'TAKE_ORDER',
                                    AskDrink(robot),
                                    remapping={     'personName_in':'personName'},
                                    transitions={   'done':'done',
                                                    'failed':'TAKE_ORDER'})


#########################################################################################


class TakeNewOrderFocus(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["done"])

        with self:

            concurrenceContainer = smach.Concurrence(   outcomes = ['succeeded'],
                                                        default_outcome = 'succeeded',
                                                        input_keys = [],
                                                        output_keys = [],
                                                        outcome_map = {'succeeded':{'TAKE_NEW_ORDER':'done', 
                                                                                    'FOCUS_ON_FACE_LOOP':'done'}})

            with concurrenceContainer:
                smach.Concurrence.add('FOCUS_ON_FACE_LOOP', FocusOnFaceLoop(robot))
                smach.Concurrence.add('TAKE_NEW_ORDER', TakeNewOrder(robot))
                

            smach.StateMachine.add('CONCURRENT_MACHINE', 
                                    concurrenceContainer,
                                    transitions={'succeeded':'done'})


#########################################################################################


class CocktailParty(smach.StateMachine):
    def __init__(self, robot):

        # Create a SMACH state machine
        sm = smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])

        # Queries:
        qPartyRoomWaypts = Compound('waypoint', 'party_room', Compound('pose_2d', 'X', 'Y', 'Phi'))

        qWaitingPlace = Compound('waypoint', 'party_room_waiting', Compound('pose_2d', 'X', 'Y', 'Phi'))

        qLookoutWaypts = Compound('waypoint', Compound('party_room_lookout', 'R'), Compound('pose_2d', 'X', 'Y', 'Phi'))

        qStorageRoomWaypts = Compound('waypoint', Compound('storage_room', 'S'), Compound('pose_2d', 'X', 'Y', 'Phi'))

        qGrabpoint = Conjunction(  Compound("grab_goal", Compound("grab", "Drink", "CarryingLoc")),
                                   Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                   Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                   Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))

        arm = robot.leftArm

#-----------------------------------------ENTER THE ROOM---------------------------------------------------------

        with self:

            # Start
            smach.StateMachine.add( 'START_CHALLENGE',
                                    StartChallengeRobust(robot, 'initial'), #query_meeting_point
                                    transitions={   'Done':'DELETE_MODELS', 
                                                    'Aborted':'DELETE_MODELS', 
                                                    'Failed':'DELETE_MODELS'})
            
            # Clean face recognition models
            smach.StateMachine.add( 'DELETE_MODELS',
                                    DeleteModels(robot), 
                                    transitions={   'done':'SAY_GOTO_PARTY_ROOM'})

            # Inform you are going to the Party Room
            smach.StateMachine.add( 'SAY_GOTO_PARTY_ROOM',
                                    Say(robot, "I'm going to the party room.", block=False),
                                    transitions={   'spoken':'GOTO_PARTY_ROOM'})

            #Go to the party room
            smach.StateMachine.add('GOTO_PARTY_ROOM',
                                    NavigateGeneric(robot, goal_query = qPartyRoomWaypts),
                                    transitions={   'arrived':'LOOKOUT_CONTAINER', 
                                                    'unreachable':'LOOKOUT_CONTAINER', 
                                                    'preempted':'LOOKOUT_CONTAINER', 
                                                    'goal_not_defined':'LOOKOUT_CONTAINER'})


#----------------------------------------FIND PEOPLE TO SERVE-------------------------------------------------------
           
            # Create container for the iterator
            lookoutContainer = smach.StateMachine(  outcomes = ['looked_all', 'got_request'])
            
            # Go to predefined lookout positions in the room and look arround to detect waving people
            with lookoutContainer:

                smach.StateMachine.add( 'NAV_TO_LOOKOUT',
                                        NavToLookout(robot),
                                        transitions={   'unreachable':'NAV_TO_LOOKOUT',
                                                        'arrived':'DETECT_WAVING_PEOPLE',
                                                        'visited_all':'looked_all'})
    
                smach.StateMachine.add( 'DETECT_WAVING_PEOPLE',
                                        DetectWavingPeople(robot),
                                        transitions={   'detected':'got_request',
                                                        'not_detected':'NAV_TO_LOOKOUT',
                                                        'error':'NAV_TO_LOOKOUT'})

            #add container to the iterator lookoutIterator
            smach.StateMachine.add( 'LOOKOUT_CONTAINER', 
                                    lookoutContainer, 
                                    transitions={   'looked_all':'GOTO_WAITING_PLACE',
                                                    'got_request':'ORDERS_CONTAINER'})


#----------------------------------------WAIT FOR PEOPLE TO COME IN FRONT (BACKUP PLAN)------------------


            # Go to waiting location if no one was detected previously (backup plan)
            smach.StateMachine.add('GOTO_WAITING_PLACE',
                                    NavigateGeneric(robot, goal_query = qWaitingPlace),
                                    transitions={   'arrived':'WAIT_PERSON_ITERATOR',
                                                    'unreachable':'WAIT_PERSON_ITERATOR', 
                                                    'preempted':'WAIT_PERSON_ITERATOR', 
                                                    'goal_not_defined':'WAIT_PERSON_ITERATOR'})

            # Iterate through Lookout Waypoints twice before giving up
            waitPersonIterator = smach.Iterator(outcomes=['timed_out', 'found_person'], 
                                                it = lambda:range(0, 3),
                                                it_label='waitIndex',
                                                input_keys=[],
                                                output_keys=[],
                                                exhausted_outcome = 'timed_out')
            with waitPersonIterator:

                waitPersonContainer = smach.StateMachine(   outcomes = ['found_person', 'continue'],
                                                            input_keys = ['waitIndex'])
                with waitPersonContainer:

                    # Wait for someone to come in front of the robot
                    smach.StateMachine.add( "WAIT_FOR_PERSON", 
                                            WaitForPerson(robot),
                                            remapping={     'waitIndex_in':'waitIndex'},
                                            transitions={   'waiting':'continue',
                                                            'unknown_person':'found_person'})

                #close waitPersonContainer
                smach.Iterator.set_contained_state( 'WAIT_PERSON_CONTAINER', 
                                                     waitPersonContainer, 
                                                     loop_outcomes=['continue'],
                                                     break_outcomes=['found_person'])

            # add the lookoutIterator to the main state machine
            smach.StateMachine.add( 'WAIT_PERSON_ITERATOR',
                                    waitPersonIterator,
                                    {   'timed_out':'ASSUMING_PERSON_FRONT',
                                        'found_person':'TAKE_NEW_ORDER'})


            smach.StateMachine.add( 'ASSUMING_PERSON_FRONT',
                                    Say(robot, "I'm going to assume there is a person in front of me", block=False),
                                    transitions={   'spoken':'TAKE_NEW_ORDER'})

            smach.StateMachine.add( 'TAKE_NEW_ORDER',
                                    TakeNewOrder(robot),
                                    transitions={'done':'CHECK_PENDING_ORDERS'})

#----------------------------------------TAKE THEIR ORDERS-------------------------------------------------------

            # create orders container
            ordersContainer = smach.StateMachine(  outcomes = ['succeeded','aborted'])

            with ordersContainer:

                smach.StateMachine.add( 'NAV_TO_WAIVING_PERSON',
                                        NavToWavingPerson(robot),
                                        transitions={   'going':'NAV_TO_WAIVING_PERSON',
                                                        'arrived':'CONFIRM_PERSON',
                                                        'max_orders':'succeeded',
                                                        'visited_all':'succeeded'})

                smach.StateMachine.add( 'CONFIRM_PERSON',
                                        ConfirmPerson(robot),
                                        transitions={   'found':'TAKE_NEW_ORDER_FOCUS',
                                                        'error':'NAV_TO_WAIVING_PERSON',
                                                        'not_found':'NAV_TO_WAIVING_PERSON'})

                smach.StateMachine.add( 'TAKE_NEW_ORDER_FOCUS',
                                        TakeNewOrder(robot),
                                        transitions={   'done':'NAV_TO_WAIVING_PERSON'})

            # add orders container to the main state machine
            smach.StateMachine.add( 'ORDERS_CONTAINER',
                                    ordersContainer,
                                    transitions={   'succeeded':'CHECK_PENDING_ORDERS',
                                                    'aborted':'SAY_FAILED'})

            smach.StateMachine.add( 'CHECK_PENDING_ORDERS',
                                    CheckPendingOrders(robot),
                                    transitions={   'enough_orders':'FIND_DRINKS_CONTAINER',
                                                    'insuficient_orders':'LOOKOUT_CONTAINER'})

#--------------------------------------------FIND THE REQUESTED DRINKS---------------------------------------------

            # create find drinks container
            findDrinksContainer = smach.StateMachine(  outcomes = ['succeeded','aborted'])

            with findDrinksContainer:

                # Go to storage room waypoint and look for the drinks
                smach.StateMachine.add( 'LOOK_FOR_DRINK',
                                        LookForDrinks(robot),
                                        transitions={   'looking':'LOOK_FOR_DRINK',
                                                        'found':'PREPARE_PICKUP',
                                                        'visited_all':'SAY_DRINK_NOT_FOUND',
                                                        'done':'succeeded'})

                # Pickup the drink
                smach.StateMachine.add( 'PREPARE_PICKUP',
                                        PreparePickup(robot),
                                        transitions={   'pickup_left':'PICKUP_DRINK_LEFT',
                                                        'pickup_right':'PICKUP_DRINK_RIGHT',
                                                        'pickup_basket':'PICKUP_DRINK_BASKET',
                                                        'no_requested_drinks_here':'LOOK_FOR_DRINK',
                                                        'grabbed_all':'succeeded'}) 

                smach.StateMachine.add( 'PICKUP_DRINK_LEFT',
                                        GrabMachine('left', robot, qGrabpoint),
                                        transitions={   'succeeded':'ASSERT_PICKUP',
                                                        'failed':'RESET_SEARCHED_LOCATIONS' }) 

                smach.StateMachine.add( 'PICKUP_DRINK_RIGHT',
                                        GrabMachine('right', robot, qGrabpoint),
                                        transitions={   'succeeded':'ASSERT_PICKUP',
                                                        'failed':'RESET_SEARCHED_LOCATIONS' }) 

                smach.StateMachine.add( 'PICKUP_DRINK_BASKET',
                                        GrabMachine('left', robot, qGrabpoint),
                                        transitions={   'succeeded':'DROP_IN_BASKET',
                                                        'failed':'RESET_SEARCHED_LOCATIONS' })

                smach.StateMachine.add( 'DROP_IN_BASKET',
                                        DropInBasket(robot),
                                        transitions={   'done':'ASSERT_PICKUP'}) 

                smach.StateMachine.add( 'RESET_SEARCHED_LOCATIONS',
                                        ResetSearchedLocations(robot),
                                        transitions={   'done':'LOOK_FOR_DRINK'})

                smach.StateMachine.add( 'ASSERT_PICKUP',
                                        AssertPickup(robot, qGrabpoint),
                                        transitions={   'done':'PREPARE_PICKUP'}) 

                smach.StateMachine.add( 'SAY_DRINK_NOT_FOUND',
                                        Say(robot, "I could not find any of the drinks requested.", block=False),
                                        transitions={   'spoken':'aborted' }) 

            # add find drinks container to the main state machine
            smach.StateMachine.add( 'FIND_DRINKS_CONTAINER',
                                    findDrinksContainer,
                                    transitions={   'succeeded':'DELIVER_DRINKS_CONTAINER',
                                                    'aborted':'DELIVER_DRINKS_CONTAINER'})


#-----------------------------------------DELIVER THE REQUESTED DRINKS---------------------------------------------

            # create deliver drinks container
            deliverDrinksContainer = smach.StateMachine(  outcomes = ['succeeded','aborted'])

            with deliverDrinksContainer:      

                # Go to the last person known location
                smach.StateMachine.add('NAV_TO_LAST_KNOWN_LOCATION',
                                        NavToLastKnowLoc(robot),
                                        transitions={   'unreachable':'NAV_TO_LAST_KNOWN_LOCATION' , 
                                                        'arrived':'DETECT_PEOPLE', 
                                                        'visited_all':'NAV_TO_LOOKOUT'})

                # if the request weren't all delivered continue searching on the lookout points
                smach.StateMachine.add( 'NAV_TO_LOOKOUT',
                                        NavToLookout(robot),
                                        transitions={   'unreachable':'NAV_TO_LOOKOUT',
                                                        'arrived':'DETECT_PEOPLE',
                                                        'visited_all':'aborted'})

                smach.StateMachine.add( 'DETECT_PEOPLE',
                                        DetectPeople(robot),
                                        transitions={   'detected':'NAV_TO_DETECTED_PERSON',
                                                        'not_detected':'NAV_TO_LAST_KNOWN_LOCATION',
                                                        'error':'NAV_TO_LAST_KNOWN_LOCATION'})

                smach.StateMachine.add( 'NAV_TO_DETECTED_PERSON',
                                        NavToDetectedPerson(robot),
                                        transitions={   'unreachable':'NAV_TO_DETECTED_PERSON',
                                                        'arrived':'PREPARE_DELIVERY',
                                                        'visited_all':'NAV_TO_LAST_KNOWN_LOCATION'})

                smach.StateMachine.add( 'PREPARE_DELIVERY',
                                        PrepareDelivery(robot),
                                        remapping={     'person_out':'personServed',
                                                        'drink_out':'drinkServed'},
                                        transitions={   'handover_left':'HANDOVER_DRINK_LEFT',
                                                        'handover_right':'HANDOVER_DRINK_RIGHT',
                                                        'handover_basket':'SAY_DRINK_IN_BASKET',
                                                        'unknown':'NAV_TO_DETECTED_PERSON',
                                                        'not_correct':'NAV_TO_DETECTED_PERSON',
                                                        'no_people':'NAV_TO_DETECTED_PERSON',
                                                        'error':'NAV_TO_DETECTED_PERSON'})

                smach.StateMachine.add( 'SAY_DRINK_IN_BASKET',
                                        Say(robot, "Please remove your drink from my basket and say continue when you're done"),
                                        transitions={'spoken':'HEAR_CONTINUE' })

                smach.StateMachine.add( 'HEAR_CONTINUE',
                                        HearContinue(robot),
                                        transitions={   'continue':'RETRACT_SERVED_PERSON',
                                                        'force_continue':'RETRACT_SERVED_PERSON',
                                                        'no_continue':'HEAR_CONTINUE'})

                smach.StateMachine.add( 'HANDOVER_DRINK_LEFT',
                                        HandoverDrinkLeft(robot),
                                        transitions={   'done':'RETRACT_SERVED_PERSON'})

                smach.StateMachine.add( 'HANDOVER_DRINK_RIGHT',
                                        HandoverDrinkRight(robot),
                                        transitions={   'done':'RETRACT_SERVED_PERSON'})

                smach.StateMachine.add( 'SAY_PERSON_NOT_FOUND',
                                        Say(robot, 'No one here.', block=False),
                                        transitions={   'spoken':'NAV_TO_LAST_KNOWN_LOCATION' })

                smach.StateMachine.add( 'RETRACT_SERVED_PERSON',
                                        RetractServedPerson(robot),
                                        remapping={     'person_in':'personServed',
                                                        'drink_in':'drinkServed'},
                                        transitions={   'done':'NAV_TO_LAST_KNOWN_LOCATION',
                                                        'all_served':'succeeded',
                                                        'failed':'aborted'})

            # add find drinks container to the main state machine
            smach.StateMachine.add( 'DELIVER_DRINKS_CONTAINER',
                                    deliverDrinksContainer,
                                    transitions={   'succeeded':'SERVED_STATUS',
                                                    'aborted':'HANDOVER_UNDELIVERED_DRINKS'})

            smach.StateMachine.add( 'HANDOVER_UNDELIVERED_DRINKS',
                                    HandoverUndeliveredDrinks(robot),
                                    transitions={   'done':'SERVED_STATUS'})

            smach.StateMachine.add( 'SERVED_STATUS',
                                    ServedStatus(robot),
                                    transitions={   'complete':'SAY_FINISHED_SERVING',
                                                    'incomplete':'LOOKOUT_CONTAINER'})


 #-------------------------------------------FINISH THE CHALLENGE AND LEAVE----------------------------------------

            # Say the job is complete
            smach.StateMachine.add('SAY_FINISHED_SERVING',
                                    Say(robot, "I finished my task, enjoy your drinks", block=False),
                                    transitions={'spoken':'EXIT'}) 

            # Go to the exit
            smach.StateMachine.add('EXIT',
                                    NavigateGeneric(robot, goal_name='exit_1'),
                                    transitions={   'arrived':'FINISH', 
                                                    'preempted':'CLEAR_PATH_TO_EXIT', 
                                                    'unreachable':'CLEAR_PATH_TO_EXIT', 
                                                    'goal_not_defined':'CLEAR_PATH_TO_EXIT'})
        
            smach.StateMachine.add('CLEAR_PATH_TO_EXIT',
                                    Say(robot, "I couldn't go to the exit. Please clear the path, I will give it another try."),
                                    transitions={'spoken':'GO_TO_EXIT_SECOND_TRY'})

            # Try again
            smach.StateMachine.add('GO_TO_EXIT_SECOND_TRY', 
                                    NavigateGeneric(robot, goal_name='exit_2'),
                                    transitions={   'arrived':'FINISH', 
                                                    'preempted':'FINISH', 
                                                    'unreachable':'FINISH', 
                                                    'goal_not_defined':'FINISH'})

            smach.StateMachine.add( 'FINISH', Finish(robot),
                                    transitions={'stop':'Done'})

            smach.StateMachine.add('SAY_FAILED', 
                                    Say(robot, "I could not accomplish my task, sorry about that, please forgive me."),
                                    transitions={   'spoken':'EXIT'})
 

 #########################################################################################

if __name__ == '__main__':
    rospy.init_node('executive_cocktail_party')
 
    amigo = Amigo(wait_services=True)


    # Total number of people served
    amigo.reasoner.query(Compound('retractall', Compound('people_served_count', 'X')))
    amigo.reasoner.query(Compound('assert',Compound('people_served_count', '0')))

    # Variable to preempt the focus on face loop
    amigo.reasoner.query(Compound('retractall', Compound('preempt_head_focus', 'X')))
    amigo.reasoner.query(Compound('assert',Compound('preempt_head_focus', '0')))

    # requests being fullfilled from the people who ordered
    amigo.reasoner.query(Compound('retractall', Compound('goal', 'X')))

    # locations visited
    amigo.reasoner.query(Compound('retractall', Compound('visited', 'X')))

    # drinks currently being carried
    amigo.reasoner.query(Compound('retractall', Compound('carrying', 'X')))

    # tag to identify people who already ordered, used when getting the requests
    # amigo.reasoner.query(Compound('retractall', Compound('ordered', 'X')))

    # tag to identify people who the robot already tried to deliver the drink, but failed
    amigo.reasoner.query(Compound('retractall', Compound('approached', 'X')))

    # last know location of people saved as waypoints
    amigo.reasoner.query(Compound('retract', Compound('waypoint', Compound('last_known_location', 'X'), 'Y')))

    amigo.reasoner.query(Compound('retractall', Compound('challenge', 'X')))
    amigo.reasoner.query(Compound('retractall', Compound('state', 'X', 'Y')))
    amigo.reasoner.query(Compound('retractall', Compound('type', 'X', 'Y')))

    # Load locations and objects from knowledge files
    amigo.reasoner.query(Compound('load_database', 'tue_knowledge', 'prolog/locations.pl'))
    amigo.reasoner.query(Compound('load_database', 'tue_knowledge', 'prolog/objects.pl'))

    # Assert current challenge
    amigo.reasoner.assertz(Compound('challenge', 'cocktailparty'))
  

    initial_state = None
    # initial_state = 'FIND_DRINKS_CONTAINER'
    # initial_state = 'DELIVER_DRINKS_CONTAINER'
    # initial_state = 'GOTO_WAITING_PLACE'

    machine = CocktailParty(amigo)
    
    if initial_state != None :
        amigo.reasoner.reset()

        machine.set_initial_state([initial_state])

        amigo.reasoner.query(   Compound('assert', 
                                Compound('goal',
                                Compound('serve', 'david_coke', 'david', 'coke', Compound('pose_2d', '2.5071', '1.2574', '0.0')))))

        amigo.reasoner.query(   Compound('assert', 
                                Compound('goal',
                                Compound('serve', 'william_coffee', 'william', 'orange_drink', Compound('pose_2d', '3.0161', '0.9186', '0.0')))))

        #amigo.reasoner.query(   Compound("assert", 
        #                        Compound("carrying", 
                                #Compound("drink", "coke", "basket"))))

        #amigo.reasoner.query(   Compound("assert", 
                                #Compound("carrying", 
                                #Compound("drink", "cup", "right_hand"))))

        amigo.reasoner.query(   Compound('assert', 
                                Compound('waypoint', Compound('last_known_location', '2.5071_1.2574_0.0'), Sequence('2.5071', '1.2574', '0.0'))))

        amigo.reasoner.query(   Compound('assert', 
                                Compound('waypoint', Compound('last_known_location', '3.0161_0.9186_0.0'), Sequence('3.0161', '0.9186', '0.0'))))


    introserver = smach_ros.IntrospectionServer('SM_TOP', machine, '/SM_ROOT_PRIMARY')
    introserver.start()

    try:
        outcome = machine.execute()
    except Exception, e:
        amigo.speech.speak(e)
    finally:
        introserver.stop()
