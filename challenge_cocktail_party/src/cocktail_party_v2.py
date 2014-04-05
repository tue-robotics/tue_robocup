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

grasp_arm = "left"
#grasp_arm = "right"


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

        # reset head position
        self.robot.head.reset_position()

        # TODO: can i define non-blocking also here?
        self.robot.speech.speak("Ladies and gentlemen, please wave to call me and place an order.")

        # Turn ON Human Tracking
        self.response_start = self.robot.perception.toggle(['human_tracking'])

        if self.response_start.error_code == 0:
            rospy.loginfo("human_tracking turned on")
        elif self.response_start.error_code == 1:
            rospy.loginfo("human_tracking failed to start")
            self.robot.speech.speak("I was not able to start human tracking.")
            return "error"

        # self.robot.head.set_pan_tilt(pan=-1.0)
        # rospy.sleep(2)
        # self.robot.head.set_pan_tilt(pan=-0.0)
        # rospy.sleep(2)
        # self.robot.head.set_pan_tilt(pan=1.0)
        rospy.sleep(4)

        self.robot.head.reset_position()

        # Turn OFF Human Tracking
        self.response_stop = self.robot.perception.toggle([])

        # if self.response_stop.error_code == 0:
        #     rospy.loginfo("human_tracking turned off")
        # elif self.response_stop.error_code == 1:
        #     rospy.loginfo("human_tracking failed to shutdown")
        #     self.robot.speech.speak("I was not able to stop human tracking.")
        #     return "error"

        # compose person query
        qPeopleFound = Conjunction( Compound("property_expected", "ObjectID", "class_label", "validated_person"),
                                    Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                    Compound("not", Compound("ordered", "ObjectID")))

        # get results from the query
        peopleFoundRed = self.robot.reasoner.query(qPeopleFound)
        
        if len(peopleFoundRed) > 0 :
            self.robot.speech.speak("Someone is calling me, I will be with there soon!")
            return 'detected'
        else:
            self.robot.speech.speak("No one called me. Forever alone.")
            return 'not_detected'

#########################################################################################

class WaitForPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["waiting", "unknown_person"],
                                input_keys=['waitIndexIn'])
        
        self.robot = robot

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: WaitForPerson\n")

        # reset visited locations and people who already ordered
        # self.robot.reasoner.query(Compound("retractall", Compound("visited", "X")))
        # self.robot.reasoner.query(Compound("retractall", Compound("ordered", "X")))

        waited_no = userdata.waitIndexIn

        if waited_no == 3:
            # Reset the counter for waiting
            waited_no = 0;
            self.robot.speech.speak("I was not able to detect a person, assuming someone is in front of me!")
            return "unknown_person"
        else:
            rospy.loginfo("Waited for {0} times!!!".format(waited_no))

        # reset robo pose
        self.robot.spindle.reset()
        self.robot.reasoner.reset()
        self.robot.head.set_pan_tilt(tilt=-0.2)
        
        self.robot.speech.speak("Ladies and gentlemen, please step in front of me to order your drink.")

        # prepare query for detected person
        query_detect_person = Conjunction(Compound("property_expected", "ObjectID", "class_label", "human_face"),
                                          Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                          Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

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
            self.robot.speech.speak("Please, don't keep me waiting.")
            waited_no += 1
            return "waiting"
        elif wait_result == "preempted":
            self.robot.speech.speak("Waiting for person was preemted... I don't even know what that means!")
            return "waiting"
        # if the query succeeded
        elif wait_result == "query_true":
            answers = self.robot.reasoner.query(query_detect_person)
            # get the person location
            possible_locations = [( float(answer["X"]), 
                                    float(answer["Y"]), 
                                    float(answer["Z"])) for answer in answers]

            # x,y,z = possible_locations[0]

            # if z > 1.5:
            #     self.robot.spindle.high()
            #     rospy.logdebug("Spindle should come up now!")

            # look at the person
            # lookat_point = msgs.PointStamped(x,y,z)
            # rospy.loginfo("AMIGO should look at person now. (x = {0}, y = {1}, z = {2})".format(x,y,z))
            # self.robot.head.send_goal(lookat_point,timeout=0)

            # Reset the counter for waiting for person
            # waited_no = 0;
            # self.robot.reasoner.query(Compound("retractall", Compound("waited_times_no", "X")))
            # self.robot.reasoner.query(Compound("assertz",Compound("waited_times_no", waited_no)))

            # ASSERT THE FACE FOUND AS A VALIDATED PERSON!

            # go to Unknown Person state
            return "unknown_person"

         
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
                self.person_learn_failed = 3
            if self.person_learn_failed == 1:
                self.robot.speech.speak("I will call you Michael")
                response_answer = "michael"
                self.person_learn_failed = 2
            if self.person_learn_failed == 0:
                self.robot.speech.speak("I will call you Joseph")
                response_answer = "joseph"
                self.person_learn_failed = 1

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

        # currentLoc[0] = amigoPose.pose.position.X
        # currentLoc[1] = amigoPose.pose.position.Y
        # currentLoc[2] = rotation
        
        # save the requested drink
        # assert( goal( serve( Person, Drink, Last Known Location)))
        self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("serve", userdata.personName_in, response_answer,  Compound("1","1","1")))))

        # rospy.loginfo("I'm getting a {0} for {1}", format(response_answer), format(userdata.personName_in))
        return "done"


#########################################################################################

class LearnPersonFaceCustom(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["face_learned" , "learn_failed"],
                                input_keys=['personName_in'])

        self.robot = robot

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: LearnPersonFaceCustom\n")

        self.robot.speech.speak("Now " + userdata.personName_in + ", let me have a look at you, such that I can remember you later.")

        # learn the face of the person
        learn_machine = Learn_Person(self.robot, userdata.personName_in)
        learn_result = learn_machine.execute()
        # self.robot.reasoner.query(Compound("retractall", Compound("goal", "X")))  # Make sure we're not left with a goal from last time

        ## TO CHECK IF OUTCOME IS face_learned or learn_failed and ACT adequatly!
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

        self.robot.speech.speak("Going to the next lookout point.")

        # get the waypoint of where to search, party_room_lookout
        goal_answers = self.robot.reasoner.query(Conjunction(  Compound("=", "Waypoint", Compound("party_room_lookout", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint"))))

        # if there is no location associated with lookout points say it
        if not goal_answers:
            self.robot.speech.speak("Finished visiting all locations")
            # retract all visited locations
            # amigo.reasoner.query(Compound("retractall", Compound("visited", "X")))
            return "visited_all"

        # for now, take the first goal found
        goal_answer = goal_answers[0]

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypoint_name = goal_answer["Waypoint"]

        # navigate to the waypoint queried
        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        # assert that this location has been visited
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        # If nav_result is unreachable DO NOT stop looking, there are more options, return not_found when list of Waypoints is empty
        if nav_result == "unreachable":                    
            return "unreachable"
        elif nav_result == "preempted":
            return "going"

        # we made it to the new goal. Let's have a look to see whether we can find the object here
        return "arrived"

#########################################################################################

class NavToPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["going" , "arrived", "visited_all"])
        
        self.robot = robot

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: NavToPerson\n")

        # compose person query
        qPeopleWaving = Conjunction(Compound("property_expected", "ObjectID", "class_label", "validated_person"),
                                    Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Phi")),
                                    Compound("not", Compound("visited", "ObjectID")),
                                    Compound("not", Compound("ordered", "ObjectID")))

        # get results from the query
        goal_answers = self.robot.reasoner.query(qPeopleWaving)
        
        rospy.loginfo("I found {0} persons". format(len(goal_answers)))

        # if there is no location associated with lookout points say it
        if not goal_answers:
            self.robot.speech.speak("Finished visiting all people")
            # retract all visited locations
            # amigo.reasoner.query(Compound("retractall", Compound("visitedPerson", "X")))
            return "visited_all"

        self.robot.speech.speak("Going to the next person who waved.")

        # for now, take the first goal found
        goal_answer = goal_answers[0]

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypoint_name = goal_answer["ObjectID"]

        # navigate to the waypoint queried
        nav = NavigateGeneric(self.robot, goal_pose_2d = goal)
        nav_result = nav.execute()

        # assert that this location has been visited
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))
        self.robot.reasoner.query(Compound("assert", Compound("ordered", waypoint_name)))

        # If nav_result is unreachable DO NOT stop looking, there are more options, return not_found when list of Waypoints is empty
        if nav_result == "unreachable":                    
            return "going"
        elif nav_result == "preempted":
            return "going"

        # TODO: Make amigo look at the person

        # we made it to the new goal. Let's have a look to see whether we can find the object here
        return "arrived"


#########################################################################################


class ServedStatus(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["complete", "incomplete"])

        self.robot = robot

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: ServedStatus\n")

        # query the number of people served so far
        answer = self.robot.reasoner.query(Compound("n_people_served", "X"))
        nServed = float(answer[0]["X"])
        
        rospy.loginfo("I already served {0} people". format(nServed))

        # if there were enough people served, finish the challenge
        if nServed < 3:
            # retract most of the facts
            amigo.reasoner.query(Compound("retractall", Compound("visited", "X")))
            self.robot.reasoner.reset()
            return "incomplete"
        else:
            return "complete"


#########################################################################################


class PendingOrders(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["enough_orders", "insuficient_orders"])

        self.robot = robot

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: ServedStatus\n")

        # retract all visited locations and people
        # import ipdb; ipdb.set_trace()
        self.robot.reasoner.query(Compound("retractall", Compound("visited", "X")))

        # SHOULD BE RECTRACTING WAVING PERSONS!!!!!!!!

        # query the number of current orders
        answer = self.robot.reasoner.query(Compound("goal", "X"))
        nOrders = len(answer)
        
        rospy.loginfo("I have {0} pending orders". format(nOrders))

        # if there are enough requests start serving
        if nOrders < 2:
            self.robot.speech.speak("I need more requests before getting the drinks")
            return "insuficient_orders"
        else:
            # retract the people already served, maybe they want something again
            self.robot.reasoner.query(Compound("retractall", Compound("ordered", "X")))
            self.robot.speech.speak("I will now get your drinks")
            return "enough_orders"  


#########################################################################################

class LookForDrink(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["looking" , "found", "not_found"])

        self.robot = robot

    def execute(self, userdata = None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: LookForDrink\n")

        # query requested drink
        return_result = self.robot.reasoner.query(Compound("goal", Compound("serve", "Person", "Drink", "LastLoc")))

        # if there is no drink requested, return not found
        if not return_result:
            self.robot.speech.speak("I forgot which drink you wanted")
            return "not_found"

        drink_names = set([str(answer["Drink"]) for answer in return_result])            
        # get the response from the query requested drink
        serving_drink = " or ".join(drink_names) #str(return_result[0]["Drink"])  

        # get the waypoint of where to search, storage_room
        goal_answers = self.robot.reasoner.query(Conjunction(  Compound("=", "Waypoint", Compound("storage_room", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint"))))

        # if there is no location associated with drinks return not found
        if not goal_answers:
            self.robot.speech.speak("I want to find a " + serving_drink + ", but I don't know where to go... I'm sorry!")
            return "not_found"


        # looked_no = randrange(1, 3)
        looked_no = 1
        # say different phrases depending on the number of looked up drinks up until now
        if looked_no == 1:
            self.robot.speech.speak("I'm on the move, looking for your " + serving_drink)
        elif looked_no == 2:
            self.robot.speech.speak("Still on the move looking for your " + serving_drink)
        else:
            self.robot.speech.speak("I think I know the location of your " + serving_drink)


        # for now, take the first goal found
        goal_answer = goal_answers[0]

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypoint_name = goal_answer["Waypoint"]

        # navigate to the waypoint queried
        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        # assert that this location has been visited
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        # If nav_result is unreachable DO NOT stop looking, there are more options, return not_found when list of Waypoints is empty
        if nav_result == "unreachable":                    
            return "looking"
        elif nav_result == "preempted":
            return "looking"

        # we made it to the new goal. Let's have a look to see whether we can find the object here

        # look to points of interest, if there is any, look at it
        roi_answers = self.robot.reasoner.query(Compound("point_of_interest", waypoint_name, Compound("point_3d", "X", "Y", "Z")))
        if roi_answers:
            roi_answer = roi_answers[0]
            self.robot.head.send_goal(msgs.PointStamped(float(roi_answer["X"]), float(roi_answer["Y"]), float(roi_answer["Z"]), "/map"))


        # query for detect object, finishes when something found or timeout!
        query_detect_object = Conjunction(Compound("goal", Compound("serve", "Drink")), #get all requested drinks
                                          Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                          Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")))


        self.robot.speech.speak("Let's see what I can find here")

        # start object template matching
        self.response_start = self.robot.perception.toggle(["object_segmentation"])
 
        if self.response_start.error_code == 0:
            rospy.loginfo("Template matching has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Template matching failed to start")
            self.robot.speech.speak("I was not able to start object recognition.")
            return "not_found"

        wait_machine = Wait_query_true(self.robot, query_detect_object, 7)
        wait_result = wait_machine.execute()

        rospy.loginfo("Object recogition will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Template matching is stopped")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping template matching ")

        ############################################################
        # update the number of served people (this will no be staying here, its just for now)
        answer = self.robot.reasoner.query(Compound("n_people_served", "X"))
        nServed = float(answer[0]["X"])
        nServed += 1
        rospy.loginfo("Hurray! Another drink was served ({0})".format(nServed))
        self.robot.reasoner.query(Compound("retractall", Compound("n_people_served", "X")))
        self.robot.reasoner.query(Compound("assertz",Compound("n_people_served", nServed)))
        ############################################################

        # interpret results wait machine
        if wait_result == "timed_out":
            self.robot.speech.speak("Did not find your " + serving_drink)
            return "looking"
        elif wait_result == "preempted":
            self.robot.speech.speak("Finding drink was preemted... I don't even know what that means!")
            return "looking"
        elif wait_result == "query_true":
            self.robot.speech.speak("Hey, I found your " + serving_drink)
            return "found"


#########################################################################################

class LookForPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["looking" , "found", "not_found"])
        self.robot = robot

    def execute(self, userdata=None):
        
        # find out who we need to return the drink to
        return_result = self.robot.reasoner.query(Compound("current_person", "Person"))

        if not return_result:
            self.robot.speech.speak("That's horrible, I forgot who I should bring the drink to!")
            return "not_found"

        serving_person = str(return_result[0]["Person"])
        
        # Get the next waypoint in the party room
        goal_answers = self.robot.reasoner.query(Conjunction(  
                                                    Compound("=", "Waypoint", Compound("party_room", "W")),
                                                    Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                    Compound("not", Compound("visited", "Waypoint"))
                                                            ))

        if not goal_answers:
            return_drink_result = self.robot.reasoner.query(Compound("goal", Compound("serve", "Drink")))
            if return_drink_result:
                serving_drink = str(return_drink_result[0]["Drink"])
                self.robot.speech.speak(str(serving_person) +", I have been looking everywhere. To hand over your " + str(serving_drink))
            else:
                self.robot.speech.speak(str(serving_person) +", I have been looking everywhere. But could not find you.")
            return "not_found"

        # for now, take the first goal found
        goal_answer = goal_answers[0]

        self.robot.speech.speak(str(serving_person) + ", I'm on my way!", language="us", personality="kyle", voice="default", mood="excited")

        # convert response location to numbers
        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypoint_name = goal_answer["Waypoint"]

        # go to waypoint
        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        if nav_result == "unreachable":                    
            return "looking"
        elif nav_result == "preempted":
            return "looking"

        # we made it to the new goal. Let's have a look to see whether we can find the person here
        self.robot.head.set_pan_tilt(tilt=-0.2)
        self.robot.spindle.reset()

        self.robot.speech.speak("Let me see who I can find here...")
        
        # turn on face segmentation
        self.response_start = self.robot.perception.toggle(["face_segmentation"])
        if self.response_start.error_code == 0:
            rospy.loginfo("Face segmentation has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Face segmentation failed to start")
            self.robot.speech.speak("I was not able to start face segmentation.")
            return 'looking'
        rospy.sleep(2)

        rospy.loginfo("Face segmentation will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Face segmentation is stopped")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping face segmentation")

        # compose person query
        person_query = Conjunction(  
                                    Compound( "property_expected", "ObjectID", "class_label", "face"),
                                    Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        # query for people found
        person_result = self.robot.reasoner.query(person_query)
 
        # if no one was found
        if not person_result:

            # repeat the search with a different head tilt
            self.robot.head.set_pan_tilt(tilt=0.2)
            #self.robot.speech.speak("No one here. Checking for sitting persons!")

            self.response_start = self.robot.perception.toggle(["face_segmentation"])
            if self.response_start.error_code == 0:
                rospy.loginfo("Face segmentation has started correctly")
            elif self.response_start.error_code == 1:
                rospy.loginfo("Face segmentation failed to start")
                self.robot.speech.speak("I was not able to start face segmentation.")
                return 'looking'
            rospy.sleep(2)

            rospy.loginfo("Face segmentation will be stopped now")
            self.response_stop = self.robot.perception.toggle([])
        
            if self.response_stop.error_code == 0:
                rospy.loginfo("Face segmentation is stopped")
            elif self.response_stop.error_code == 1:
                rospy.loginfo("Failed stopping face segmentation")

            # repeat the query
            person_result = self.robot.reasoner.query(person_query)

            # if no one was found continue looking
            if not person_result:
                self.robot.speech.speak("No one here.")
                return "looking"
        if len(person_result) > 1:
            self.robot.speech.speak("I see some people!")
        else:
            self.robot.speech.speak("I found someone!")
        return "found"     


#########################################################################################

class PersonFound(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["correct", "unknown", "persons_unchecked", "not_correct"])
        self.robot = robot

    def execute(self, userdata=None):

        # prepare query to get faces detected in front of amigo
        person_query = Conjunction(  
                        Compound( "property_expected", "ObjectID", "class_label", "face"),
                        Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                        Compound( "property_expected", "ObjectID", "position", Compound("point_3d", "X", "Y", "Z")),
                        Compound( "not", Compound("registered", "ObjectID")))

        # get results from the query
        person_detection_result = self.robot.reasoner.query(person_query)
        rospy.loginfo("I found {0} people". format(len(person_detection_result)))
        
        # navigate to one of the detected faces
        nav = NavigateGeneric(self.robot, lookat_query=person_query)
        nav_result = nav.execute()

        # if nav_result == "unreachable":  #Compound("meeting_point", waypoint_name)
        #     self.robot.speech.speak("The location of the person unreachable", block=False)
        #     return "?"
        # elif nav_result == "preempted":
        #     return "?"
        # elif nav_result == "arrived":
        #     self.robot.speech.speak("I reached the person location", block=False)
        #     return "?"
        # else: #goal not defkined
        #     self.robot.speech.speak("The location of the person is invalid", block=False)
        #     return "?"

        # retrieve all possible locations
        possible_locations = [( float(answer["X"]), 
                                float(answer["Y"]), 
                                float(answer["Z"])) for answer in person_detection_result]

        # if there are locations of interest...
        if len(possible_locations) > 0:
            x,y,z = possible_locations[0]
            if z > 1.5:
                self.robot.spindle.high()
                rospy.logdebug("Spindle should come up now!")

            # Look at person
            lookat_point = msgs.PointStamped(x,y,z)
            rospy.loginfo("AMIGO should look at person now. (x = {0}, y = {1}, z = {2})".format(x,y,z))
            self.robot.head.send_goal(lookat_point,timeout=0)

        # get the objectID of the person we are trying to serve the drink to
        self.robot.reasoner.query(Conjunction(Compound("current_person", "ObjectID"),
                                              Compound("assert", Compound("registered", "ObjectID"))))
        return_result = self.robot.reasoner.query(Compound("current_person", "Person"))       

        # if there is no result, we forgot who we were serving, return!
        if not return_result:
            self.robot.speech.speak("That's horrible, I forgot who I should bring the drink to!")
            return "not_correct"

        serving_person = str(return_result[0]["Person"]) 
        self.robot.speech.speak("Hi there, human. Please look into my eyes, so I can recognize you.")
        
        # perform face recognition on the person found
        self.response_start = self.robot.perception.toggle(["face_recognition"])
        if self.response_start.error_code == 0:
            rospy.loginfo("Face recognition has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Face recognition failed to start")
            self.robot.speech.speak("I was not able to start face recognition.")
            return 'unknown'
        
        # sleep while we wait for results on the recognition
        rospy.sleep(6.0)

        rospy.loginfo("Face recognition will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Face recognition is stopped")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping face recognition")

        # Interpret face regnition results
        person_result = self.robot.reasoner.query(
                                            Conjunction(  
                                                Compound( "property_expected", "ObjectID", "class_label", "face"),
                                                Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                                Compound( "property", "ObjectID", "name", Compound("discrete", "DomainSize", "NamePMF"))))
        # if there were people found...
        if len(person_result) > 0:
            name_pmf = person_result[0]["NamePMF"]

            if len(person_result) > 1:
                rospy.logwarn("Multiple faces detected, only checking the first one!")

            name=None
            name_prob=0

            for name_possibility in name_pmf:
                print name_possibility
                prob = float(name_possibility[0])
                if prob > 0.1856 and prob > name_prob:
                    name = str(name_possibility[1][0])
                    name_prob = prob

            # If the person is not recognized
            if not name:
                self.robot.speech.speak("I don't know who you are.")
                if len(person_detection_result) > 1:
                    return 'persons_unchecked'
                else:
                    return "unknown"  

            # if the person if recognized but not the one we are searching for
            if name != serving_person:
                self.robot.speech.speak("Hello " + str(name) + "! You are not the one I should return this drink to. Moving on!")
                if len(person_detection_result) > 1:
                    return 'persons_unchecked'
                else:
                    return "not_correct"      

            # if it is the persons we are searching for
            if name:
                self.robot.speech.speak("Hello " + str(name)) 
                return "correct"

        # if there were no people found...
        else:
            self.robot.speech.speak("I thought there was someone here, but I'm mistaken.")
            if len(person_detection_result) > 1:
                return 'persons_unchecked'
            rospy.loginfo("No person names received from world model") 
        return "not_correct"


#########################################################################################

class HandoverToKnownHuman(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["done"])

        if grasp_arm == "left":
            arm = robot.leftArm
        if grasp_arm == "right":
            arm = robot.rightArm

        with self:
            smach.StateMachine.add( 'PRESENT_DRINK',
                                    Say(robot, ["I'm going to hand over your drink now", "Here you go! Handing over your drink"], block=False),
                                    transitions={"spoken":"POSE"})

            smach.StateMachine.add( 'POSE',
                                    Handover_pose(arm, robot),
                                    transitions={   'succeeded':'PLEASE_TAKE',
                                                    'failed':'PLEASE_TAKE'})
            
            smach.StateMachine.add( 'PLEASE_TAKE',
                                    Say(robot, ["Please hold the drink, I'm going to let it go.", "Please take the drink, I'll let it go"]),
                                    transitions={"spoken":"OPEN_GRIPPER"})

            smach.StateMachine.add( "OPEN_GRIPPER", 
                                    SetGripper(robot, arm, gripperstate=0, drop_from_frame="/amigo/grippoint_left"), #open
                                    transitions={   'succeeded':'CLOSE_AFTER_DROP',
                                                    'failed':'CLOSE_AFTER_DROP'})
            smach.StateMachine.add( 'CLOSE_AFTER_DROP',
                                    SetGripper(robot, arm, gripperstate=1), #close
                                    transitions={   'succeeded':'RESET_ARM',
                                                    'failed':'RESET_ARM'})
            smach.StateMachine.add('RESET_ARM', 
                                    ArmToPose(robot, arm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), 
                                    transitions={   'done':'RESET_TORSO',
                                                    'failed':'RESET_TORSO'})
            smach.StateMachine.add('RESET_TORSO',
                                    ResetTorso(robot),
                                    transitions={   'succeeded':'SAY_ENJOY',
                                                    'failed'   :'SAY_ENJOY'})

            smach.StateMachine.add( 'SAY_ENJOY',
                                    Say(robot, ["Enjoy your drink!", "I hope your thirsty, enjoy!"], block=False),
                                    transitions={"spoken":"done"})


#########################################################################################

class HandoverToUnknownHuman(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["done"])

        if grasp_arm == "left":
            arm = robot.leftArm
        if grasp_arm == "right":
            arm = robot.rightArm

        qPartyRoomWaypts = Compound("waypoint", "party_room", Compound("pose_2d", "X", "Y", "Phi"))    

        with self:
            smach.StateMachine.add('GOTO_PARTY_ROOM',
                                    NavigateGeneric(robot, goal_query=qPartyRoomWaypts),
                                    transitions={   "arrived":"PRESENT_DRINK", 
                                                    "unreachable":"PRESENT_DRINK", 
                                                    "preempted":"PRESENT_DRINK", 
                                                    "goal_not_defined":"PRESENT_DRINK"})
            smach.StateMachine.add( 'PRESENT_DRINK',
                                    Say(robot, ["I'm going to hand over your drink now", "Here you go! Handing over your drink"],block=False),
                                    transitions={"spoken":"POSE"})

            smach.StateMachine.add( 'POSE',
                                    Handover_pose(arm, robot),
                                    transitions={   'succeeded':'PLEASE_TAKE',
                                                    'failed':'PLEASE_TAKE'})
            
            smach.StateMachine.add( 'PLEASE_TAKE',
                                    Say(robot, ["Please hold the drink, I'm going to let it go.", "Please take the drink, i'll let it go"]),
                                    transitions={"spoken":"OPEN_GRIPPER"})

            smach.StateMachine.add( "OPEN_GRIPPER", 
                                    SetGripper(robot, arm, gripperstate=0, drop_from_frame="/amigo/grippoint_right"), #open
                                    transitions={   'succeeded':'CLOSE_AFTER_DROP',
                                                    'failed':'CLOSE_AFTER_DROP'})
            smach.StateMachine.add( 'CLOSE_AFTER_DROP',
                                    SetGripper(robot, arm, gripperstate=1), #close
                                    transitions={   'succeeded':'RESET_ARM',
                                                    'failed':'RESET_ARM'})
            smach.StateMachine.add('RESET_ARM', 
                                    ArmToPose(robot, arm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), 
                                    transitions={   'done':'RESET_TORSO',
                                                    'failed':'RESET_TORSO'})
            smach.StateMachine.add('RESET_TORSO',
                                    ResetTorso(robot),
                                    transitions={   'succeeded':'SAY_ENJOY',
                                                    'failed'   :'SAY_ENJOY'})

            smach.StateMachine.add( 'SAY_ENJOY',
                                    Say(robot, ["Enjoy your drink!", "I hope your thirsty, enjoy!"], block=False),
                                    transitions={"spoken":"done"})

#########################################################################################

class FocusOnFace(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['loop', 'preempted'])

        self.robot = robot

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: FocusOnFace\n")

        self.robot.perception.toggle(['human_tracking'])      

        # prepare query to get faces detected in front of amigo
        person_query = Conjunction(  
                        Compound( "property_expected", "ObjectID", "class_label", "human_face"),
                        Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                        Compound( "property_expected", "ObjectID", "position", Compound("point_3d", "X", "Y", "Z")))

        res = self.robot.reasoner.query(person_query)

        if len(res) > 0:
            x,y,z = res[0][X],res[0][Y],res[0][Z]

            lookat_point = msgs.PointStamped(x,y,z)

            rospy.loginfo("AMIGO should look at person now. (x = {0}, y = {1}, z = {2})".format(x, y, z))
            self.robot.head.send_goal(lookat_point, timeout=0)


        # check if the behaviour is supposed to be preempted or loop
        answer = self.robot.reasoner.query(Compound("preempt_head_focus", "X"))
        preempt = float(answer[0]["X"])

        if preempt == 0:
            rospy.sleep(0.3)
            return 'loop'
        else:
            return 'preempted'
        

class FocusOnFaceLoop(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['done'])

        with self:

            # Learn the persons name
            smach.StateMachine.add( 'FOCUS_ON_FACE',
                                    FocusOnFace(robot),
                                    transitions={   'loop':'FOCUS_ON_FACE',
                                                    'preempted':'done'})


class PreemptFaceFocus(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])

        self.robot = robot

    def execute(self, userdata):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: PreemptFaceFocus!!!!!\n")

        robot.reasoner.query(Compound("retractall", Compound("preempt_head_focus", "X")))
        robot.reasoner.query(Compound("assert",Compound("preempt_head_focus", "1")))

        self.robot.perception.toggle(['']) 

        return 'done'


#########################################################################################


class TakeNewOrder(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["done"])

        with self:

            # Learn the persons name
            smach.StateMachine.add( 'LEARN_PERSON_NAME',
                                    LearnPersonName(robot),
                                    remapping={     'personName_out':'personName'},
                                    transitions={   'learned':'LEARN_PERSON_FACE',
                                                    'failed':'LEARN_PERSON_NAME'})

            # Learn the persons face
            smach.StateMachine.add( 'LEARN_PERSON_FACE',
                                    LearnPersonFaceCustom(robot),
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
                smach.Concurrence.add('TAKE_NEW_ORDER', TakeNewOrder(robot))
                smach.Concurrence.add('FOCUS_ON_FACE_LOOP', FocusOnFaceLoop(robot))

            smach.StateMachine.add('CONCURRENT_MACHINE', 
                                    concurrenceContainer,
                                    transitions={'succeeded':'PREEMPT_FACE_FOCUS'})

            smach.StateMachine.add( 'PREEMPT_FACE_FOCUS',
                                    PreemptFaceFocus(robot),
                                    transitions={   'done':'done'})


#########################################################################################

class CocktailParty(smach.StateMachine):
    def __init__(self, robot):

        # Create a SMACH state machine
        sm = smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])

        robot.reasoner.assertz(Compound("ordered", "dummy"))
        robot.reasoner.query(Compound("retractall", Compound("ordered", "X")))

        # Queries:
        qPartyRoomWaypts = Compound('waypoint', 'party_room', Compound('pose_2d', 'X', 'Y', 'Phi'))

        qWaitingPlace = Compound('waypoint', 'party_room_waiting', Compound('pose_2d', 'X', 'Y', 'Phi'))

        qLookoutWaypts = Compound('waypoint', Compound('party_room_lookout', 'R'), Compound('pose_2d', 'X', 'Y', 'Phi'))

        qStorageRoomWaypts = Compound('waypoint', Compound('storage_room', 'S'), Compound('pose_2d', 'X', 'Y', 'Phi'))

        qGrabDrinksWaypts = Conjunction(Compound('goal', Compound('serve', 'Drink')),
                                        Compound( 'property_expected', 'ObjectID', 'class_label', 'Drink'),
                                        Compound( 'property_expected', 'ObjectID', 'position', Compound('in_front_of', 'amigo')),
                                        Compound( 'property_expected', 'ObjectID', 'position', Sequence('X', 'Y', 'Z')))

        if grasp_arm == 'left':
            arm = robot.leftArm
        if grasp_arm == 'right':
            arm = robot.rightArm


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

            with lookoutContainer:

                smach.StateMachine.add( 'NAV_TO_LOOKOUT',
                                        NavToLookout(robot),
                                        transitions={   'unreachable':'NAV_TO_LOOKOUT',
                                                        'arrived':'DETECT_WAVING_PEOPLE',
                                                        'visited_all':'looked_all'})

                # Go to predefined lookout positions in the room and look arround to detect waving people
                smach.StateMachine.add( 'DETECT_WAVING_PEOPLE',
                                        DetectWavingPeople(robot),
                                        transitions={   'detected':'got_request',
                                                        'not_detected':'NAV_TO_LOOKOUT',
                                                        'error':'DETECT_WAVING_PEOPLE'})

            #add container to the iterator lookoutIterator
            smach.StateMachine.add( 'LOOKOUT_CONTAINER', 
                                    lookoutContainer, 
                                    transitions={   'looked_all':'GOTO_WAITING_PLACE',
                                                    'got_request':'ORDERS_CONTAINER'})


#----------------------------------------WAIT FOR PEOPLE TO SERVE (FALLBACK)-------------------------------------------------------


            # Go to waiting location if no one was detected previously (backup plan)
            smach.StateMachine.add('GOTO_WAITING_PLACE',
                                    NavigateGeneric(robot, goal_query = qWaitingPlace),
                                    transitions={   'arrived':'WAIT_PERSON_ITERATOR',
                                                    'unreachable':'WAIT_PERSON_ITERATOR', 
                                                    'preempted':'WAIT_PERSON_ITERATOR', 
                                                    'goal_not_defined':'WAIT_PERSON_ITERATOR'})

            # Iterate through Lookout Waypoints twice before giving up
            waitPersonIterator = smach.Iterator(outcomes=['timed_out', 'found_person'], 
                                                it = lambda: range(0, 3),
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
                                            remapping={     'waitIndexIn':'waitIndex'},
                                            transitions={   "waiting":"continue",
                                                            "unknown_person":"found_person"})

                #close waitPersonContainer
                smach.Iterator.set_contained_state( 'WAIT_PERSON_CONTAINER', 
                                                     waitPersonContainer, 
                                                     loop_outcomes=['continue'],
                                                     break_outcomes=['found_person'])

            # add the lookoutIterator to the main state machine
            smach.StateMachine.add( 'WAIT_PERSON_ITERATOR',
                                    waitPersonIterator,
                                    {   'timed_out':'ASSUMING_PERSON_FRONT',
                                        'found_person':'ORDERS_CONTAINER'})


            smach.StateMachine.add( 'ASSUMING_PERSON_FRONT',
                                    Say(robot, "I'm going to assume there is a person in front of me", block=False),
                                    transitions={   'spoken':'TAKE_NEW_ORDER_FOCUS'})

            smach.StateMachine.add( 'TAKE_NEW_ORDER_FOCUS',
                                    TakeNewOrderFocus(robot),
                                    transitions={'done':'CHECK_PENDING_ORDERS'})

#----------------------------------------TAKE THEIR ORDERS-------------------------------------------------------

            # create orders container
            ordersContainer = smach.StateMachine(  outcomes = ['succeeded','aborted'])

            with ordersContainer:

                smach.StateMachine.add( 'NAV_TO_PERSON',
                                        NavToPerson(robot),
                                        transitions={   'going':'NAV_TO_PERSON',
                                                        'arrived':'TAKE_NEW_ORDER',
                                                        'visited_all':'VISITED_ALL_PERSONS'})

                # Learn the persons name
                # smach.StateMachine.add( 'LEARN_PERSON_NAME',
                #                         LearnPersonName(robot),
                #                         remapping={     'personName_out':'personName'},
                #                         transitions={   'learned':'LEARN_PERSON_FACE',
                #                                         'failed':'LEARN_PERSON_NAME'})

                # Learn the persons face
                # smach.StateMachine.add( 'LEARN_PERSON_FACE',
                #                         LearnPersonFaceCustom(robot),
                #                         remapping={     'personName_in':'personName'},
                #                         transitions={   'face_learned':'TAKE_ORDER',
                #                                         'learn_failed':'LEARN_PERSON_FACE'})

                # Take the persons order
                # smach.StateMachine.add( 'TAKE_ORDER',
                #                         AskDrink(robot),
                #                         remapping={     'personName_in':'personName'},
                #                         transitions={   'done':'NAV_TO_PERSON',
                #                                         'failed':'TAKE_ORDER'})

                smach.StateMachine.add( 'TAKE_NEW_ORDER',
                                        TakeNewOrder(robot),
                                        transitions={'done':'NAV_TO_PERSON'})

                smach.StateMachine.add( 'VISITED_ALL_PERSONS',
                                        Say(robot, "I have taken all the orders from this group.", block=False),
                                        transitions={   'spoken':'succeeded'})

            # add orders container to the main state machine
            smach.StateMachine.add( 'ORDERS_CONTAINER',
                                    ordersContainer,
                                    transitions={   'succeeded':'CHECK_PENDING_ORDERS',
                                                    'aborted':'SAY_FAILED'})

            smach.StateMachine.add( 'CHECK_PENDING_ORDERS',
                                    PendingOrders(robot),
                                    transitions={   'enough_orders':'FIND_DRINKS_CONTAINER',
                                                    'insuficient_orders':'LOOKOUT_CONTAINER'})

#--------------------------------------------FIND THE REQUESTED DRINKS---------------------------------------------

            # create find drinks container
            findDrinksContainer = smach.StateMachine(  outcomes = ['succeeded','aborted'])

            with findDrinksContainer:

                # Go to storage room waypoint and look for the drinks
                smach.StateMachine.add( 'LOOK_FOR_DRINK',
                                        LookForDrink(robot),
                                        transitions={   'looking':'LOOK_FOR_DRINK',
                                                        'found':'PICKUP_DRINK',
                                                        'not_found':'SAY_DRINK_NOT_FOUND'})
                
                # Say drink not found
                smach.StateMachine.add( 'SAY_DRINK_NOT_FOUND',
                                        Say(robot, ["I could not find any of the drinks requested.", 
                                                    "I looked really hard, but I couldn't find the drinks i was looking for."]),
                                        transitions={   'spoken':'aborted' }) 

                # Determine which arm is available if any
                # smach.StateMachine.add( 'SET_AVAILABLE_ARM',
                #                         LookForDrink(robot),            # query console for available arm!
                #                         transitions={   'sucess':'STATE',
                #                                         'error':'SAY_DRINK_NOT_GRASPED'})

                # Pickup the drink
                smach.StateMachine.add( 'PICKUP_DRINK',
                                        GrabMachine(arm, robot, qStorageRoomWaypts),
                                        transitions={   'succeeded':'RETRACT_PICKED_DRINK',
                                                        'failed':'SAY_DRINK_NOT_GRASPED' }) 

                # Say drink not grasped
                smach.StateMachine.add( 'SAY_DRINK_NOT_GRASPED',
                                        Say(robot, ['I could not pick up the drink you wanted', 
                                                    'I failed to grab the object you wanted.']),
                                        transitions={   'spoken':'RETRACT_PICKED_DRINK' }) 

                smach.StateMachine.add( 'RETRACT_PICKED_DRINK',
                                        Retract_facts(robot, [Compound('drinksRequested', 'X')]),
                                        transitions={'retracted':'LOOK_FOR_DRINK'})

            # add find drinks container to the main state machine
            smach.StateMachine.add( 'FIND_DRINKS_CONTAINER',
                                    findDrinksContainer,
                                    transitions={   'succeeded':'SERVED_STATUS',
                                                    'aborted':'SERVED_STATUS'})


#-----------------------------------------DELIVER THE REQUESTED DRINKS---------------------------------------------

            # TODO: If the drinks being carried were all handed and less than 3 drinks have been served, 
            #   return to GOTO_PARTY_ROOM, else go to SAY_FINISHED_SERVING

            # smach.StateMachine.add( 'HUMAN_HANDOVER',
            #                         Human_handover(arm,robot),
            #                         transitions={   'succeeded':'RETRACT_SERVED_PERSON',
            #                                         'failed':'GOTO_INITIAL_FAIL' })

            # smach.StateMachine.add( 'RETRACT_SERVED_PERSON',
            #                         Retract_facts(robot, [Compound('visited', 'X')]),
            #                         transitions={'retracted':'LOOK_FOR_PERSON'})           

            #Go to the last person known location
            # smach.StateMachine.add('GO_TO_LAST_KNOWN_LOCATION',
            #                         NavigateGeneric(robot, goal_query = qPeopleWaving),
            #                         transitions={   'arrived':'LEARN_PERSON_NAME', 
            #                                         'unreachable':'order_not_taken', 
            #                                         'preempted':'order_not_taken'})

            # smach.StateMachine.add( 'LOOK_FOR_PERSON',
            #                         LookForPerson(robot),
            #                         transitions={   'looking':'LOOK_FOR_PERSON',
            #                                         'found':'PERSON_FOUND',
            #                                         'not_found':'SAY_PERSON_NOT_FOUND'})
            # smach.StateMachine.add( 'PERSON_FOUND',
            #                         PersonFound(robot),
            #                         transitions={   'correct':'HANDOVER_DRINK',
            #                                         'unknown':'LOOK_FOR_PERSON',
            #                                         'persons_unchecked':'PERSON_FOUND',
            #                                         'not_correct':'LOOK_FOR_PERSON'})

            # smach.StateMachine.add( 'SAY_PERSON_NOT_FOUND',
            #                         Say(robot, ['I could not find you.', 
            #                                     'I can't find you. I really don't like fluids.',
            #                                     'I could not find you.']),
            #                         transitions={   'spoken':'HANDOVER_DRINK_UNKNOWN_PERSON' }) #GOTO_INITIAL_FAIL

            # smach.StateMachine.add( 'HANDOVER_DRINK_UNKNOWN_PERSON',
            #                         HandoverToUnknownHuman(robot),
            #                         transitions={'done':'GOTO_INITIAL_FAIL'})

            # smach.StateMachine.add( 'HANDOVER_DRINK',
            #                         HandoverToKnownHuman(robot),
            #                         transitions={'done':'GOTO_INITIAL_SUCCESS'})

            # smach.StateMachine.add( 'RETRACT_SERVED_PERSON',
            #                         Retract_facts(robot, [Compound('visited', 'X')]),
            #                         transitions={'retracted':'LOOK_FOR_PERSON'})           
            

            # close the lookoutIterator
            smach.StateMachine.add( 'SERVED_STATUS',
                                    ServedStatus(robot),
                                    transitions={   'complete':'SAY_FINISHED_SERVING',
                                                    'incomplete':'LOOKOUT_CONTAINER'})


 #-------------------------------------------FINISH THE CHALLENGE AND LEAVE----------------------------------------

            # Say the job is complete
            smach.StateMachine.add('SAY_FINISHED_SERVING',
                                    Say(robot, "I finished my task, enjoy your drinks", block = False),
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

    # Number of people served
    amigo.reasoner.query(Compound("retractall", Compound("n_people_served", "X")))
    amigo.reasoner.query(Compound("assert",Compound("n_people_served", "0")))

    amigo.reasoner.query(Compound("retractall", Compound("preempt_head_focus", "X")))
    amigo.reasoner.query(Compound("assert",Compound("preempt_head_focus", "0")))

    #################################

    # Retract all old facts
    amigo.reasoner.query(Compound("retractall", Compound("challenge", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("goal", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("explored", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
    amigo.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("current_object", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("current_person", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("visited", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("registered", "X")))
    amigo.reasoner.query(Compound("retractall", Compound("type", "X", "Y")))

    amigo.reasoner.query(Compound("retractall", Compound("curLook", "X", "Y", "Phi")))

    # Load locations and objects from knowledge files
    amigo.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
    amigo.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))

    # Use reasoner for max iter in LookForPerson 
    amigo.reasoner.query(Compound("retractall", Compound("looked_person_no", "X")))
    amigo.reasoner.query(Compound("assert",Compound("looked_person_no", "0")))

    # Use reasoner for max iter in LookForDrink 
    amigo.reasoner.query(Compound("retractall", Compound("looked_drink_no", "X")))
    amigo.reasoner.query(Compound("assert",Compound("looked_drink_no", "0")))

    # Assert current challenge
    amigo.reasoner.assertz(Compound("challenge", "cocktailparty"))

    # initial_state = None
    initial_state = "ASSUMING_PERSON_FRONT"

    machine = CocktailParty(amigo)
    
    if initial_state != None:
        machine.set_initial_state([initial_state])

    introserver = smach_ros.IntrospectionServer('SM_TOP', machine, '/SM_ROOT_PRIMARY')
    introserver.start()

    try:
        outcome = machine.execute()
    except Exception, e:
        amigo.speech.speak(e)
    finally:
        introserver.stop()
