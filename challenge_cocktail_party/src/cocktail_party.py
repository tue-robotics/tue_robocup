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


MIN_SIMULTANEOUS_ORDERS = 3   # Amigo won't fetch the drinks until he has this amount of requests
MAX_SIMULTANEOUS_ORDERS = 3   # Amigo will imediatly fetch the drinks when he has reached this amount of requests
TOTAL_ORDERS = 3              # The challenge will finish when Amigo has served this amount of requests


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

# Uses the human_tracking to detect people while rotating the head. Wave not implemented yet.
class DetectWavingPeople(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['detected', 'not_detected', 'error'])
        
        self.robot = robot

        # compose person query
        self.peopleFoundQ = Conjunction(Compound('property_expected', 'ObjectID', 'class_label', 'validated_person'),
                                        Compound('not', Compound('visited', 'ObjectID')))

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: DetectWavingPeople\n")

        self.robot.head.reset_position()
        self.robot.speech.speak("Please wave to call me")

        # turn head to one side to start the swipping the room
        self.robot.head.set_pan_tilt(pan=-1.1, tilt=0.0, timeout=3.0)
        
        # Turn ON Human Tracking
        self.response_start = self.robot.perception.toggle(['human_tracking'])

        if self.response_start.error_code == 0:
            rospy.loginfo("human_tracking turned on")
        elif self.response_start.error_code == 1:
            rospy.logwarn("human_tracking failed to start")
            self.robot.speech.speak("I was not able to start human tracking.")
            return "error"


        self.robot.head.set_pan_tilt(pan=0.0, pan_vel=0.1, tilt=0.0, timeout=3.0)

        self.robot.head.set_pan_tilt(pan=1.1, pan_vel=0.1, tilt=0.0, timeout=5.0)

        # Turn OFF Human Tracking
        self.response_stop = self.robot.perception.toggle([])

        if self.response_stop.error_code == 0:
            rospy.loginfo("human_tracking turned off")
        elif self.response_stop.error_code == 1:
            rospy.logwarn("human_tracking failed to shutdown")
            self.robot.speech.speak('I was not able to stop human tracking.')
            return 'error'

        # sleep to give enough time for the human tracking to stop before moving the head again
        rospy.sleep(1)

        self.robot.head.reset_position()

        # get results from the query
        peopleFound = self.robot.reasoner.query(self.peopleFoundQ)
        
        if peopleFound:
            self.robot.speech.speak("Someone is calling me, I will be with you soon! {0}".format(len(peopleFound)), block=False)
            return 'detected'
        else:
            self.robot.speech.speak("No one called me.", block=False)
            return 'not_detected'


#########################################################################################


# Uses the human_tracking to detect people while turning the head arround.
class DetectPeople(smach.State):
    def __init__(self, robot, distance_to_walls=0.2, time=None, room=None):
        smach.State.__init__(   self, 
                                outcomes=["detected", "not_detected", "error"])
        
        self.robot = robot
        self.distance_to_walls = distance_to_walls  # distance to inflate the room walls
        self.time = time
        self.room = room
        # compose person query
        self.peopleDetectedQ = Conjunction( Compound('property_expected', 'ObjectID', 'class_label', 'validated_person'),
                                            Compound('property_expected', 'ObjectID', 'position', Sequence('X','Y','Z')),
                                            Compound('not', Compound('approached', 'ObjectID')))

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: DetectPeople\n")

        # Get the coordinates of the room
        if self.room:
            
            roomDimensionsQ = Compound('room_dimensions',self.room, Compound('size', 'Xmin', 'Ymin', 'Zmin', 'Xmax', 'Ymax', 'Zmax'))

            room_dimensions = self.robot.reasoner.query(roomDimensionsQ)

            if not room_dimensions:
                rospy.logerr("\t\t[Cocktail Party] Dimensions for room were not found in reasoner!\n")


            # assumed is that there is only one block per room.
            room_dimensions_answer = room_dimensions[0]
            x_min = float(room_dimensions_answer['Xmin'])
            y_min = float(room_dimensions_answer['Ymin'])
            z_min = float(room_dimensions_answer['Zmin'])
            x_max = float(room_dimensions_answer['Xmax'])
            y_max = float(room_dimensions_answer['Ymax'])
            z_max = float(room_dimensions_answer['Zmax'])

            rospy.loginfo("\t\t[Cocktail Party] room dimensions: X - [{0}, {1}], Y - [{2}, {3}], Z - [{4}, {5}]\n".format(
                x_min, x_max, y_min, y_max, z_min, z_max))


        self.robot.spindle.high()
        
        # turn head to one side to start the swipping the room
        self.robot.head.set_pan_tilt(pan=-0.7, tilt=0.0, timeout=3.0)

        # sleep to make sure the spindle is at the target position
        rospy.sleep(0.8)
        
        # Turn ON Human Tracking
        self.response_start = self.robot.perception.toggle(['human_tracking'])

        if self.response_start.error_code == 0:
            rospy.loginfo("human_tracking turned on")
        elif self.response_start.error_code == 1:
            rospy.loginfo("human_tracking failed to start")
            self.robot.speech.speak("I was not able to start human tracking.")
            return "error"

        # TODO check color
        self.robot.lights.set_color(0, 0, 1)

        # sleep to give time for the tracking to start
        rospy.sleep(0.5)

        # look right
        self.robot.head.set_pan_tilt(pan=0.0, pan_vel=0.1, tilt=0.0, timeout=3.0)
        # look left
        self.robot.head.set_pan_tilt(pan=0.7, pan_vel=0.1, tilt=0.0, timeout=5.0)

        # Turn OFF Human Tracking
        self.response_stop = self.robot.perception.toggle([])

        if self.response_stop.error_code == 0:
            rospy.loginfo("human_tracking turned off")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("human_tracking failed to shutdown")
            self.robot.speech.speak("I was not able to stop human tracking.")
            return "error"

        # sleep to give the tracking time to stop before reseting the head
        rospy.sleep(0.5)
        
        self.robot.head.reset_position()

        ###################################################
        # Check the results from the laser people detection

        # laserDetectQ = Conjunction( Compound( "property_expected", "ObjectID", "class_label", "person"),
        #                             Compound( "property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        # peopleResult = self.robot.reasoner.query(laserDetectQ)

        # if not peopleResult:
        #     self.robot.speech.speak("Laser detection didn't find anything.", block=False)
        # else:
        #     self.robot.speech.speak("Laser detection found something. {0}".format(len(peopleResult)), block=False)

        ###################################################

        # get results from the query
        peopleFound = self.robot.reasoner.query(self.peopleDetectedQ)
        
        if peopleFound:
            rospy.loginfo("\t\t[Cocktail Party] Found {0} person(s) (unfiltered)\n".format(len(peopleFound)))

            # assert people detected outside the room as visited
            for candidate in peopleFound:
                rospy.loginfo("\t\t[Cocktail Party] Person found at ({0},{1},{2})\n".format(candidate['X'], candidate['Y'], candidate['Z']))
                xPos, yPos, zPos = float(candidate['X']), float(candidate['Y']), float(candidate['Z'])
                # check if the person found is inside or outside of the room
                if not (xPos > x_min and xPos < x_max and
                        yPos > y_min and yPos < y_max and
                        zPos > z_min and zPos < z_max):

                    rospy.logwarn("\t\t[Cocktail Party] Person outside of the room!\n")

                    self.robot.reasoner.query(Compound('assert', Compound('visited', candidate['ObjectID'])))
                    self.robot.reasoner.query(Compound('assert', Compound('approached', candidate['ObjectID'])))
                else:
                    rospy.loginfo("\t\t[Cocktail Party] Person inside the room!\n")


            peopleFoundROI = self.robot.reasoner.query(self.peopleDetectedQ)

            if peopleFoundROI:
                rospy.loginfo("\t\t[Cocktail Party] Found {0} person(s) (filtered)\n".format(len(peopleFoundROI)))
                self.robot.speech.speak("I think i saw someone.", mood='excited', block=False)
                
                return 'detected'
            else:
                self.robot.speech.speak("There's no one here.", block=False)
                return 'not_detected'

        else:
            self.robot.speech.speak("There's no one here.", block=False)
            return 'not_detected'


#########################################################################################


# Wait for the person to step in front of the robot, uses the human_tracking node.
class WaitForPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['waiting', 'unknown_person'],
                                input_keys=['waitIndex_in'])
        
        self.robot = robot
        self.sentenceCounter = 0

        # self.faceInFrontQ = Conjunction(Compound('property_expected', 'ObjectID', 'class_label', 'face'),
        #                                 Compound('property_expected', 'ObjectID', 'position', Compound('in_front_of', 'amigo')),
        #                                 Compound('property_expected', 'ObjectID', 'position', Sequence('X','Y','Z')))

        self.faceInFrontQ = Conjunction(Compound('property_expected', 'ObjectID', 'class_label', 'human_face'),
                                        Compound('property_expected', 'ObjectID', 'position', Compound('in_front_of', 'amigo')),
                                        Compound('property_expected', 'ObjectID', 'position', Sequence('X','Y','Z')))

    def execute(self, userdata):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: WaitForPerson\n")

        waitedCount = userdata.waitIndex_in

        if waitedCount == 3:
            # Reset the counter for waiting
            waitedCount = 0;
            self.robot.speech.speak("I didn't see anyone in front of me.")
            return "unknown_person"
        else:
            rospy.loginfo("Waited for {0} times!!!".format(waitedCount))

        # set pose for close interaction with people
        self.robot.reasoner.reset()
        self.robot.spindle.high()
        self.robot.head.set_pan_tilt(tilt=-0.2, pan=0.0)
        
        # say difference sentences 
        if self.sentenceCounter == 0:
            self.robot.speech.speak("Please step in front of me to order your drink.")
        elif self.sentenceCounter == 1:
            self.robot.speech.speak("Would another person stand in front of me to order.")
            # give time for the last person to move away
            rospy.sleep(1.0)
        else:
            self.robot.speech.speak("Does someone else want to order? Please come to me.")
            # give time for the last person to move away
            rospy.sleep(1.0)
            self.sentenceCounter = 0

        # TODO check color
        self.robot.lights.set_color(0, 0, 1)
        
        # start face segmentation node
        self.response_start = self.robot.perception.toggle(['human_tracking'])

        if self.response_start.error_code == 0:
            rospy.loginfo("Human Tracking has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Human Tracking failed to start")
            self.robot.speech.speak("I was not able to start Human Tracking.")
            return "waiting"

        # start face segmentation node
        # self.response_start = self.robot.perception.toggle(['face_segmentation'])

        # if self.response_start.error_code == 0:
        #     rospy.loginfo("face_segmentation turned on")
        # elif self.response_start.error_code == 1:
        #     rospy.logwarn("face_segmentation failed to start")
        #     self.robot.speech.speak("I was not able to start face segmentation.")

        # wait until the query detected person is true, or 10 second timeout
        wait_machine = Wait_query_true(self.robot, self.faceInFrontQ, 15)
        wait_result = wait_machine.execute()

        # TODO check color
        self.robot.lights.set_color(1, 0, 0)
        
        # TODO not sure if this is actually working
        # Do a sleep with the node still on so that the location of the face is more correct since the person was moving
        rospy.sleep(1.5)

        # turn off face segmentation
        rospy.loginfo("Human Tracking will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Human Tracking is stopped")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping Human Tracking")

        # turn off face segmentation
        # rospy.loginfo("Face Segmentation will be stopped now")
        # self.response_stop = self.robot.perception.toggle([])
        
        # if self.response_stop.error_code == 0:
        #     rospy.loginfo("Face Segmentation is stopped")
        # elif self.response_stop.error_code == 1:
        #     rospy.logwarn("Failed stopping Face Segmentation")

        # if the query timed out...
        if wait_result == "timed_out":
            self.robot.speech.speak("Please, don't keep me waiting.", mood='excited', block=False)
            waitedCount += 1
            return "waiting"
        elif wait_result == "preempted":
            self.robot.speech.speak("Waiting for person was preemted.", block=False)
            return "waiting"
        # if the query succeeded
        elif wait_result == "query_true":
            rospy.loginfo("\t\t[Cocktail Party] A person was seen in front of Amigo\n")

            self.sentenceCounter +=1

            result = self.robot.reasoner.query(self.faceInFrontQ)

            if result:
                # TODO: SELECT THE ONE THATS CLOSEST TO AMIGO, NOT JUST THE FIRST
                x,y,z = result[0]['X'],result[0]['Y'],result[0]['Z']

                lookat_point = msgs.PointStamped(x,y,z)

                rospy.loginfo("\t\t[Cocktail Party] Looking at person's face at ({0}, {1}, {2})\n".format(x, y, z))
                self.robot.head.send_goal(lookat_point, timeout=0)
            else: 
                rospy.logwarn('\t\t[Cocktail Party] No face found in front of amigo\n')

            return "unknown_person"


#########################################################################################


# Double-check if there is a face in front of the robot, using face_segmentation, and look at it
class ConfirmPersonDetection(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['found', 'not_found', 'error'])
        
        self.robot = robot

        # prepare query for detected person
        self.detectPersonQ = Conjunction(   Compound('property_expected', 'ObjectID', 'class_label', 'face'),
                                            Compound('property_expected', 'ObjectID', 'position', Compound('in_front_of', 'amigo')),
                                            Compound('property_expected', 'ObjectID', 'position', Sequence('X','Y','Z')),
                                            # Compound('not', Compound('confirmed', 'ObjectID'))
                                            )
    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: ConfirmPersonDetection\n")

        # reset robo pose
        self.robot.spindle.high()
        self.robot.head.set_pan_tilt(tilt=-0.2, pan=0.0)

        self.robot.speech.speak("Let me make sure there's someone here.", block=False)

        # TODO use the human_tracking instead of face_segmentation, its much faster

        # start face segmentation node
        self.response_start = self.robot.perception.toggle(['face_segmentation'])

        if self.response_start.error_code == 0:
            rospy.loginfo("face_segmentation turned on")
        elif self.response_start.error_code == 1:
            rospy.logwarn("face_segmentation failed to start")
            self.robot.speech.speak("I was not able to start face segmentation.")

        # wait until the query detected person is true, or 10 second timeout
        wait_machine = Wait_query_true(self.robot, self.detectPersonQ, 6)
        # TODO TEST IF I CAN REDUCE THIS TIME
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
            rospy.loginfo("\t\t[Cocktail Party] No faces were found in front of Amigo.\n")
            return 'not_found'
        elif wait_result == 'preempted':
            self.robot.speech.speak("Waiting for person was preemted.", block=False)
            return 'not_found'
        # if the query succeeded
        elif wait_result == 'query_true':
            rospy.loginfo("\t\t[Cocktail Party] Person found!\n")
            
            result = self.robot.reasoner.query(self.detectPersonQ)

            if result:
                # SELECT THE ONE THATS CLOSEST TO AMIGO, NOT JUST THE FIRST
                x,y,z = result[0]['X'],result[0]['Y'],result[0]['Z']

                lookat_point = msgs.PointStamped(x,y,z)

                rospy.loginfo("\t\t[Cocktail Party] Looking at person's face at ({0}, {1}, {2})\n".format(x, y, z))
                self.robot.head.send_goal(lookat_point, timeout=0)
            else: 
                rospy.logwarn('\t\t[Cocktail Party] No face found in front of amigo\n')
            
            # objectID = result[0]['ObjectID']
            # self.robot.reasoner.query(Compound('assert', Compound('confirmed', objectID)))
            return 'found'


#########################################################################################


# Ask the persons name, if nothing is heard for several times the robot chooses one
class LearnPersonName(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['learned', 'failed'],
                                output_keys=['personName_out'])

        self.robot = robot
        self.ask_user_service_get_learn_person_name = rospy.ServiceProxy('interpreter/ask_user', AskUser)
        self.person_learn_failed = 0

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: LearnPersonName\n")
        
        # ask the name of the user (within 3 tries and within 60 seconds an answer is received)
        self.response = self.ask_user_service_get_learn_person_name('name', 3, rospy.Duration(60))
            
        # test if the name is allowed / exists
        for x in range(0,len(self.response.keys)):
            if self.response.keys[x] == 'answer':
                response_answer = self.response.values[x]

        # if no answer was found / unsupported name
        if response_answer == 'no_answer' or response_answer == 'wrong_answer':
            if self.person_learn_failed == 0:
                self.robot.speech.speak("I will call you Richard", block=False)
                response_answer = "richard"
            if self.person_learn_failed == 1:
                self.robot.speech.speak("I will call you Jennifer", block=False)
                response_answer = "jennifer"
            if self.person_learn_failed == 2:
                self.robot.speech.speak("I will call you Charles", block=False)
                response_answer = "charles"
            if self.person_learn_failed == 3:
                self.robot.speech.speak("I will call you Kimberly", block=False)
                response_answer = "kimberly"
            if self.person_learn_failed == 4:
                self.robot.speech.speak("I will call you Luis", block=False)
                response_answer = "luis"

            self.person_learn_failed += 1
        else:
             self.person_learn_failed = 0

        userdata.personName_out = str(response_answer)
        self.robot.speech.speak("Hello " + str(response_answer) + "!", mood='excited', block=False)

        # Person's name successfully learned
        return 'learned'


#########################################################################################


# Ask the persons drink, if nothing is heard for several times the robot chooses one
class AskDrink(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['done', 'failed'],
                                input_keys=['personName_in'])

        self.ask_user_service_get_drink = rospy.ServiceProxy('interpreter/ask_user', AskUser)
        self.robot = robot
        self.person_learn_failed = 0
        self.drink_learn_failed = 0

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: AskDrink\n")

        # ask the name of the drink
        self.response = self.ask_user_service_get_drink('drink_cocktail', 3 , rospy.Duration(60))  # This means that within 3 tries and within 60 seconds an answer is received. 
            
        # determine if the answer is allowed
        for x in range(0,len(self.response.keys)):
            if self.response.keys[x] == 'answer':
                response_answer = self.response.values[x]

        # if the answer is not allwed just assume another possible drink
        if response_answer == 'no_answer' or  response_answer == 'wrong_answer':
            if self.drink_learn_failed == 2:
                self.robot.speech.speak("I will just bring you a Orange Juice")
                response_answer = 'orange_juice'
                self.drink_learn_failed = 3
            elif self.drink_learn_failed == 1:
                self.robot.speech.speak("I will just bring you a Energy Drink")
                response_answer = "energy_drink"
                self.drink_learn_failed = 2
            elif self.drink_learn_failed == 0:
                self.robot.speech.speak("I will just bring you a water")
                response_answer = 'water'
                self.drink_learn_failed = 1
            else:
                self.robot.speech.speak("I will just bring you a cola")
                response_answer = 'cola'

        # get Amigo's current location and rotation
        amigoPose = self.robot.base.location
        pos, rot = amigoPose.pose.position, amigoPose.pose.orientation
        rotation = transformations.euler_z_from_quaternion(rot)

        locX = amigoPose.pose.position.x
        locY = amigoPose.pose.position.y
        locZ = amigoPose.pose.position.z
        locPhi = rotation

        uniqueID = "{0}_{1}".format(userdata.personName_in, response_answer)
        lastLocID = "{0}_{1}_{2}".format(locX, locY, locPhi)
        
        # save the requested drink
        self.robot.reasoner.query(  Compound('assert', 
                                    Compound('goal',
                                    Compound('serve', uniqueID, userdata.personName_in, response_answer, Compound('pose_2d', locX, locY, rotation)))))

        self.robot.reasoner.query(  Compound('assert', 
                                    Compound('waypoint', Compound('last_known_location', lastLocID), Compound('pose_2d', locX, locY, rotation))))

        rospy.loginfo("\t\t[Cocktail Party] I'm getting a {0} for {1}".format(response_answer,userdata.personName_in))

        return 'done'


#########################################################################################


# Learn the persons face
class LearnPersonFace(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["face_learned" , "learn_failed"],
                                input_keys=['personName_in'])

        self.robot = robot

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: LearnPersonFace\n")

        # learn the face of the person
        learn_machine = Learn_Person(self.robot, userdata.personName_in)
        learn_result = learn_machine.execute()

        if learn_result == 'face_learned':
            rospy.loginfo("Face learning succeeded")
        elif learn_result == 'learn_failed':
            rospy.logwarn("Failed learning face. Just continue to the next state and ask drink.")
        return learn_result


#########################################################################################


# Navigate to a lookout point. When reached assert it as visited so the robot can iterate through several points
class NavToLookout(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["unreachable" , "arrived", "visited_all"])
        self.robot = robot

        self.lookoutPointsQ = Conjunction(  Compound("=", "Waypoint", Compound("party_room_lookout", "W")),
                                            Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                            Compound("not", Compound("visited", "Waypoint")))

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: NavToLookout\n")

        # get the waypoint of where to search, party_room_lookout
        lookoutPoints = self.robot.reasoner.query(self.lookoutPointsQ)

        # if there is no location associated with lookout points say it
        if not lookoutPoints:
            rospy.loginfo("\t\t[Cocktail Party] Visited all lookout points\n")
            return "visited_all"

        self.robot.speech.speak("Going to the next location.", block=False)

        # take the first goal found
        goal_answer = lookoutPoints[0]

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypointName = goal_answer["Waypoint"]

        # navigate to the waypoint queried
        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        # assert that this location has been visited
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypointName)))

        # If nav_result is unreachable DO NOT stop looking, there are more options, return not_found when list of Waypoints is empty
        if nav_result == "unreachable":
            rospy.logwarn("\t\t[Cocktail Party] Could not go to the lookout location.\n")
            self.robot.speech.speak("I'm unable to go to the lookout place.", block=False)
            return "unreachable"
        elif nav_result == "preempted":
            rospy.logwarn("\t\t[Cocktail Party] Could not go to the lookout location.\n")
            self.robot.speech.speak("I'm unable to go to the lookout place.", block=False)
            return "unreachable"

        return "arrived"


#########################################################################################


# Navigate to a waiting point. When reached assert it as visited so the robot can iterate through several points
class NavToWaitingLoc(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["unreachable" , "arrived", "visited_all"])
        self.robot = robot

        self.waitingLocQ = Conjunction( Compound("=", "Waypoint", Compound("party_room_waiting", "W")),
                                        Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                        Compound("not", Compound("visited", "Waypoint")))

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: NavToWaitingLoc\n")

        # get the waypoint of where to search, party_room_lookout
        waitingLocs = self.robot.reasoner.query(self.waitingLocQ)

        # if there is no location associated with lookout points say it
        if not waitingLocs:
            rospy.loginfo("\t\t[Cocktail Party] Visited all waiting places.\n")
            return "visited_all"

        self.robot.speech.speak("Going to the waiting location.", block=False)

        # for now, take the first goal found
        goal_answer = waitingLocs[0]

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypointName = goal_answer["Waypoint"]

        # navigate to the waypoint queried
        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        # assert that this location has been visited
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypointName)))

        if nav_result == "unreachable":
            rospy.logwarn("\t\t[Cocktail Party] Could not go to the waiting location.\n")
            self.robot.speech.speak("I'm unable to go to the waiting place.", block=False)
            return "unreachable"
        elif nav_result == "preempted":
            rospy.logwarn("\t\t[Cocktail Party] Could not go to the waiting location.\n")
            self.robot.speech.speak("I'm unable to go to the waiting place.", block=False)
            return "unreachable"

        return "arrived"


#########################################################################################


class NavResetLocRoom(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["unreachable" , "arrived", "visited_all"])
        self.robot = robot

        self.resetLocRoomQ = Conjunction( Compound("=", "Waypoint", Compound("failed_nav_room", "W")),
                                        Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                        Compound("not", Compound("visited", "Waypoint")))

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: NavResetLocRoom\n")

        # Reset stuff for a clean retry
        self.robot.base.reset_costmap()
        self.robot.reasoner.reset()

        # get the waypoint of where to search, party_room_lookout
        resetLocs = self.robot.reasoner.query(self.resetLocRoomQ)

        # if there is no location associated with lookout points say it
        if not resetLocs:
            rospy.loginfo("\t\t[Cocktail Party] Visited all Living Room reset places.\n")
            self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
            return "visited_all"

        # for now, take the first goal found
        goal_answer = resetLocs[0]

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypointName = goal_answer["Waypoint"]

        # navigate to the waypoint queried
        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        # assert that this location has been visited
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypointName)))

        if nav_result == "unreachable":
            rospy.logwarn("\t\t[Cocktail Party] Could not go to the reset location.\n")
            self.robot.speech.speak("I'm unable to even go to the reset location.", block=False)
            amigo.base.reset_costmap()
            return "unreachable"
        elif nav_result == "preempted":
            rospy.logwarn("\t\t[Cocktail Party] Could not go to the reset location.\n")
            self.robot.speech.speak("I'm unable to even go to the reset location.", block=False)
            amigo.base.reset_costmap()
            return "unreachable"

        self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
        return "arrived"


#########################################################################################


class NavResetLocKitchen(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["unreachable" , "arrived", "visited_all"])
        self.robot = robot

        self.resetLocKitchenQ = Conjunction( Compound("=", "Waypoint", Compound("failed_nav_kitchen", "W")),
                                        Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                        Compound("not", Compound("visited", "Waypoint")))

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: NavResetLocKitchen\n")

        # Reset stuff for a clean retry
        amigo.base.reset_costmap()
        amigo.reasoner.reset()

        # get the waypoint of where to search, party_room_lookout
        resetLocs = self.robot.reasoner.query(self.resetLocKitchenQ)

        # if there is no location associated with lookout points say it
        if not resetLocs:
            rospy.loginfo("\t\t[Cocktail Party] Visited all Kitchen reset places.\n")
            self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
            return "visited_all"

        self.robot.speech.speak("Going to the waiting location.", block=False)

        # for now, take the first goal found
        goal_answer = resetLocs[0]

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypointName = goal_answer["Waypoint"]

        # navigate to the waypoint queried
        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        # assert that this location has been visited
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypointName)))

        if nav_result == "unreachable":
            rospy.logwarn("\t\t[Cocktail Party] Could not go to the reset location.\n")
            self.robot.speech.speak("I'm unable to even go to the reset location.", block=False)
            amigo.base.reset_costmap()
            return "unreachable"
        elif nav_result == "preempted":
            rospy.logwarn("\t\t[Cocktail Party] Could not go to the reset location.\n")
            self.robot.speech.speak("I'm unable to even go to the reset location.", block=False)
            amigo.base.reset_costmap()
            return "unreachable"

        self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
        return "arrived"


#########################################################################################


# Navigate to the location of a detected person. When reached assert it as visited so the robot can iterate through several people
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
        
        # In case Amigo still has more people to visit but cannot take more orders
        if len(goals) >= MAX_SIMULTANEOUS_ORDERS:
            rospy.loginfo("\t\t[Cocktail Party] Maximum number of simultaneous orders reached ({0})\n". format(len(goals)))
            return 'max_orders'

        # compose person query
        peopleWavingQ = Conjunction(Compound('property_expected', 'ObjectID', 'class_label', 'validated_person'),
                                    Compound('property_expected', 'ObjectID', 'position', Sequence('X','Y','Z')),
                                    Compound('not', Compound('visited', 'ObjectID')),
                                    Compound('not', Compound('ordered', 'ObjectID')))

        # get results from the query
        peopleWaving = self.robot.reasoner.query(peopleWavingQ)


        # if there is no location associated with lookout points
        if not peopleWaving:
            rospy.loginfo("\t\t[Cocktail Party] Visited all the persons\n")
            return 'visited_all'

        else:
            rospy.loginfo("\t\t[Cocktail Party] Found {0} person(s). Going to visit the first in the list.\n". format(len(peopleWaving)))

            # If there is more than 1 result, force a query that only returns the first result
            #   otherwise i don't know which one the nav generic will choose
            if len(peopleWaving) > 1:
                selectedID = peopleWaving[0]["ObjectID"]

                peopleWavingQ = Conjunction(Compound('property_expected', selectedID, 'class_label', 'validated_person'),
                                            Compound('property_expected', selectedID, 'position', Sequence('X','Y','Z')),
                                            Compound('not', Compound('visited', selectedID)),
                                            Compound('not', Compound('ordered', selectedID)))
                objectID = selectedID
            else:
                objectID = peopleWaving[0]["ObjectID"]

            # Use the lookat query
            nav = NavigateGeneric(self.robot, lookat_query=peopleWavingQ)
            navResult = nav.execute()

            # assert that this location has been visited
            rospy.loginfo("\t\t[Cocktail Party] Asserting as visited: {0}\n".format(objectID))
            self.robot.reasoner.query(Compound('assert', Compound('visited', objectID)))
            self.robot.reasoner.query(Compound('assert', Compound('ordered', objectID)))

            # If nav_result is unreachable DO NOT stop looking, there are more options
            if navResult == 'unreachable':                    
                return 'going'
            elif navResult == 'preempted':
                return 'going'
            else:
                return 'arrived'


#########################################################################################

# Navigate to the last know location of a person who ordered.
class NavToLastKnowLoc(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['unreachable' , 'arrived', 'visited_all'])
        
        self.robot = robot
        self.resetVisited = 0

        self.lastLocQ = Conjunction(Compound('=', 'Waypoint', Compound('last_known_location', 'ID')),
                                    Compound('waypoint', 'Waypoint', Compound('pose_2d', 'X', 'Y', 'Phi')),
                                    Compound('not', Compound('visited', 'Waypoint')))

    def execute(self, userdata=None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: NavToLastKnowLoc\n")

        lastLoc = self.robot.reasoner.query(self.lastLocQ)
        
        # reset visited pepople if all the last know locations have been visited
        if not lastLoc:
            rospy.loginfo("\t\t[Cocktail Party] Visited all the last know locations\n")
            
            if self.resetVisited == 0:
                self.robot.reasoner.reset()
                amigo.reasoner.query(Compound('retractall', Compound('approached', 'X')))
                self.resetVisited += 1

            return 'visited_all'
        else:    
            self.resetVisited = 0

            self.robot.speech.speak("Going to the last place where I saw people.", block=False)

            # take the first goal found
            # waypointName = lastLoc[0]['Waypoint']

            goal = (float(lastLoc[0]['X']), float(lastLoc[0]['Y']), float(lastLoc[0]['Phi']))

            # nav = NavigateGeneric(self.robot, lookat_query = self.lastLocQ)
            nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
            nav_result = nav.execute()

            # assert that this location has been visited
            # self.robot.reasoner.query(Compound('assert', Compound('visited', waypointName)))
            
            # changed this to assert all at once, remove this if Amigo goes to the people, instead of the reverse
            for waypoint in lastLoc:
                self.robot.reasoner.query(Compound('assert', Compound('visited', waypoint["Waypoint"])))

            # If nav_result is unreachable DO NOT stop looking, there are more options, return not_found when list of Waypoints is empty
            if nav_result == "unreachable":                    
                return "unreachable"
            elif nav_result == "preempted":
                return "unreachable"

            return "arrived"


#########################################################################################


# Check how many people have been successfully and determine if the challenge is compelte or not
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
        
        rospy.loginfo("\t\t[Cocktail Party] Already served {0} person(s)\n". format(nServed))

        # if there were enough people served, finish the challenge
        if nServed < TOTAL_ORDERS:
            # retract most of the facts
            self.robot.reasoner.query(Compound('retractall', Compound('goal', 'X')))
            self.robot.reasoner.query(Compound('retractall', Compound('carrying', 'X')))
            self.robot.reasoner.query(Compound('retractall', Compound('ordered', 'X')))
            self.robot.reasoner.query(Compound('retractall', Compound('approached', 'X')))
            self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
            self.robot.reasoner.query(Compound('retractall', Compound('confirmed', 'X')))
            self.robot.reasoner.query(Compound('retract', Compound("waypoint", Compound("last_known_location", "X"), "Y")))

            self.robot.reasoner.reset()

            rospy.loginfo("\t\t[Cocktail Party] Need to serve {0} more person(s) to finish the challenge\n". format(TOTAL_ORDERS-nServed))
            self.robot.speech.speak("I still need to serve more people.", block=False)
            return "incomplete"
        else:
            return "complete"


#########################################################################################


# Check how many orders have been made
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
            self.robot.speech.speak("I need more requests before getting the drinks.", block=False)
            return "insuficient_orders"
        else:
            # retract the people already served, maybe they want something again
            self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
            self.robot.reasoner.query(Compound('retractall', Compound('ordered', 'X')))
            self.robot.reasoner.query(Compound('retractall', Compound('approached', 'X')))
            self.robot.reasoner.query(Compound('retractall', Compound('confirmed', 'X')))
            
            self.robot.speech.speak("Please step aside so I can go and get the drinks.")
            rospy.sleep(1.0)
            
            self.robot.reasoner.reset()
            return "enough_orders"  


#########################################################################################


# Navigate to the storage room and see what drinks are there
class LookForDrinks(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=["looking" , "found", "visited_all", "reset_at_room", "reset_at_kitchen", "done"])

        self.robot = robot

        self.arrivedAtKitchen = False

        self.goalsQ = Conjunction(  Compound('goal', Compound('serve', 'ObjectID', 'Person', 'Drink', 'LastKnowPose')), 
                                    Compound('not', Compound('carrying', Compound('drink', 'Drink', 'CarryingLoc'))))

        self.storageWaypointsQ = Conjunction(   Compound('=', 'Waypoint', Compound('storage_room', 'W')),
                                                Compound('waypoint', 'Waypoint', Compound('pose_2d', 'X', 'Y', 'Phi')),
                                                Compound('not', Compound('visited', 'Waypoint')))
    
        # query for detect object, finishes when something found or timeout!
        self.drinksOfInterestQ = Conjunction(   Compound('goal', Compound('serve', 'UniqueID', 'Person', 'Drink', 'LastKnowPose')),
                                                Compound( 'property_expected', 'ObjectID', 'class_label', 'Drink'),
                                                Compound( 'property_expected', 'ObjectID', 'position', Compound('in_front_of', 'amigo')))
    
    def execute(self, userdata = None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: LookForDrinks\n")

        # query requested drink, that haven't been picked up
        goals = self.robot.reasoner.query(self.goalsQ)

        # if there is no drink requested, return not found
        if not goals:
            self.robot.speech.speak("I picked up all the drinks.", mood='excited', block=False)
            self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
            self.robot.spindle.high()
            return 'done'

        # get the waypoint of where to search, storage_room
        storageRoomWaypts = self.robot.reasoner.query(self.storageWaypointsQ)

        # if there are no more unvisited locations, give up
        if not storageRoomWaypts:
            if self.arrivedAtKitchen == True:
                rospy.logwarn("\t\t[Cocktail Party] Going to the kitchen reset location.\n")
                self.robot.speech.speak("I'm having trouble completing my requests, let me try again.", block=False)
                return "reset_at_kitchen"
            else:
                rospy.logwarn("\t\t[Cocktail Party] Going to the room reset location.\n")
                self.robot.speech.speak("I'm having some troubles getting to the kitchen, let me try again.", block=False)
                return "reset_at_room"


        drinkNames = set([str(answer["Drink"]) for answer in goals])
        orderedDrinks = " and ".join(drinkNames)

        # say different phrases randomly
        lookedIdx =  random.randint(0, 4)

        if lookedIdx == 1:
            self.robot.speech.speak("I'm on the move, looking for your " + orderedDrinks, block = False)
        elif lookedIdx == 2:
            self.robot.speech.speak("Still on the move looking for your " + orderedDrinks, block = False)
        else:
            self.robot.speech.speak("I think I know the location of your " + orderedDrinks, block = False)

        #take the first goal found
        goal = (float(storageRoomWaypts[0]['X']), float(storageRoomWaypts[0]['Y']), float(storageRoomWaypts[0]['Phi']))
        waypointName = storageRoomWaypts[0]['Waypoint']

        # navigate to the waypoint queried
        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        navResult = nav.execute()

        # assert that this location has been visited
        self.robot.reasoner.query(Compound('assert', Compound('visited', waypointName)))

        # If navResult is unreachable DO NOT stop looking, there are more options, return not_found when list of Waypoints is empty
        if navResult == 'unreachable' or navResult == 'preempted':
            return 'looking'

        # if the robot reaches this point it means he is probably in the kitchen
        self.arrivedAtKitchen = True

        # look to points of interest, if there is any, look at it
        pointsInterest = self.robot.reasoner.query(Compound('point_of_interest', waypointName, Compound('point_3d', 'X', 'Y', 'Z')))

        # If point of interest where found, take the first one
        if pointsInterest:
            x,y,z = pointsInterest[0]['X'],pointsInterest[0]['Y'],pointsInterest[0]['Z']
            lookat_point = msgs.PointStamped(x,y,z)
            self.robot.head.send_goal(lookat_point, timeout=2.0)
        else:
            self.robot.speech.speak("I don't know where to look for drinks.", block=False)
            rospy.logerr("\t\t[Cocktail Party] No point of interest found!\n")

        self.robot.speech.speak("Let's see what I can find here.", block=False)

        # start object template matching
        self.response_start = self.robot.perception.toggle(["object_segmentation"])
 
        if self.response_start.error_code == 0:
            rospy.loginfo("Object segmentation has started correctly")
        elif self.response_start.error_code == 1:
            rospy.logwarn("Object segmentation failed to start")
            self.robot.speech.speak("I was not able to start object recognition.")
            return "reset_at_kitchen"

        wait_machine = Wait_query_true(self.robot, self.drinksOfInterestQ, 7)
        wait_result = wait_machine.execute()

        rospy.loginfo("Object recogition will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Object segmentation is stopped")
        elif self.response_stop.error_code == 1:
            rospy.logwarn("Failed stopping Object segmentation")

        # interpret results wait machine
        if wait_result == 'timed_out':
            self.robot.speech.speak("I did not find your " + orderedDrinks, block=False)
            return 'looking'
        elif wait_result == 'preempted':
            self.robot.speech.speak("Finding drink was preempted.", block=False)
            return 'looking'
        elif wait_result == 'query_true':

            drinksFound = self.robot.reasoner.query(self.drinksOfInterestQ)

            if len(drinksFound) == 1:
                orderedDrinks = drinksFound[0]["Drink"]
            else:
                drinkNames = set([str(answer["Drink"]) for answer in drinksFound])
                orderedDrinks = " and ".join(drinkNames)

            self.robot.speech.speak("Hey, I found a {0}!".format(orderedDrinks), mood="excited", block=False)
            return 'found'


#########################################################################################


# Determine which arm is going to be used to pickup the drink
class PreparePickup(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=[  'pickup_left', 
                                            'pickup_right', 
                                            'pickup_basket', 
                                            'grabbed_all',
                                            'no_requested_drinks_here'])

        self.robot = robot

        self.armCount = 0

        # get the drinks requested that were seen in this storage location AND are NOT already in the robot's posession
        self.drinksRequestedQ = Conjunction(Compound('goal', Compound('serve', 'UniqueID', 'Person', 'Drink', 'LastKnowPose')),
                                            Compound('property_expected', 'ObjectID', 'class_label', 'Drink'),
                                            # Compound('property_expected', 'ObjectID', 'position', Compound('in_front_of', 'amigo')),
                                            Compound('property_expected', 'ObjectID', 'position', Sequence('X', 'Y', 'Z')),
                                            Compound('not', Compound('carrying', Compound('drink', 'Drink', 'CarryingLoc'))))

        self.requestsQ = Conjunction(   Compound('goal', Compound('serve', 'UniqueID', 'Person', 'Drink', 'LastKnowPose')),
                                        Compound('not', Compound('carrying', Compound('drink', 'Drink', 'CarryingLoc'))))

    def execute(self, userdata = None):

        rospy.loginfo('\t\t[Cocktail Party] Entered State: PreparePickup\n')

        carryingRes = self.robot.reasoner.query(Compound('carrying', 
                                                Compound('drink', 'Drink', 'CarryingLoc')))

        # location already carrying drinks
        carryingNames = set([str(answer['CarryingLoc']) for answer in carryingRes])

        # find which arms are available for carrying (default 'right_arm')
        carryingLoc = 'right_arm'

        if not 'basket' in carryingNames:
            carryingLoc = 'basket'
        elif not 'right_arm' in carryingNames:
            carryingLoc = 'right_arm'
        elif not 'left_arm' in carryingNames:
            carryingLoc = 'left_arm'

        drinksRequested = self.robot.reasoner.query(self.drinksRequestedQ)

        if not drinksRequested:
            # get the drinks requested that are NOT already in the robot's posession
            incompleteReq = self.robot.reasoner.query( self.requestsQ)

            if not incompleteReq:
                self.robot.speech.speak("I finished picking up all the drinks.", block=False)
                self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
                self.robot.reasoner.reset()
                self.robot.spindle.high()
                return 'grabbed_all'
            else:
                # there are still drinks requested, but they cannot be found here, so lets go to another storage
                self.robot.speech.speak("I still need to find drinks", block=False)
                return 'no_requested_drinks_here'
        else:
            rospy.loginfo("\t\t[Cocktail Party] Still have to pick up {0} orders\n".format(len(drinksRequested)))

            drinkName = drinksRequested[0]["Drink"]
            rospy.loginfo("\t\t[Cocktail Party] Preparing to pickup the {0}, with the {1}\n".format(drinkName, carryingLoc))

            # retract previous goals
            self.robot.reasoner.query(  Compound('retractall', Compound('grab_goal', 'X')))

            # add new goal
            self.robot.reasoner.query(  Compound('assert', 
                                        Compound('grab_goal',
                                        Compound('grab', drinkName, carryingLoc))))

            if carryingLoc == 'left_arm':
                return 'pickup_left'
            elif carryingLoc  == 'right_arm':
                return 'pickup_right'
            else:
                return 'pickup_basket'


#########################################################################################


# Forces the robot to search again the storage room, for times when it cannot grasp the drink at first
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
            self.robot.speech.speak("I could not pick up one of the drinks. I'll try again.", block=False)
            self.resetCount+=1;

            self.robot.reasoner.query(Compound('retractall', Compound('visited', 'X')))
        else:
            self.resetCount=0;
            self.robot.speech.speak("I still could not pickup the drink.", block=False)


        return 'done'


#########################################################################################


# Assert that a drink has been picked up with a specific arm
class AssertPickup(smach.State):
    def __init__(self, robot, query):
        smach.State.__init__(   self, 
                                outcomes=['done'])

        # initializations
        self.robot = robot
        self.pickedUpDrinkQ = query

    def execute(self, userdata = None):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: AssertPickup\n")

        res = self.robot.reasoner.query(self.pickedUpDrinkQ)

        # assert that this drink has been grabbed
        self.robot.reasoner.query(  Compound("assert", 
                                    Compound("carrying", 
                                    Compound("drink", res[0]["Drink"], res[0]["CarryingLoc"]))))

        rospy.loginfo("\t\t[Cocktail Party] Asserted pickup of a {0} with the {1}\n".format(res[0]["Drink"], res[0]["CarryingLoc"]))

        return 'done'


#########################################################################################

# TODO Use this for the prepare delivery instead of the code that is there now, and use the output keys for the name and probability!

class RecognizePerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['no_people_found', 'no_people_recognized', 'person_recognized'],
                                   input_keys=[],
                                   output_keys=[])
        self.robot = robot
        
        self.detect_query = Conjunction(Compound( "property_expected", "ObjectID", "class_label", "face"),
                                        Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")))
        
        self.recognize_query = Conjunction(Compound( "property_expected", "ObjectID", "class_label", "face"),
                                           Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                           Compound( "property", "ObjectID", "name", Compound("discrete", "DomainSize", "NamePMF")))

    def execute(self, userdata=None):
        ''' Reset robot '''
        self.robot.speech.speak("Let me see who's here", block=False)
        self.robot.head.look_up()
        self.robot.spindle.reset()
   
        cntr = 0
        name = None
        # TODO does the toggle needs to start both or just recognition?
        self.robot.perception.toggle(["face_recognition", "face_segmentation"])

        while (cntr < 10 and not name):
            rospy.sleep(0.5)
            cntr += 1
            rospy.loginfo("Checking for the {0} time".format(cntr))

            person_result = self.robot.reasoner.query(self.recognize_query)

            # get the name PMF, which has the following structure: [p(0.4, exact(will)), p(0.3, exact(john)), ...]
            if len(person_result) > 0:
                name_pmf = person_result[0]["NamePMF"]

                if len(person_result) > 1:
                    rospy.logwarn("Multiple faces detected, only checking the first one!")

                name=None
                name_prob=0
                for name_possibility in name_pmf:
                    print name_possibility
                    prob = float(name_possibility[0])
                    if prob > 0.175 and prob > name_prob:
                        name = str(name_possibility[1][0])
                        #print "Updated most probable name to " + str(name)
                        name_prob = prob

        self.robot.perception.toggle([])
        
        if not person_result:
            rospy.logwarn("No person names received from world model")
            return 'no_people_found'
        elif not name:
            self.robot.speech.speak("I don't know who you are.", block=False)
            return 'no_people_recognized'
        elif name:
            self.robot.speech.speak("Hello " + str(name), block=False)
            #userdata.name = str(name)
            return 'person_recognized'        
            

#########################################################################################


# Identify the person in front of the robot and determine where his/her drink is
class PrepareDelivery(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,  outcomes=[  'handover_left', 
                                                'handover_right',
                                                'handover_basket',
                                                'unknown', 
                                                'not_correct',
                                                'no_people',
                                                'error'],
                                    output_keys=[   'person_out ',
                                                    'drink_out'])
        self.robot = robot

        self.facesDetectedQ = Conjunction(  Compound( 'property_expected', 'ObjectID', 'class_label', 'face'),
                                            Compound( 'property_expected', 'ObjectID', 'position', Compound('in_front_of', 'amigo')),
                                            Compound( 'property', 'ObjectID', 'name', Compound('discrete', 'DomainSize', 'NamePMF')))

    def execute(self, userdata):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: PrepareDelivery\n")

        self.robot.spindle.high()
        self.robot.head.set_pan_tilt(tilt=-0.0, pan=0.0, timeout=2.0)

        self.robot.speech.speak("Let me see if there's someone here. Please look at my face.", block=False)
        
        # TODO check color
        self.robot.lights.set_color(0, 0, 1)

        # perform face recognition on the person found
        self.response_start = self.robot.perception.toggle(['face_recognition'])

        if self.response_start.error_code == 0:
            rospy.loginfo("Face recognition has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Face recognition failed to start")
            self.robot.speech.speak("I was not able to start face recognition.")
            return 'error'
        
        # sleep while we wait for results on the recognition
        rospy.sleep(8.0)

        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Face recognition is stopped")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping face recognition")

        # Interpret face regnition results
        facesDetected = self.robot.reasoner.query(self.facesDetectedQ)
        
        # if there were people found...
        if facesDetected:
            if len(facesDetected) > 1:
                rospy.logwarn("Multiple faces detected, only checking the first one!")

            name_pmf = facesDetected[0]['NamePMF']

            # initialize variables
            personName = None
            name_prob = 0

            # get the name with the highest probability
            for nameIterator in name_pmf:
                prob = float(nameIterator[0])
                rospy.loginfo("\t\t[Cocktail Party] Name: {0} ({1})\n".format(nameIterator[1][0], prob))
                if prob > 0.1856 and prob > name_prob:
                    personName = str(nameIterator[1][0])
                    name_prob = prob

            # If the person is not recognized
            if not personName:
                rospy.loginfo("\t\t[Cocktail Party] Couldn't identify the person\n")
                self.robot.speech.speak("I do not know you.", block=False)
                return 'unknown'
            else:
                # get requests
                requests = self.robot.reasoner.query(   Compound("goal", 
                                                        Compound("serve", "ObjectID", "PersonName", "Drink", "LastKnowPose")))

                # if there are no drinks requested, return error
                if not requests:
                    rospy.logerr("\t\t[Cocktail Party] No results for the requests query!\n")
                    self.robot.speech.speak("I forgot who I had to serve", block=False)
                    return 'error'

                peopleNames = set([str(answer['PersonName']) for answer in requests])

                for nameIterator in peopleNames:
                    # if the person is recognized and is the one we are searching for
                    if personName == nameIterator:
                        
                        drinkNameRes = self.robot.reasoner.query(   Compound('goal', 
                                                                    Compound('serve', 'ObjectID', nameIterator, 'Drink', 'LastKnowPose')))
                        drinkName = str(drinkNameRes[0]['Drink'])

                        carryingRes = self.robot.reasoner.query(Compound('carrying', 
                                                                Compound('drink', drinkName, 'CarryingLoc')))

                        if not carryingRes:
                            self.robot.speech.speak("Hello {0}, I don't have any drinks for you.".format(personName), block=False)
                            rospy.logwarn("\t\t[Cocktail Party] Query about carried objects is empty!\n") 
                            return 'error'

                        person_out = personName
                        drink_out = drinkName

                        self.robot.reasoner.query(  Compound('retractall', Compound('delivering', 'X')))

                        self.robot.reasoner.query(  Compound('assert', 
                                                    Compound('delivering',
                                                    Compound('delivery', personName, drinkName))))

                        carryingLoc = str(carryingRes[0]['CarryingLoc'])

                        self.robot.speech.speak("Hello " + personName + ", I have the " + drinkName + " your requested.", block=False)
                        
                        rospy.loginfo("\t\t[Cocktail Party] Delivering a {0} to {1}, carried in the {2}\n".format(
                            drinkName, personName, carryingLoc))

                        if carryingLoc == 'left_arm':
                            return 'handover_left'
                        elif carryingLoc == 'right_arm':
                            return 'handover_right'
                        elif carryingLoc == 'basket':
                            return 'handover_basket'
                        else:
                            self.robot.speech.speak("Oh no, I forgot where i was carrying {0}'s order ".format(personName), block=False)
                            rospy.logerr("\t\t[Cocktail Party] The CarryingLoc is not valid (carryingLoc=[{0}]).\n".format(carryingLoc)) 
                            return 'error'

                self.robot.speech.speak("Hello {0}, you did not request anything.".format(personName), block=False)
                return 'not_correct'

        # if there were no people found...
        else:
            self.robot.speech.speak("I thought there was someone here, but I'm mistaken.", block=False)
            rospy.logwarn("\t\t[Cocktail Party] No person names received from world model\n") 
            return 'no_people'


#########################################################################################


# Navigate to a detected person
class NavToDetectedPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=[   'unreachable', 
                                                'arrived',
                                                'visited_all'])
        self.robot = robot

    def execute(self, userdata):

        rospy.loginfo("\t\t[Cocktail Party] Entered State: NavToDetectedPerson\n")

        detectedPersonQ = Conjunction(  Compound('property_expected', 'ObjectID', 'class_label', 'validated_person'),
                                        Compound('property_expected', 'ObjectID', 'position', Sequence('X','Y','Z')),
                                        Compound('not', Compound('visited', 'ObjectID')))

        # get results from the query
        detectedPerson = self.robot.reasoner.query(detectedPersonQ)

        # if there is no location associated with lookout points say it
        if not detectedPerson:
            rospy.loginfo("\t\t[Cocktail Party] Visited all the persons\n")
            return 'visited_all'
        else:

            # rospy.loginfo("\t\t[Cocktail Party] Found {0} person(s)\n". format(len(detectedPerson)))

            # If there is more than 1 result, force a query that only returns the first result
            #   otherwise i don't know which one the nav generic will choose
            if len(detectedPerson) > 1:
                selectedID = detectedPerson[0]["ObjectID"]

                detectedPersonQ = Conjunction(Compound('property_expected', selectedID, 'class_label', 'validated_person'),
                                            Compound('property_expected', selectedID, 'position', Sequence('X','Y','Z')),
                                            Compound('not', Compound('visited', selectedID)))

                objectID = selectedID
            else:
                objectID = detectedPerson[0]['ObjectID']

            # Use the lookat query
            nav = NavigateGeneric(self.robot, lookat_query = detectedPersonQ)
            nav_result = nav.execute()

            # assert that this location has been visited
            self.robot.reasoner.query(Compound('assert', Compound('visited', objectID)))
            self.robot.reasoner.query(Compound('assert', Compound('approached', objectID)))

            if nav_result == 'unreachable':                    
                return 'unreachable'
            elif nav_result == 'preempted':
                return 'unreachable'
            else:
                rospy.logwarn("\t\t[Cocktail Party] Invalid return value from Nav: {0}\n".format(nav_result))
                
            return 'arrived'


#########################################################################################


# Handover the drink in the left gripper
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
    

# Handover the drink in the right gripper
class HandoverDrinkRight(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["done"])

        arm = robot.rightArm

        with self:
            smach.StateMachine.add( 'PRESENT_DRINK',
                                    Say(robot, ["I'm going to hand over your drink now", 
                                                "Here you go! Handing over your drink"], block=False),
                                    transitions={'spoken':'POSE'})

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
                                    Say(robot, ["Enjoy your drink!", 
                                                "I hope you're thirsty, enjoy!"], block=False),
                                    transitions={'spoken':'done'})


#########################################################################################


# Retract the person just served from the reasoner and increment the number of served people
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
        # TODO: PASS THE VARIABLES THROUGH USERDATA AND NOT QUERY (Not working, not sure why)

        deliveryRes = self.robot.reasoner.query(Compound("delivering",
                                                Compound("delivery", "Person", "Drink")))
    
        if not deliveryRes:
            rospy.logerr("\t\t[Cocktail Party] No information on this delivery.\n")
            return 'failed'
        else:
            personName = deliveryRes[0]['Person']
            drinkName = deliveryRes[0]['Drink']

        if personName and drinkName:
            # retract from drinks being carried
            retractDrink = self.robot.reasoner.query(   Compound( 'retract', 
                                                        Compound( 'carrying', 
                                                        Compound( 'drink', drinkName, 'Arm'))))

            # retract from persons being served
            retractPerson = self.robot.reasoner.query(  Compound( 'retract', 
                                                        Compound( 'goal', 
                                                        Compound( 'serve', 'ObjectID', personName, drinkName, 'LastKnowPose'))))

            rospy.loginfo("\t\t[Cocktail Party] Retracted {0} drink and {1} person.\n".format(len(retractDrink), len(retractPerson)))

            # update the number of served people
            servedCount = self.robot.reasoner.query(Compound("people_served_count", "Counter"))

            nServed = float(servedCount[0]["Counter"]) + 1.0

            rospy.loginfo("\t\t[Cocktail Party]Hurray! Another drink was served (total served:{0})\n".format(nServed))

            self.robot.reasoner.query(Compound('retractall', Compound('people_served_count', 'X')))
            self.robot.reasoner.query(Compound('assert',Compound('people_served_count', nServed)))

            carrying = self.robot.reasoner.query(Compound('carrying', Compound('drink', 'Drink', 'Arm')))

            if not carrying:
                self.robot.speech.speak("I finished delivering all the drinks!", mood="excited", block=False)
                return 'all_served'
            else:
                rospy.loginfo("\t\t[Cocktail Party] Still have to deliver {0} drink(s)\n".format(len(carrying)))
                return 'done'

        else:
            rospy.logerr("\t\t[Cocktail Party] Unable to retract the person and drink served\n")
            return 'failed'


########################################################################################


# Drop the grasped drink in the basket
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

        # sleep to give enough time for the spindle to go up before closing the gripper
        rospy.sleep(1.5)
        
        self.robot.leftArm.send_gripper_goal_close(timeout=5.0)

        self.robot.leftArm.reset_arm()

        self.robot.spindle.reset()

        return 'done'


#########################################################################################


# Focus the head of the robot on the person's face. Only finishes when preempted
class FocusOnFace(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, 
                                    outcomes=['loop', 'preempted'])

        self.robot = robot
        self.firstTime = 0

        # prepare query for detected faces
        self.facesQ = Conjunction(  Compound('property_expected', 'ObjectID', 'class_label', 'face'),
                                    Compound('property_expected', 'ObjectID', 'position', Compound('in_front_of', 'amigo')),
                                    Compound('property_expected', 'ObjectID', 'position', Sequence('X','Y','Z')))

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

        res = self.robot.reasoner.query(self.facesQ)

        if res:
            x,y,z = res[0]['X'],res[0]['Y'],res[0]['Z']

            lookat_point = msgs.PointStamped(x,y,z)

            # rospy.loginfo('\t\t[Cocktail Party] Looking at person's face ({0}, {1}, {2})\n'.format(x, y, z))
            self.robot.head.send_goal(lookat_point, timeout=0)
        else: 
            rospy.loginfo('\t\t[Cocktail Party] No face found in front of amigo\n')

        # get the preempted flag value
        preemptFlag = self.robot.reasoner.query(Compound('preempt_head_focus', 'X'))

        if preemptFlag:
            preempt = float(preemptFlag[0]['X'])
        else:
            rospy.logwarn("\t\t[Cocktail Party] Unable to query preempt_head_focus, assuming its 0/FALSE\n")
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
            self.robot.reasoner.query(Compound('retractall', Compound('preempt_head_focus', 'X')))
            self.robot.reasoner.query(Compound('assert',Compound('preempt_head_focus', '0')))
            self.firstTime = 0

            # give it one second for the face focus to preempt, to avoind conflicts between human_tracking and face_recognition
            # rospy.sleep(1)

            return 'preempted'


#########################################################################################


# Infinite loop that calls FocusOnFace. FocusOnFace can be preempted to stop
class FocusOnFaceLoop(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['done'])

        with self:

            smach.StateMachine.add( 'FOCUS_ON_FACE',
                                    FocusOnFace(robot),
                                    transitions={   'loop':'FOCUS_ON_FACE',
                                                    'preempted':'done'})


#########################################################################################


# State that preempts FocusOnFace by asserting TRUE / 1 to the reasoner
class PreemptFaceFocus(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])

        self.robot = robot

    def execute(self, userdata):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: PreemptFaceFocus!\n")

        self.robot.reasoner.query(Compound('retractall', Compound('preempt_head_focus', 'X')))
        self.robot.reasoner.query(Compound('assert',Compound('preempt_head_focus', '1')))

        # rospy.sleep(0.5)

        return 'done'


#########################################################################################


# Handover undelivered drinks by requesting anyone to take them
class HandoverUndeliveredDrinks(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['done'])

        self.robot = robot

    def execute(self, userdata=None):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: HandoverUndeliveredDrinks\n")

        self.robot.speech.speak("I will open both grippers now.")

        # TODO: copy the arm pose from Handover_pose and set it

        self.robot.leftArm.send_gripper_goal_open(timeout=5.0)
        self.robot.rightArm.send_gripper_goal_open(timeout=5.0)
        
        self.robot.leftArm.reset_arm()
        self.robot.rightArm.reset_arm()

        self.robot.leftArm.send_gripper_goal_close(timeout=5.0)
        self.robot.rightArm.send_gripper_goal_close(timeout=5.0)

        # retract from drinks being carried
        amigo.reasoner.query(Compound('retractall', Compound('carrying', 'X')))
        amigo.reasoner.query(Compound('retractall', Compound('goal', 'X')))

        rospy.sleep(4)

        return 'done'


#########################################################################################


# Wait to hear someone say Continue before exiting the state
class HearContinue(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['continue', 'no_continue', 'force_continue'])

        self.robot = robot
        self.ask_user_service_continue = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: HearContinue\n")

        # TODO check color
        self.robot.lights.set_color(1, 0, 0)

        response = self.ask_user_service_continue("continue", 4, rospy.Duration(15))

        for x in range(0, len(response.keys)):
            if response.keys[x] == 'answer':
                if response.values[x] == 'true':
                    return 'continue'
                else: 
                    return 'no_continue'

        rospy.logwarn("answer was not found in response of interpreter. Should not happen!!")
        
        return 'no_continue'


#########################################################################################


# Check if the robot is carrying drinks
class CheckCarriedDrinks(smach.State):
    def __init__(self, robot):
        smach.State.__init__(   self, 
                                outcomes=['something', 'none'])

        self.robot = robot

    def execute(self, userdata):
        rospy.loginfo("\t\t[Cocktail Party] Entered State: CheckCarriedDrinks\n")

        carrying = self.robot.reasoner.query(Compound('carrying', 'X'))

        if carrying:
            rospy.loginfo("\t\t[Cocktail Party] Carrying {0} drinks\n".format(len(carrying)))
            return 'something'
        else:
            rospy.loginfo("\t\t[Cocktail Party] Not carrying any drinks\n")
            return 'none'


#########################################################################################


#  State machine to take a new order, learn the name, face and drink
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


#  State machine to take a new order while focussing the head on the person's face
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


# Main State Machine
class CocktailParty(smach.StateMachine):
    def __init__(self, robot):

        # Create a SMACH state machine
        sm = smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])

        # prepare queries:
        partyRoomWayptsQ = Compound('waypoint', 'party_room', Compound('pose_2d', 'X', 'Y', 'Phi'))

        waitingWayptQ = Compound('waypoint', Compound('party_room_waiting', 'S'), Compound('pose_2d', 'X', 'Y', 'Phi'))

        grabGoalDrinkQ = Conjunction(  Compound('grab_goal', Compound('grab', 'Drink', 'CarryingLoc')),
                                       Compound( 'property_expected', 'ObjectID', 'class_label', 'Drink'),
                                       Compound( 'property_expected', 'ObjectID', 'position', Compound('in_front_of', 'amigo')),
                                       Compound( 'property_expected', 'ObjectID', 'position', Sequence('X', 'Y', 'Z')))


#-----------------------------------------ENTER THE ROOM---------------------------------------------------------


        with self:

            # Start
            smach.StateMachine.add( 'START_CHALLENGE',
                                    StartChallengeRobust(robot, 'initial'),
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

            smach.StateMachine.add('GOTO_PARTY_ROOM',
                                    NavigateGeneric(robot, goal_query = partyRoomWayptsQ),
                                    transitions={   'arrived':'GOTO_WAITING_PLACE', 
                                                    'unreachable':'GOTO_WAITING_PLACE', 
                                                    'preempted':'GOTO_WAITING_PLACE', 
                                                    'goal_not_defined':'GOTO_WAITING_PLACE'})
                                    # transitions={   'arrived':'LOOKOUT_CONTAINER', 
                                    #               'unreachable':'LOOKOUT_CONTAINER', 
                                    #               'preempted':'LOOKOUT_CONTAINER', 
                                    #               'goal_not_defined':'LOOKOUT_CONTAINER'})



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


#----------------------------------------WAIT FOR PEOPLE TO COME IN FRONT (FALLBACK)------------------


            smach.StateMachine.add( 'GOTO_WAITING_PLACE',
                                    NavToWaitingLoc(robot),
                                    transitions={   'unreachable':'GOTO_WAITING_PLACE',
                                                    'arrived':'WAIT_PERSON_ITERATOR',
                                                    'visited_all':'WAIT_PERSON_ITERATOR'})

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


#----------------------------------------TAKE THE ORDERS-------------------------------------------------------


            # create orders container
            ordersContainer = smach.StateMachine(  outcomes = ['succeeded'])

            with ordersContainer:

                smach.StateMachine.add( 'NAV_TO_WAIVING_PERSON',
                                        NavToWavingPerson(robot),
                                        transitions={   'going':'NAV_TO_WAIVING_PERSON',
                                                        'arrived':'CONFIRM_PERSON_DETECTION',
                                                        'max_orders':'succeeded',
                                                        'visited_all':'succeeded'})

                smach.StateMachine.add( 'CONFIRM_PERSON_DETECTION',
                                        ConfirmPersonDetection(robot),
                                        transitions={   'found':'TAKE_NEW_ORDER_FOCUS',
                                                        'error':'NAV_TO_WAIVING_PERSON',
                                                        'not_found':'NAV_TO_WAIVING_PERSON'})

                smach.StateMachine.add( 'TAKE_NEW_ORDER_FOCUS',
                                        TakeNewOrder(robot),
                                        transitions={   'done':'NAV_TO_WAIVING_PERSON'})

            # add orders container to the main state machine
            smach.StateMachine.add( 'ORDERS_CONTAINER',
                                    ordersContainer,
                                    transitions={   'succeeded':'CHECK_PENDING_ORDERS'})

            smach.StateMachine.add( 'CHECK_PENDING_ORDERS',
                                    CheckPendingOrders(robot),
                                    transitions={   'enough_orders':'FIND_DRINKS_CONTAINER',
                                                    # 'insuficient_orders':'LOOKOUT_CONTAINER'
                                                    'insuficient_orders':'WAIT_PERSON_ITERATOR'
                                                    })


#--------------------------------------------FIND THE REQUESTED DRINKS---------------------------------------------


            # create find drinks container
            findDrinksContainer = smach.StateMachine(  outcomes = ['succeeded','aborted'])

            with findDrinksContainer:

                # Go to storage room waypoint and look for the drinks
                smach.StateMachine.add( 'LOOK_FOR_DRINKS',
                                        LookForDrinks(robot),
                                        transitions={   'looking':'LOOK_FOR_DRINKS',
                                                        'found':'PREPARE_PICKUP',
                                                        'visited_all':'SAY_DRINK_NOT_FOUND',
                                                        'reset_at_room':'NAV_TO_RESET_LOC_ROOM',
                                                        'reset_at_kitchen':'NAV_TO_RESET_LOC_KITCHEN',
                                                        'done':'succeeded'})

                # Pickup the drink
                smach.StateMachine.add( 'PREPARE_PICKUP',
                                        PreparePickup(robot),
                                        transitions={   'pickup_left':'PICKUP_DRINK_LEFT',
                                                        'pickup_right':'PICKUP_DRINK_RIGHT',
                                                        'pickup_basket':'PICKUP_DRINK_BASKET',
                                                        'no_requested_drinks_here':'LOOK_FOR_DRINKS',
                                                        'grabbed_all':'succeeded'}) 

                smach.StateMachine.add( 'PICKUP_DRINK_LEFT',
                                        GrabMachine('left', robot, grabGoalDrinkQ),
                                        transitions={   'succeeded':'ASSERT_PICKUP',
                                                        'failed':'RESET_SEARCHED_LOCATIONS' }) 

                smach.StateMachine.add( 'PICKUP_DRINK_RIGHT',
                                        GrabMachine('right', robot, grabGoalDrinkQ),
                                        transitions={   'succeeded':'ASSERT_PICKUP',
                                                        'failed':'RESET_SEARCHED_LOCATIONS' }) 

                smach.StateMachine.add( 'PICKUP_DRINK_BASKET',
                                        GrabMachine('left', robot, grabGoalDrinkQ),
                                        transitions={   'succeeded':'DROP_IN_BASKET',
                                                        'failed':'RESET_SEARCHED_LOCATIONS' })

                smach.StateMachine.add( 'DROP_IN_BASKET',
                                        DropInBasket(robot),
                                        transitions={   'done':'ASSERT_PICKUP'}) 

                smach.StateMachine.add( 'RESET_SEARCHED_LOCATIONS',
                                        ResetSearchedLocations(robot),
                                        transitions={   'done':'LOOK_FOR_DRINKS'})

                smach.StateMachine.add( 'ASSERT_PICKUP',
                                        AssertPickup(robot, grabGoalDrinkQ),
                                        transitions={   'done':'PREPARE_PICKUP'})

                smach.StateMachine.add( 'SAY_DRINK_NOT_FOUND',
                                        Say(robot, "I could not find all of the drinks requested. Let me try again.", block=False),
                                        transitions={   'spoken':'aborted' })

                smach.StateMachine.add( 'NAV_TO_RESET_LOC_ROOM',
                                        NavResetLocRoom(robot),
                                        transitions={   'unreachable':'NAV_TO_RESET_LOC_KITCHEN',
                                                        'arrived':'LOOK_FOR_DRINKS',
                                                        'visited_all':'NAV_TO_RESET_LOC_KITCHEN'})

                smach.StateMachine.add( 'NAV_TO_RESET_LOC_KITCHEN',
                                        NavResetLocKitchen(robot),
                                        transitions={   'unreachable':'LOOK_FOR_DRINKS',
                                                        'arrived':'LOOK_FOR_DRINKS',
                                                        'visited_all':'aborted'})

            # add find drinks container to the main state machine
            smach.StateMachine.add( 'FIND_DRINKS_CONTAINER',
                                    findDrinksContainer,
                                    transitions={   'succeeded':'DELIVER_DRINKS_CONTAINER',
                                                    'aborted':'CHECK_CARRIED_DRINKS_PICKUP'})

            smach.StateMachine.add( 'CHECK_CARRIED_DRINKS_PICKUP',
                                    CheckCarriedDrinks(robot),
                                    transitions={   'something':'DELIVER_DRINKS_CONTAINER',
                                                    'none':'SERVED_STATUS'})


#-----------------------------------------DELIVER THE REQUESTED DRINKS---------------------------------------------


            # create deliver drinks container
            deliverDrinksContainer = smach.StateMachine(  outcomes = ['succeeded','aborted'])

            with deliverDrinksContainer:

                # Go to the last person known location
                smach.StateMachine.add('NAV_TO_LAST_KNOWN_LOCATION',
                                        NavToLastKnowLoc(robot),
                                        transitions={   'unreachable':'NAV_TO_LAST_KNOWN_LOCATION' , 
                                                        'arrived':'DETECT_PEOPLE',
                                                        # 'arrived':'DETECT_PEOPLE_TORSO_LASER', 
                                                        'visited_all':'NAV_TO_LOOKOUT'})

                # if the request weren't all delivered continue searching on the lookout points
                smach.StateMachine.add( 'NAV_TO_LOOKOUT',
                                        NavToLookout(robot),
                                        transitions={   'unreachable':'NAV_TO_LOOKOUT',
                                                        'arrived':'DETECT_PEOPLE',
                                                        # 'arrived':'DETECT_PEOPLE_TORSO_LASER',
                                                        'visited_all':'aborted'})

                # smach.StateMachine.add( 'DETECT_PEOPLE_TORSO_LASER',
                #                         StandingPeopleDetector(robot, time=4, room='living_room'),
                #                         transitions={   'done':'DETECT_PEOPLE',
                #                                         'failed':'DETECT_PEOPLE'})

                smach.StateMachine.add( 'DETECT_PEOPLE',
                                        DetectPeople(robot, time=4, room='living_room'),
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
                                        transitions={   'spoken':'HEAR_CONTINUE_BASKET'})

                smach.StateMachine.add( 'HEAR_CONTINUE_BASKET',
                                        HearContinue(robot),
                                        transitions={   'continue':'RETRACT_SERVED_PERSON',
                                                        'force_continue':'RETRACT_SERVED_PERSON',
                                                        'no_continue':'HEAR_CONTINUE_BASKET'})

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
                                        transitions={   'done':'CHECK_CARRIED_DRINKS_DELIVERY',
                                                        'all_served':'succeeded',
                                                        'failed':'aborted'})

                smach.StateMachine.add( 'CHECK_CARRIED_DRINKS_DELIVERY',
                                    CheckCarriedDrinks(robot),
                                    transitions={   'something':'NAV_TO_DETECTED_PERSON',
                                                    'none':'succeeded'})

            # add find drinks container to the main state machine
            smach.StateMachine.add( 'DELIVER_DRINKS_CONTAINER',
                                    deliverDrinksContainer,
                                    transitions={   'succeeded':'SERVED_STATUS',
                                                    'aborted':'SAY_ANYONE_GRAB_DRINKS'})

            smach.StateMachine.add( 'SAY_ANYONE_GRAB_DRINKS',
                                    Say(robot, "Can someone please take these drinks from my hand and basket? I will drop them when i hear continue.", block=False),
                                    transitions={   'spoken':'HEAR_CONTINUE_UNDELIVERED' })

            smach.StateMachine.add( 'HEAR_CONTINUE_UNDELIVERED',
                                    HearContinue(robot),
                                    transitions={   'continue':'HANDOVER_UNDELIVERED_DRINKS',
                                                    'force_continue':'HANDOVER_UNDELIVERED_DRINKS',
                                                    'no_continue':'HEAR_CONTINUE_UNDELIVERED'})

            smach.StateMachine.add( 'HANDOVER_UNDELIVERED_DRINKS',
                                    HandoverUndeliveredDrinks(robot),
                                    transitions={   'done':'SERVED_STATUS'})

            smach.StateMachine.add( 'SERVED_STATUS',
                                    ServedStatus(robot),
                                    transitions={   'complete':'SAY_FINISHED_SERVING',
                                                    'incomplete':'GOTO_WAITING_PLACE'
                                                    # 'incomplete':'LOOKOUT_CONTAINER'
                                                    })


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
    amigo.reasoner.query(Compound('retractall', Compound('ordered', 'X')))

    # tag to identify faces that were used to confirm a person detection
    amigo.reasoner.query(Compound('retractall', Compound('confirmed', 'X')))

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
  

    # initial_state = None
    initial_state = 'FIND_DRINKS_CONTAINER'
    # initial_state = 'DELIVER_DRINKS_CONTAINER'
    # initial_state = 'GOTO_WAITING_PLACE'
    # initial_state = 'LOOKOUT_CONTAINER'

    machine = CocktailParty(amigo)
    
    # Testing data, automatically asserted
    if initial_state != None:
        amigo.reasoner.reset()

        machine.set_initial_state([initial_state])

        # simulate requests
        amigo.reasoner.query(   Compound('assert', 
                                Compound('goal',
                                Compound('serve', 'david_milk', 'david', 'milk', Compound('pose_2d', '2.829', '2.030', '-1.514')))))

        amigo.reasoner.query(   Compound('assert', 
                                Compound('goal',
                                Compound('serve', 'william_coke', 'william', 'coke',  Compound('pose_2d', '2.829', '2.030', '-1.514')))))

        # simulate picked up drinks
        # amigo.reasoner.query(   Compound('assert', Compound('carrying', Compound('drink', 'milk', 'basket'))))

        # amigo.reasoner.query(   Compound('assert', Compound('carrying', Compound('drink', 'coke', 'right_arm'))))

        # simulate last know locations of people
        # amigo.reasoner.query(   Compound('assert', 
        #                         Compound('waypoint', Compound('last_known_location', '2.756_0.857_0.0'), Sequence('2.756', '0.857', '0.0'))))


    introserver = smach_ros.IntrospectionServer('SM_TOP', machine, '/SM_ROOT_PRIMARY')
    introserver.start()

    try:
        outcome = machine.execute()
    except Exception, e:
        amigo.speech.speak(e)
    finally:
        introserver.stop()
