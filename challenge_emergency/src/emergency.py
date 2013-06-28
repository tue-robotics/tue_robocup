#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_emergency')
import rospy
import smach
import math
import cv
import roslib
import geometry_msgs.msg

#import sys
import roslib.packages as p

from robot_skills.amigo import Amigo
import robot_smach_states as states
from robot_smach_states.util.startup import startup
import robot_smach_states.util.transformations as transformations

from std_msgs.msg import String
from speech_interpreter.srv import GetContinue
from speech_interpreter.srv import GetYesNo
from virtual_cam.srv import cheese
from challenge_emergency.srv import Start

from perception_srvs.srv import StartPerception

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#import states_new as states 
from psi import *



#########################################
# Created by: Erik Geerts, Teun Derksen #
#########################################

#################
## TODO LIST!! ##
#################

# Implement
#   - Detect smoke
#   - Fix easy switching to environment (box around map)

##########################################
############## What to run: ##############
############ updated 15-4-2013 ###########
##########################################
# - astart
# - amiddle
# - roslaunch create_speech_files speech.launch   (in tue_test_lab the launch file is: speech_tue_test_lab.launch)
# - !! Wait for speech.launch to finish before !!
#   !!   launching speech interpreter          !!
# - roslaunch speech_interpreter start.launch     (in tue_test_lab the launch file is: speech_tue_test_lab.launch)
# - roslaunch challenge_emergency start.launch    (launches parameter files of map with pdf_creator.cpp, waits for service call)
# - rosrun challenge_emergency emergency.py

#############################################################
## Locations that must be defined in database on forehand: ##
#############################################################
# - initial
# - entry_point
# - other_side_room
# - exit

#################
### Questions ###
#################
# -


########################
##### STATEMACHINE #####
########################






class turn_Around_z_axis(smach.State):
    def __init__(self, robot, rotation):
        """
        @param robot
        @param rotation, the rotation in radian with which amigo needs to turn
        """
        smach.State.__init__(self, outcomes=["done", "abort"])
        self.robot    = robot
        self.rotation = rotation

    def execute(self, userdata=None):
        """ 
        @return
        """
        self.robot.head.reset_position()
        ##rospy.logger('Emergency.py:turnAmigo.Execute: {0} degrees.'.format((self.rotation)))

        rospy.loginfo("Executing: {0} radians".format(self.rotation))
        # get current position and rotation
        pos, rot = self.robot.base.get_location()
      
        # set rotation
        rotation = transformations.euler_z_from_quaternion(rot)
        rospy.loginfo("[EMERGENCY TEST] Current rotation value: rotation = {0}".format(rotation))
        rospy.loginfo("[EMERGENCY TEST] Rotation wanted value: rotation = {0}".format(self.rotation))
        rotation = rotation - self.rotation
        rospy.loginfo("[EMERGENCY TEST] New rotation value: rotation = {0}".format(rotation))
        rot = transformations.euler_z_to_quaternion(rotation)
        
        # create new path
        path = self.robot.base.get_plan(pos,rot)

        # execute new path
        if self.robot.base.execute_plan(path):
            return "done"
        else:
            return "abort"


class Look_for_people_emergency(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["visited", "unreachable", "all_matches_tried"])
        self.robot = robot

    def execute(self, userdata=None):
        
        self.robot.spindle.reset()
        self.robot.head.reset_position()

        # Move to the next waypoint in the storage room
        self.robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X"))) 

        navigate_apartment = Conjunction(  Compound("=", "Waypoint", Compound("apartment", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint")),
                                                 Compound("not", Compound("unreachable", "Waypoint")))

        goal_answers = self.robot.reasoner.query(navigate_apartment)             # I do not use not_visited and not_unreachable since these are no roi's

        if not goal_answers:
            return "all_matches_tried"                   

        #nav = states.NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav = Navigate_to_queryoutcome_emergency(self.robot, navigate_apartment, X="X", Y="Y", Phi="Phi")
        nav_result = nav.execute()

        if nav_result == "unreachable" or nav_result == "preempted":
            self.robot.reasoner.query(Conjunction(Compound("current_exploration_target", "Waypoint_name"),
                                                      Compound("assert", Compound("unreachable", "Waypoint_name"))))
            return "unreachable"

        elif nav_result == "goal_not_defined":
            rospy.loginfo("Goal not defined received by NavigateGeneric. Should never happen, since this check has done before calling upon NavigateGeneric.")
            return "unreachable"
        
        self.robot.reasoner.query(Conjunction(Compound("current_exploration_target", "Waypoint_name"),
                                              Compound("assert", Compound("visited", "Waypoint_name"))))

        return "visited"


class Navigate_to_queryoutcome_emergency(states.Navigate_abstract):
    """Move to the output of a query, which is passed to this state as a Term from the reasoner-module.
    
    The state can take some parameters that specify which keys of the dictionary to use for which data.
    By default, the binding-key "X" refers to the x-part of the goal, etc. 
    
    Optionally, also a sorter can be given that sorts the bindings according to some measure.
    """
    def __init__(self, robot, query, X="X", Y="Y", Phi="Phi"):
        states.Navigate_abstract.__init__(self, robot)

        assert isinstance(query, Term)

        self.queryTerm = query
        self.X, self.Y, self.Phi = X, Y, Phi
        
    def get_goal(self, userdata):
        """self.get_goal gets the answer to this query and lets it parse it into a list of binding-dictionaries. """
        
        # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
        # is a mapping of variable to a constant, like a string or number
        answers = self.robot.reasoner.query(self.queryTerm)

        if not answers:
            return None
            rospy.logerr("No answers found for query {query}".format(query=self.queryTerm))
        else:
            chosen_answer = answers[0]
            #From the summarized answer, 
            possible_locations = [( float(answer[self.X]),
                                    float(answer[self.Y]), 
                                    float(answer[self.Phi])) for answer in answers]

            x,y,phi = possible_locations[0]
            
            goal = possible_locations[0]
            rospy.loginfo("goal = {0}".format(goal))
            waypoint_name = chosen_answer["Waypoint"]
            
            self.robot.reasoner.query(Compound("assert", Compound("current_exploration_target", waypoint_name))) 

            rospy.logdebug("Found location for '{0}': {1}".format(self.queryTerm, (x,y,phi)))
            return self.robot.base.point(x,y), self.robot.base.orient(phi)


class Looking_for_people(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "failed"])

        self.robot = robot

    def execute(self, userdata):
        rospy.loginfo("reset_head and spindle")
        self.robot.spindle.reset()
        self.robot.head.reset_position()
        rospy.sleep(1.5)
        
        rospy.loginfo("Starting face segmentation")
        self.response_start = self.robot.perception.toggle(['face_segmentation'])
        #rospy.loginfo("error_code = {0}".format(self.response_start.error_code))
        #rospy.loginfo("error_msg = {0}".format(self.response_start.error_msg))
        if self.response_start.error_code == 0:
            rospy.loginfo("Face segmentation has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Face segmentation failed to start")
            self.robot.speech.speak("I was not able to start face segmentation.")
            return "failed"
        rospy.sleep(2)

        rospy.loginfo("Face segmentation will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Face segmentation is stopped")
            return "done"
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping face segmentation")
            return "failed"


# class Check_persons_found2(smach.State):
#     def __init__(self, robot):
#         smach.State.__init__(self, outcomes=["no_person_found", "person_unreachable", "person_found"])
#         self.robot = robot


#     def execute(self, userdata=None):
#         # Move to the next waypoint in the storage room

#         persons_answers = self.robot.reasoner.query(Conjunction( 
#                                                     Compound("property_expected","ObjectID", "class_label", "face"),
#                                                     Compound("property_expected","ObjectID", Sequence("X","Y","Z")),
#                                                     Compound("not", Compound("registered", "ObjectID"))))

#         if not persons_answers:
#             return "no_person_found"
        
#         if persons_answers[1]:
#             self.robot.speech.speak("I see some people!")
#         elif not persons_answers[1]:
#             self.robot.speech.speak("I found someone!")
        
#         # for now, take the first goal found : TODO, sort list on shortest distance!!
#         goal_answer = goal_answers[0]

#         goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Z"]))
#         person_id = goal_answer["ObjectID"]

#         nav = states.NavigateGeneric(self.robot, goal_lookat_point=goal)  # Should be made!!
#         nav_result = nav.execute()

#         if nav_result == "unreachable" or nav_result == "preempted":  
#             self.robot.reasoner.query(Compound("assert", Compound("registered", person_id)))  
#             self.robot.speech.speak("Although I was not able to reach the person I want to speak, I will still ask you some questions.")          
#             return "person_unreachable"

#         self.robot.reasoner.query(Compound("assert", Compound("registered", person_id)))                       

#         return "person_found"


class Check_persons_found(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["no_person_found", "person_unreachable", "person_found"])
        self.robot = robot


    def execute(self, userdata=None):

        self.robot.head.reset_position()
        self.robot.spindle.reset()

        person_query = Conjunction( 
                            Compound("property_expected","ObjectID", "class_label", "face"),
                            Compound("property_expected","ObjectID", "position", Sequence("X","Y","Z")),
                            Compound("not", Compound("registered", "ObjectID")))

        persons_answers = self.robot.reasoner.query(person_query)

        if not persons_answers:
            return "no_person_found"
        
        if len(persons_answers) > 1:
            self.robot.speech.speak("I see some people!")
        else:
            self.robot.speech.speak("I found someone!")

        # def distance_to_base(xyphi_tup):
        #         x = float(xyphi_tup[0])
        #         y = float(xyphi_tup[1])

        #         origX = self.robot.base.location[0].x
        #         origY = self.robot.base.location[0].y

        #         dist = math.sqrt(abs(origX-x)**2 + abs(origY-y)**2)
        #         return dist

        #####################################################################################
        # TODO Loy: 
        # - from answers of person_query, get the x,y,z value with shortest distance.
        # - most important is to know what the ObjectID is. This ObjectID should be asserted as registered(already done below))
        # - assert x,y,z value to reasoner.
        # - formualte new query for NavigateGeneric with only this answer possible
        # - run NavigateGeneric with lookat_query=..new query..
        #####################################################################################


        # persons = (float(persons_answers["X"]), float(persons_answers["Y"]), float(persons_answers["Z"]))
        # rospy.loginfo("persons = {0}".format(persons))
        
        # person_names = persons_answers["ObjectID"]
        # rospy.loginfo("waypoint_name = {0}".format(person_names))
        

        # x,y,phi = min(goals, key=distance_to_base)

        # rospy.loginfo("x = {0}, y = {1}, phi = {2}".format(x, y, phi))


        #nav = states.NavigateGeneric(self.robot, lookat_query=person_query) 
        #nav = states.NavigateGeneric(self.robot, lookat_query=person_query, goal_sorter=distance_to_base) 

        nav = Navigate_to_queryoutcome_point_emergency(self.robot, person_query, X="X", Y="Y", Z="Z")

        nav_result = nav.execute()

        if nav_result == "unreachable" or nav_result == "preempted":
            self.robot.reasoner.query(Conjunction(Compound("current_person", "ObjectID"),
                                                      Compound("assert", Compound("registered", "ObjectID"))))
            return "person_unreachable"

        elif nav_result == "goal_not_defined":
            rospy.loginfo("Goal not defined received by NavigateGeneric. Should never happen, since this check has done before calling upon NavigateGeneric.")
            return "no_person_found"

        self.robot.reasoner.query(Conjunction(Compound("current_person", "ObjectID"),
                                              Compound("assert", Compound("registered", "ObjectID"))))                 

        return "person_found"


class Navigate_to_queryoutcome_point_emergency(states.Navigate_abstract):
    """Move to the output of a query, which is passed to this state as a Term from the reasoner-module.
    
    The state can take some parameters that specify which keys of the dictionary to use for which data.
    By default, the binding-key "X" refers to the x-part of the goal, etc. 
    
    Optionally, also a sorter can be given that sorts the bindings according to some measure.
    """
    def __init__(self, robot, query, X="X", Y="Y", Z="Z"):
        states.Navigate_abstract.__init__(self, robot)

        assert isinstance(query, Term)

        self.queryTerm = query
        self.X, self.Y, self.Z = X, Y, Z
        
    def get_goal(self, userdata):
        """self.get_goal gets the answer to this query and lets it parse it into a list of binding-dictionaries. """
        
        self.robot.reasoner.query(Compound("retractall", Compound("current_person", "X"))) 
        
        # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
        # is a mapping of variable to a constant, like a string or number
        answers = self.robot.reasoner.query(self.queryTerm)

        if not answers:
            return None
            rospy.logerr("No answers found for query {query}".format(query=self.queryTerm))
        else:
            chosen_answer = answers[0]
            #From the summarized answer, 
            possible_locations = [( float(answer[self.X]), 
                                    float(answer[self.Y]), 
                                    float(answer[self.Z])) for answer in answers]

            x,y,z = possible_locations[0]
            
            goal = possible_locations[0]
            rospy.loginfo("goal = {0}".format(goal))
            waypoint_name = chosen_answer["ObjectID"]

            
            self.robot.reasoner.query(Compound("assert", Compound("current_person", waypoint_name))) 

            rospy.logdebug("Found location for '{0}': {1}".format(self.queryTerm, (x,y,z)))

            look_point = geometry_msgs.msg.PointStamped()
            look_point.point = self.robot.base.point(x,y)
            pose = states.util.msg_constructors.Quaternion(z=1.0)

            base_poses_for_point = self.robot.base.get_base_goal_poses(look_point, 0.8, 0.0)
            if base_poses_for_point:
                base_pose_for_point = base_poses_for_point[0]
            else:
                rospy.logerr("IK returned empty pose.")
                return look_point.point, pose  #outWhen the IK pose is empty, just try to drive to the point itself. Will likely also fail.
                
            if base_pose_for_point.pose.position.x == 0 and base_pose_for_point.pose.position.y == 0:
                rospy.logerr("IK returned empty pose.")
                return look_point.point, pose  #outWhen the IK pose is empty, just try to drive to the point itself. Will likely also fail.

            return base_pose_for_point.pose.position, base_pose_for_point.pose.orientation


class Ask_yes_no(smach.State):
    def __init__(self, robot, tracking=True, rate=2):
        smach.State.__init__(self, outcomes=["yes", "preempted", "no"])

        self.robot = robot
        self.preempted = False
        self.rate = rate
        self.get_yes_no_service = rospy.ServiceProxy('interpreter/get_yes_no', GetYesNo)

    def execute(self, userdata=None):

        self.response = self.get_yes_no_service(2 , 8) # 3 tries, each max 10 seconds

        if self.response.answer == "true":
            return "yes"
        else:
            return "no"

# Class for moving arm back for guidance
class MoveArmBack(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot

    def execute(self, userdata=None):      
        rospy.loginfo("Moving arm back")
    
        self.robot.leftArm.send_joint_goal(0.0,-1.57,0.0,1.57,0.0,0.0,0.0)
        self.robot.rightArm.send_joint_goal(0.0,-1.57,0.0,1.57,0.0,0.0,0.0)
        rospy.sleep(1.5)
        self.robot.leftArm.send_gripper_goal_close(10)
        self.robot.rightArm.send_gripper_goal_close(10)
        
        return 'finished'

# Class for registering person
class Register(smach.State):
    def __init__(self, robot=None, status=1):
        smach.State.__init__(self, outcomes=['finished'])
    
        self.robot = robot

        self.status = status

        self.get_picture = rospy.ServiceProxy('virtual_cam/cheese', cheese)

    def execute(self, userdata=None):    

        return_result = self.robot.reasoner.query(Compound("register_person_no", "X"))
        person_no = float(return_result[0]["X"])  

        #First move head to look at person where face is detected
        person_query = Conjunction( 
                            Compound("current_person","ObjectID"),
                            Compound("property_expected","ObjectID", "position", Sequence("X","Y","Z")))

        answers = self.robot.reasoner.query(person_query)

        if not answers:            
            rospy.logerr("No answers found for query. SHOULD NOT HAPPEN!! Query: {query}".format(query=person_query))
            pos, rot = self.robot.base.get_location()
            x = pos.x
            y = pos.y
        else:
            possible_locations = [( float(answer["X"]), 
                                    float(answer["Y"]), 
                                    float(answer["Z"])) for answer in answers]

            x,y,z = possible_locations[0]

        rospy.logdebug("[EG] status of person is {0} (1 = not oke, 0 is oke)".format(self.status))
        rospy.logdebug("[EG] person_no = {0}".format(person_no))
        rospy.logdebug("[EG] x value person = {0}".format(x))
        rospy.logdebug("[EG] y value person = {0}".format(y))

        # Register person
        rospy.logdebug("Register person in file ....")
        if person_no == 1:
            f = open(p.get_pkg_dir('challenge_emergency')+'/output/status.txt','w')  # 'w' means write
            
        else:
            f = open(p.get_pkg_dir('challenge_emergency')+'/output/status.txt','a')  # 'a' means append

        #f = open('status.txt','a')
        if self.status == 0:
            f.write('person_%d;0;' % person_no)  # 0 is not oke; 1 is ok and ;2 fire
            f.write('%.2f;%.2f \n' % (x,y))
            f.close()
            pathname = "/home/amigo/ros/fuerte/tue/trunk/tue_robocup/challenge_emergency/output/person_%d.png" % person_no

        elif self.status == 1:
            f.write('person_%d;1;' % person_no)  # 0 is not oke; 1 is ok and ;2 fire
            f.write('%.2f;%.2f \n' % (x, y))
            f.close()
            pathname = "/home/amigo/ros/fuerte/tue/trunk/tue_robocup/challenge_emergency/output/person_%d.png" % person_no

        elif self.status == 2:
            f.write('fire_1;2;')                # 0 is not oke; 1 is ok and ;2 fire
            f.write('%.2f;%.2f \n' % (x, y))  # TODO ERIK exact location stove
            f.close()
            pathname = "/home/amigo/ros/fuerte/tue/trunk/tue_robocup/challenge_emergency/output/fire.png"
        
        
        rospy.logdebug("pathname = {0}".format(pathname))

        ### TODO/IDEA: play sound of taking picture

        if self.get_picture(pathname):  # bool terug. (filename met hele pad.png)
            rospy.logdebug("picture taken!!")
        else:
            rospy.logdebug("no picture taken, i'm sorry")       

        person_no += 1
        self.robot.reasoner.query(Compound("retractall", Compound("register_person_no", "X")))
        self.robot.reasoner.query(Compound("assertz",Compound("register_person_no", person_no)))
        
        return 'finished'

# Class to move arm to initial pose
class MoveArmInitial(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot

    def execute(self, userdata=None):   

        rospy.loginfo("Moving arm back")
    
        self.robot.leftArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
        self.robot.rightArm.send_joint_goal(-0.1,-0.2,0.2,0.8,0.0,0.0,0.0)
        rospy.sleep(1.5)
        self.robot.leftArm.send_gripper_goal_close(10)
        self.robot.rightArm.send_gripper_goal_close(10)

        return 'finished'


class Run_pdf_creator(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done', 'failed'])

        self.robot = robot
        self.startup_pdf_creator = rospy.ServiceProxy('/emergency_paper/startup', Start)

    def execute(self, userdata=None):

        try:
            self.response = self.startup_pdf_creator()  # starts pdf creator
        except Exception, e:
        #except KeyError, ke:
            print e
            rospy.loginfo("FAILED creating pdf on usb-stick")
            return "failed"
        rospy.loginfo("PDF is created on usb-stick")
        
        #rospy.loginfo("[EG] DELETE SLEEP AFTER TESTING")
        #rospy.sleep(60)
        
        return "done"


class Look_at_person(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot

    def execute(self, userdata=None):    

        #First move head to look at person where face is detected
        person_query = Conjunction( 
                            Compound("current_person","ObjectID"),
                            Compound("property_expected","ObjectID", "position", Sequence("X","Y","Z")))

        answers = self.robot.reasoner.query(person_query)

        if not answers:            
            rospy.logerr("No answers found for query. SHOULD NOT HAPPEN!! Query: {query}".format(query=person_query))
            pos, rot = self.robot.base.get_location()
            x = pos.x
            y = pos.y
        else:
            possible_locations = [( float(answer["X"]), 
                                    float(answer["Y"]), 
                                    float(answer["Z"])) for answer in answers]

            x,y,z = possible_locations[0]

            if z > 1.5:
                self.robot.spindle.send_goal(0.4,waittime=5.0)  
                rospy.logdebug("Spindle should come up now!")

            lookat_point = self.robot.head.point(x,y,z)
            rospy.loginfo("AMIGO should look at person now. (x = {0}, y = {1}, z = {2})".format(x,y,z))
            self.robot.head.send_goal(lookat_point, timeout=0, keep_tracking=True)
        return 'finished'

class Check_waypoints_available(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['available','not_available'])

        self.robot = robot

    def execute(self, userdata=None):    

        navigate_apartment = Conjunction(  Compound("=", "Waypoint", Compound("apartment", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint")),
                                                 Compound("not", Compound("unreachable", "Waypoint")))

        goal_answers = self.robot.reasoner.query(navigate_apartment)             # I do not use not_visited and not_unreachable since these are no roi's

        if not goal_answers:
            return "not_available"     
        else:
            return "available"


class Sleep_1_sec(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot

    def execute(self, userdata=None):    

        rospy.sleep(1)
        return "finished"


def setup_statemachine(robot):

    # Retract old facts
    robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
    robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
    robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
    robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
    robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
    robot.reasoner.query(Compound("retractall", Compound("not_visited", "X")))
    robot.reasoner.query(Compound("retractall", Compound("not_unreachable", "X")))
    robot.reasoner.query(Compound("retractall", Compound("registered", "X")))
    robot.reasoner.query(Compound("retractall", Compound("visited", "X")))
    robot.reasoner.query(Compound("retractall", Compound("unreachable", "X")))

    # Emergency parameters
    robot.reasoner.query(Compound("retractall", Compound("current_person", "X")))
    robot.reasoner.query(Compound("retractall", Compound("register_person_no", "X")))
    robot.reasoner.query(Compound("assert",Compound("register_person_no", "1")))

    # Load database
    robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))

    # Assert the current challenge.
    robot.reasoner.query(Compound("assertz",Compound("challenge", "emergency")))

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:

        ######################################################
        ##################### ENTER ROOM #####################
        ######################################################

        # Start challenge via StartChallengeRobust
        smach.StateMachine.add( "START_CHALLENGE",
                                    states.StartChallengeRobust(robot, "initial"), 
                                    transitions={   "Done":"SAY_GO_TO_KITCHEN", 
                                                    "Aborted":"SAY_GO_TO_KITCHEN", 
                                                    "Failed":"SAY_GO_TO_KITCHEN"})   # There is no transition to Failed in StartChallengeRobust (28 May)

        smach.StateMachine.add("SAY_GO_TO_KITCHEN",
                                    states.Say(robot, "I will go to the kitchen, see if I can do something over there.", block=False),
                                    transitions={   "spoken":"GO_TO_KITCHEN"})
        
        ######################################################
        #################### GO TO KITCHEN ###################
        ######################################################

        query_kitchen_1 = Compound("waypoint", "kitchen_1", Compound("pose_2d", "X", "Y", "Phi"))
        # Then amigo will drive to the registration table. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_KITCHEN', 
                                    states.Navigate_to_queryoutcome(robot, query_kitchen_1, X="X", Y="Y", Phi="Phi"),
                                    transitions={   'arrived':'RESET_HEAD', 
                                                    'preempted':'GO_TO_KITCHEN_SECOND_TRY', 
                                                    'unreachable':'GO_TO_KITCHEN_SECOND_TRY', 
                                                    'goal_not_defined':'GO_TO_KITCHEN_SECOND_TRY'})

        query_kitchen_2 = Compound("waypoint", "kitchen_2", Compound("pose_2d", "X", "Y", "Phi"))
        # Then amigo will drive to the registration table. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_KITCHEN_SECOND_TRY', 
                                    states.Navigate_to_queryoutcome(robot, query_kitchen_2, X="X", Y="Y", Phi="Phi"),
                                    transitions={   'arrived':'RESET_HEAD', 
                                                    'preempted':'FAIL_BUT_START_SEARCH', 
                                                    'unreachable':'FAIL_BUT_START_SEARCH', 
                                                    'goal_not_defined':'FAIL_BUT_START_SEARCH'})

        smach.StateMachine.add("RESET_HEAD",
                                states.ResetHead(robot),
                                transitions={'done':'SET_TIME_MARKER'})

        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('FAIL_BUT_START_SEARCH',
                                    states.Say(robot, "I was not able to go to the kitchen, I will just wait here.", block=False),  #LOCATION SHOULD BE FOUND, otherwise sentence is to long for non-blocking
                                    transitions={'spoken':'SET_TIME_MARKER'}) 

        smach.StateMachine.add('SET_TIME_MARKER',
                                    states.SetTimeMarker(robot, "find_fire"),
                                    transitions={   'done':'CHECK_TIME' })

        smach.StateMachine.add('CHECK_TIME',
                                    states.CheckTime(robot, "find_fire", rospy.Duration(55)),
                                    transitions={   'ok':'SAY_WEATHER_QUEEN',
                                                    'timeout':'SAY_FIRE_ALARM' })  

        smach.StateMachine.add('SAY_WEATHER_QUEEN',
                                    states.Say(robot, "Although it is a rainy day, it will be lovely! I just can't wait to meet the queen in a few hours!", block=False),  
                                    transitions={'spoken':'CHECK_TIME_2'}) 

        smach.StateMachine.add('CHECK_TIME_2',
                                    states.CheckTime(robot, "find_fire", rospy.Duration(55)),
                                    transitions={   'ok':'SLEEP_1_SEC',
                                                    'timeout':'SAY_FIRE_ALARM' })   

        smach.StateMachine.add('SLEEP_1_SEC',
                                    Sleep_1_sec(robot),
                                    transitions={   'finished':'CHECK_TIME_2'}) 

        # Say that something is burning
        smach.StateMachine.add('SAY_FIRE_ALARM',
                                    states.Say(robot, "Oh no! I hear the fire alarm, I will go look for people and help them!"), # Deze blocken!!
                                    transitions={'spoken':'FIND_PEOPLE'}) 



        ######################################################
        ################# DETECT FIRE/SMOKE ##################
        ######################################################
        '''
        TODOS: In the rulebook it is said that smoke should be detected within 1 minute and that the robot should look and point 
        from a close distance (less than 1 m) and take a picture. 
        -> Build timer 1 minute!!
        -> Detect fire
        -> Make picture
        '''
        # Inform people
        smach.StateMachine.add('SAY_NO_FIRE',
                                    states.Say(robot, 'I cannot find any fire. Anyway, I will go look for people and help them!'),
                                    transitions={'spoken':'FIND_PEOPLE'})



        # # Turn 360 degrees to search for the smoke
        # smach.StateMachine.add('TURN_ROUND_Z_AXIS',
        #                             turn_Around_z_axis(robot, 10),
        #                             transitions={   'done':'SAY_NEXT_LOCATION',    ###### 
        #                                             'abort':'SAY_NEXT_LOCATION'})
        # # Inform people
        # smach.StateMachine.add('SAY_NEXT_LOCATION',
        #                             states.Say(robot, 'Did not find fire. Maybe at the other side of the room'),
        #                             transitions={'spoken':'NEXT_LOCATION'})

        # # Query reasoner to find other side of goal
        # query_kitchen_3 = Compound("waypoint", "kitchen_3", Compound("pose_2d", "X", "Y", "Phi"))
        # smach.StateMachine.add('NEXT_LOCATION',
        #                         states.Navigate_to_queryoutcome(robot, query_kitchen_3, X="X", Y="Y", Phi="Phi"),
        #                         transitions={   "arrived":"SAY_DETECT_SMOKE",
        #                                         "unreachable":'SAY_DETECT_SMOKE_FAILED_REACHING_KITCHEN',
        #                                         "preempted":'SAY_DETECT_SMOKE_FAILED_REACHING_KITCHEN',
        #                                         "goal_not_defined":'SAY_DETECT_SMOKE_FAILED_REACHING_KITCHEN'})
        
        # # Time ran out say, you couldn't find smoke, but register
        # smach.StateMachine.add('SAY_DETECT_SMOKE',
        #                             states.Say(robot, 'Time ran out, could not find fire, but I smell something burning. Therefore I will register a picture of the stove.'),
        #                             transitions={'spoken':'REGISTER_FIRE'}) 

        # # Time ran out say, you couldn't find smoke, but register
        # smach.StateMachine.add('SAY_DETECT_SMOKE_FAILED_REACHING_KITCHEN',
        #                             states.Say(robot, 'Time ran out, could not find fire, but I smell something burning. Therefore I will register a picture of the stove.'),
        #                             transitions={'spoken':'NEXT_LOCATION_2ND_TRY'})  

        # smach.StateMachine.add('NEXT_LOCATION_2ND_TRY',
        #                         states.Navigate_to_queryoutcome(robot, query_kitchen_3, X="X", Y="Y", Phi="Phi"),
        #                         transitions={   "arrived":"REGISTER_FIRE",
        #                                         "unreachable":'REGISTER_FIRE',
        #                                         "preempted":'REGISTER_FIRE',
        #                                         "goal_not_defined":'REGISTER_FIRE'})

        # smach.StateMachine.add('REGISTER_FIRE',
        #                             Register(robot, 2),                            #input 2 (fire)
        #                             transitions={'finished':'SAY_FIND_FIRST_PEOPLE'})

        # # Time ran out say, you couldn't find smoke, but register
        # smach.StateMachine.add('SAY_FIND_FIRST_PEOPLE',
        #                             states.Say(robot, 'Now I am going to look for people!'),
        #                             transitions={'spoken':'FIND_PEOPLE'})  

        '''
        Idea: FLASH RED light on and off after detecting smoke until the end of the challenge (not during interpretation). 
        Then say: "If anyone can see or hear me, 
        try to get to the exit!!"
        '''

        ######################################################
        ################### SAVING PERSONS ###################
        ######################################################

        ############ DRIVE TO PREDEFINED LOCATIONS ###########

        smach.StateMachine.add('SAY_FIND_PEOPLE',
                                    states.Say(robot, 'I am going to look for more people.'),
                                    transitions={'spoken':'FIND_PEOPLE'})

        smach.StateMachine.add("FIND_PEOPLE",
                                Look_for_people_emergency(robot),
                                transitions={   'visited':'SAY_START_PEOPLE_DETECTION',
                                                'unreachable':'FAILED_DRIVING_TO_LOCATION',
                                                'all_matches_tried':'SAY_GO_TO_EXIT'})

        # Could not reach ROI     
        smach.StateMachine.add("FAILED_DRIVING_TO_LOCATION",
                                states.Say(robot,"I was not able to reach the desired location to detect people. I will try another location.", block=False),  #LOCATION SHOULD BE FOUND, otherwise sentence is to long for non-blocking
                                transitions={'spoken':'FIND_PEOPLE'})

        ############ DRIVE TO PREDEFINED LOCATIONS ###########

        # Start people detection
        smach.StateMachine.add("SAY_START_PEOPLE_DETECTION",
                                states.Say(robot,"Please look into my eyes. I will start my perception now.", block=False),
                                transitions={'spoken':'START_PEOPLE_DETECTION'})

        smach.StateMachine.add("START_PEOPLE_DETECTION",
                                Looking_for_people(robot),
                                transitions={'done':'CHECK_WORLD_MODEL_FOR_UNREGISTERED_PEOPLE',
                                             'failed':'FAILED_PERCEPTION'})

        # Could not reach ROI    
        smach.StateMachine.add("FAILED_PERCEPTION",
                                states.Say(robot,"I failed starting perception, maybe more luck at the next location.", block=False),  #LOCATION SHOULD BE FOUND, otherwise sentence is to long for non-blocking
                                transitions={'spoken':'FIND_PEOPLE'})


        smach.StateMachine.add("CHECK_WORLD_MODEL_FOR_UNREGISTERED_PEOPLE",
                                Check_persons_found(robot),
                                transitions={'no_person_found':'SAY_NO_PERSON_FOUND',
                                             'person_unreachable':'SAY_PERSON_UNREACHABLE',
                                             'person_found':'LOOK_AT_PERSON'})

        smach.StateMachine.add("SAY_NO_PERSON_FOUND",
                                states.Say(robot,"I do not see people over here", block=False),  #LOCATION SHOULD BE FOUND, otherwise sentence is to long for non-blocking
                                transitions={'spoken':'NO_PERSON_FOUND'})

        Check_waypoints_available
        smach.StateMachine.add("NO_PERSON_FOUND",
                                Check_waypoints_available(robot),
                                transitions={'available':'SAY_NEXT_LOCATION',
                                             'not_available':'SAY_GO_TO_EXIT'})

        smach.StateMachine.add("SAY_NEXT_LOCATION",
                                states.Say(robot,"I will try the next location.", block=False),  #LOCATION SHOULD BE FOUND, otherwise sentence is to long for non-blocking
                                transitions={'spoken':'FIND_PEOPLE'})

        smach.StateMachine.add("CHECK_WORLD_MODEL_FOR_MORE_UNREGISTERED_PEOPLE",
                                Check_persons_found(robot),
                                transitions={'no_person_found':'SAY_FIND_PEOPLE',
                                             'person_unreachable':'SAY_PERSON_UNREACHABLE',
                                             'person_found':'LOOK_AT_PERSON'})

        smach.StateMachine.add("SAY_PERSON_UNREACHABLE",
                                states.Say(robot,["I failed going to the desired person", "I am not able to reach the person I want to speak"], block=False),
                                transitions={'spoken':'GO_TO_LAST_EXPLORATION_POINT'})

        ## hier state toevoegen, terugrijden naar lookat point.

        query_last_exploration_location = Conjunction(Compound("current_exploration_target", "Location"),
                                                      Compound("waypoint", "Location", Compound("pose_2d", "X", "Y", "Phi")))

        smach.StateMachine.add('GO_TO_LAST_EXPLORATION_POINT', 
                                    states.Navigate_to_queryoutcome(robot, query_last_exploration_location, X="X", Y="Y", Phi="Phi"),
                                    transitions={   'arrived':'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE', 
                                                    'preempted':'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE', 
                                                    'unreachable':'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE', 
                                                    'goal_not_defined':'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE'})

        smach.StateMachine.add("SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE",
                                states.Say(robot,["I will ask my questions from here.", "I will ask you some questions from here."], block=False),
                                transitions={'spoken':'LOOK_AT_PERSON'})

        ############### GET INFO STATUS PERSON ###############

        # Look at person
        smach.StateMachine.add('LOOK_AT_PERSON',
                                    Look_at_person(robot),                          
                                    transitions={'finished':'DETECT_PEOPLE'})   

        # People detection
        smach.StateMachine.add('DETECT_PEOPLE',
                                    states.Say(robot, [ "Hello, are you able to walk?",
                                                        "Hi, are you able to move?"]),
                                    transitions={'spoken':'ANSWER_ARE_YOU_OKAY'})
        # Await anser of question 1
        smach.StateMachine.add('ANSWER_ARE_YOU_OKAY',
                                    Ask_yes_no(robot),
                                    transitions={   'yes':'ASK_IF_KNOWS_DIRECTION',
                                                    'preempted':'SAY_REGISTER_NOT_OKAY',
                                                    'no':'SAY_REGISTER_NOT_OKAY'})      

        # Person is ok and is asked if he/she knows the way
        smach.StateMachine.add('ASK_IF_KNOWS_DIRECTION',
                                    states.Say(robot, 'Do you know the way to the exit'),
                                    transitions={'spoken':'ANSWER_IF_KNOWS_DIRECTION'})

        # Await question of he/she knows the way
        smach.StateMachine.add('ANSWER_IF_KNOWS_DIRECTION',
                                    Ask_yes_no(robot),
                                    transitions={   'yes':'SAY_REGISTER_OKAY_EXIT_BY_THEMSELVES',       
                                                    'preempted':'SAY_REGISTER_OKAY_EXIT_NOT_BY_THEMSELVES',
                                                    'no':'SAY_REGISTER_OKAY_EXIT_NOT_BY_THEMSELVES'})

        ################## REGISTER PERSON ###################

        # Person is not okay:
        smach.StateMachine.add('SAY_REGISTER_NOT_OKAY',
                                    states.Say(robot, 'I will just register your position and take a picture so the fire department is able to find you.', block=True),
                                    transitions={'spoken':'REGISTER_PERSON_NOT_OKAY'})     

        smach.StateMachine.add('REGISTER_PERSON_NOT_OKAY',
                                    Register(robot, status=0),                             #input 0 (person is not oke)
                                    transitions={'finished':'SAVE_PDF_ON_STICK_PERSON_NOT_OKAY'})

        smach.StateMachine.add('SAVE_PDF_ON_STICK_PERSON_NOT_OKAY',
                                    Run_pdf_creator(robot),
                                    transitions={'done':'SAY_REGISTER_NOT_OKAY_CALM',
                                                 'failed':'SAY_REGISTER_NOT_OKAY_CALM'})

        smach.StateMachine.add('SAY_REGISTER_NOT_OKAY_CALM',
                                    states.Say(robot, 'Please stay calm and help will arrive very soon.'), 
                                    transitions={'spoken':'CHECK_WORLD_MODEL_FOR_MORE_UNREGISTERED_PEOPLE'})     

        # Person is ok and needs no assistance to exit
        smach.StateMachine.add('SAY_REGISTER_OKAY_EXIT_BY_THEMSELVES',
                                    states.Say(robot, 'I will register your position and take a picture.', block=True),
                                    transitions={'spoken':'REGISTER_PERSON_OKAY_EXIT_BY_THEMSELVES'})     

        smach.StateMachine.add('REGISTER_PERSON_OKAY_EXIT_BY_THEMSELVES',
                                    Register(robot, status=1),                            #input 1 (person is oke)
                                    transitions={'finished':'SAVE_PDF_ON_STICK_PERSON_OKAY_EXIT_BY_THEMSELVES'})

        smach.StateMachine.add('SAVE_PDF_ON_STICK_PERSON_OKAY_EXIT_BY_THEMSELVES',
                                    Run_pdf_creator(robot),
                                    transitions={'done':'SAY_MOVE_TO_EXIT',
                                                 'failed':'SAY_MOVE_TO_EXIT'})

        smach.StateMachine.add('SAY_MOVE_TO_EXIT',
                                    states.Say(robot, 'Okay, please go to the exit and leave the room'),
                                    transitions={'spoken':'CHECK_WORLD_MODEL_FOR_MORE_UNREGISTERED_PEOPLE'})

        # Person is ok and needs assistance to exit
        smach.StateMachine.add('SAY_REGISTER_OKAY_EXIT_NOT_BY_THEMSELVES',
                                    states.Say(robot, 'Before escorting you to the exit, I will first register your position and take a picture.', block=True),
                                    transitions={'spoken':'REGISTER_PERSON_OKAY_EXIT_NOT_BY_THEMSELVES'})     

        smach.StateMachine.add('REGISTER_PERSON_OKAY_EXIT_NOT_BY_THEMSELVES',
                                    Register(robot, status=1),                            #input 1 (person is oke)
                                    transitions={'finished':'SAVE_PDF_ON_STICK_PERSON_OKAY_EXIT_NOT_BY_THEMSELVES'})

        smach.StateMachine.add('SAVE_PDF_ON_STICK_PERSON_OKAY_EXIT_NOT_BY_THEMSELVES',
                                    Run_pdf_creator(robot),
                                    transitions={'done':'MOVE_ARM_BACK_SAY',
                                                 'failed':'MOVE_ARM_BACK_SAY'})
        
        ################### GUIDE TO EXIT ####################

        smach.StateMachine.add('MOVE_ARM_BACK_SAY',
                                    states.Say(robot, 'I will turn around and move my arms to the back, so that you can hold on to them', block=False),
                                    transitions={'spoken':'MOVE_ARM_BACK_TURN'})

        # Turn 360 degrees (will be 3/4 of a round)
        smach.StateMachine.add('MOVE_ARM_BACK_TURN',
                                    turn_Around_z_axis(robot, 3.14),
                                    transitions={   'done':'MOVE_ARM_BACK', 
                                                    'abort':'MOVE_ARM_BACK'})
        
        # Move arm back when person needs guidance
        smach.StateMachine.add('MOVE_ARM_BACK',
                                    MoveArmBack(robot),
                                    transitions={'finished':'SAY_FOLLOW_ME'})

        # Person is ok and needs assistance to exit
        smach.StateMachine.add('SAY_FOLLOW_ME',
                                    states.Say(robot, 'Please hold on to one of my arms and I will drive to the exit.'),
                                    transitions={'spoken':'GO_TO_FRONT_OF_EXIT'})


        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_FRONT_OF_EXIT', 
                                    states.Navigate_named(robot, "front_of_exit"),  
                                    transitions={   'arrived':'SAY_GO_THROUGH_EXIT', 
                                                    'preempted':'CLEAR_PATH_TO_FRONT_OF_EXIT', 
                                                    'unreachable':'CLEAR_PATH_TO_FRONT_OF_EXIT', 
                                                    'goal_not_defined':'CLEAR_PATH_TO_FRONT_OF_EXIT'})

        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('CLEAR_PATH_TO_FRONT_OF_EXIT',
                                    states.Say(robot, "I could not go to the exit. Please clear the path, I will give it another try."),
                                    transitions={'spoken':'GO_TO_FRONT_OF_EXIT_SECOND_TRY'}) 

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_FRONT_OF_EXIT_SECOND_TRY', 
                                    states.Navigate_named(robot, "front_of_exit"),   
                                    transitions={   'arrived':'SAY_GO_THROUGH_EXIT', 
                                                    'preempted':'FAILED_GO_TO_FRONT_OF_EXIT', 
                                                    'unreachable':'FAILED_GO_TO_FRONT_OF_EXIT', 
                                                    'goal_not_defined':'FAILED_GO_TO_FRONT_OF_EXIT'})

        # Person is ok and needs assistance to exit
        smach.StateMachine.add('FAILED_GO_TO_FRONT_OF_EXIT',
                                    states.Say(robot, 'I was not able to go to the exit, try to find it yourself or wait here for help since your location is registered.'),
                                    transitions={'spoken':'MOVE_ARM_INITIAL'})
        
        # Person is ok and needs assistance to exit
        smach.StateMachine.add('SAY_GO_THROUGH_EXIT',
                                    states.Say(robot, 'You can find the exit 1 meter in front of me, please go through it!'),
                                    transitions={'spoken':'MOVE_ARM_INITIAL'})

        # When arrived at exit move arm to natural look
        smach.StateMachine.add('MOVE_ARM_INITIAL',
                                    MoveArmInitial(robot),
                                    transitions={'finished':'CHECK_WORLD_MODEL_FOR_MORE_UNREGISTERED_PEOPLE'})

        ######################################################
        ###################### FINISHED ######################
        ######################################################
        
        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('SAY_GO_TO_EXIT',
                                    states.Say(robot, "I have searched the whole apartment for people. Therefore I will go to the exit. If people are still here and can hear me, go to the exit!!", block=False),
                                    transitions={'spoken':'GO_TO_EXIT'}) 

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT', 
                                    states.Navigate_named(robot, "exit_1"),
                                    transitions={   'arrived':'SUCCEED_GO_TO_EXIT', 
                                                    'preempted':'CLEAR_PATH_TO_EXIT', 
                                                    'unreachable':'CLEAR_PATH_TO_EXIT', 
                                                    'goal_not_defined':'CLEAR_PATH_TO_EXIT'})

        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('CLEAR_PATH_TO_EXIT',
                                    states.Say(robot, "I couldn't go to the exit. Please clear the path, I will give it another try."),
                                    transitions={'spoken':'GO_TO_EXIT_SECOND_TRY'}) 

        # Then amigo will drive to the registration table. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_EXIT_SECOND_TRY', 
                                    states.Navigate_named(robot, "exit_2"),
                                    transitions={   'arrived':'SUCCEED_GO_TO_EXIT', 
                                                    'preempted':'FAILED_GO_TO_EXIT', 
                                                    'unreachable':'FAILED_GO_TO_EXIT', 
                                                    'goal_not_defined':'FAILED_GO_TO_EXIT'})

        smach.StateMachine.add('FAILED_GO_TO_EXIT',
                                    states.Say(robot, 'I was not able to go to the exit. I will stop here and save all the information I gathered in a PDF file on a USB stick.', block=False),
                                    transitions={'spoken':'SAVE_PDF_ON_STICK'})

        smach.StateMachine.add('SUCCEED_GO_TO_EXIT',
                                    states.Say(robot, 'I will now save all the information I gathered in a PDF file on a USB stick.', block=False),
                                    transitions={'spoken':'SAVE_PDF_ON_STICK'})

        smach.StateMachine.add('SAVE_PDF_ON_STICK',
                                    Run_pdf_creator(robot),
                                    transitions={'done':'AT_END',
                                                 'failed':'AT_END'})
    
        # Finally amigo will stop and says 'goodbye' to show that he's done.
        smach.StateMachine.add('AT_END',
                                states.Say(robot, "Goodbye"),
                                transitions={'spoken':'Done'})

    return sm

if __name__ == "__main__":
    rospy.init_node('emergency_exec')  #, log_level=rospy.DEBUG)
    rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    rospy.loginfo("!!!!!!!!!!!!!!!!!EMERGENCY CHALLENGE!!!!!!!!!!!!!!!!!")
    rospy.loginfo("!!! MAKE SURE DEPENDENCIES LAUNCH FILE IS RUNNING !!!")
    rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    startup(setup_statemachine)
