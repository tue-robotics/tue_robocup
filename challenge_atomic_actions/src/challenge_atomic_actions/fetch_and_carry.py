#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_atomic_actions')
import rospy

# 
from speech_interpreter.srv import AskUser

import smach
from robot_skills.amigo import Amigo
from robot_smach_states import *

from robot_skills.reasoner  import Conjunction, Compound
from robot_smach_states.util.startup import startup

grasp_arm = "left"

class WaitForFetchCarry(smach.State):
    def __init__(self, robot, tracking=True, rate=2):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self.robot = robot
        self.preempted = False
        self.ask_user_service_fetch_carry = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata=None):
        #global starting_pose       
        pose = self.robot.base.location
        starting_pose = pose
        rospy.loginfo("Starting pose xyz {0}".format(starting_pose))
        self.robot.reasoner.query(Compound("assertz",Compound("starting_pose", starting_pose.pose.position.x, starting_pose.pose.position.y, starting_pose.pose.orientation.z)))

        # Here you can define how many times you want to try to listen and want the maximum duration is to listen to operator.
        self.robot.head.set_pan_tilt(tilt=-0.2)
        self.response = self.ask_user_service_fetch_carry("fetch_carry", 10, rospy.Duration(10))
        if self.response:
            if self.response.keys[0] == "answer":

                response_answer = self.response.values[0]
                if response_answer == "no_answer" or  response_answer == "wrong_answer":
                    rospy.loginfo("Object to fetch is not understood: {0} ".format(response_answer))
                    return "failed"

                self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("serve", response_answer))))
                rospy.loginfo("Object to fetch is: {0} ".format(response_answer))
                return "succeeded"
        else:
            return "failed"



        ''' Fallback if speech does not work
        response_answer = "coke"
        self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("serve", response_answer))))
        rospy.loginfo("Object to fetch is not understood: {0} ".format(response_answer))
        return 'succeeded'
        '''
        

class LookForDrink(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["looking" , "found", "not_found"])
        self.robot = robot

    def execute(self, userdata=None):
        # find out what drink we need to get
        return_result = self.robot.reasoner.query(Compound("goal", Compound("serve", "Drink")))

        if not return_result:
            self.robot.speech.speak("I forgot which drink you wanted")
            return "not_found"
        serving_drink = str(return_result[0]["Drink"])  

        # navigate to drink location
        goal_answers = self.robot.reasoner.query(Conjunction(  Compound("=", "Waypoint", Compound("goal", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint"))))

        if not goal_answers:
            self.robot.speech.speak("I want to find the " + serving_drink + ", but I don't know where to go... I'm sorry!")
            return "not_found"

        # for now, take the first goal found
        goal_answer = goal_answers[0]

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

        # look to ROI
        roi_answers = self.robot.reasoner.query(Compound("point_of_interest", waypoint_name, Compound("point_3d", "X", "Y", "Z")))
        if roi_answers:
            rospy.loginfo("Looking at {0}".format(roi_answers))
            roi_answer = roi_answers[0]
            lookat_point = self.robot.head.point(float(roi_answer["X"]),float(roi_answer["Y"]),float(roi_answer["Z"]))
            self.robot.head.send_goal(lookat_point,timeout=0)

        else:
            rospy.loginfo("Failed to find a region")
        # query to detect object, finishes when something found or timeout!
        query_detect_object = Conjunction(Compound("goal", Compound("serve", "Drink")),
                                          Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                          Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")))


        self.robot.speech.speak("Let's see what I can find here")
        #self.response_start = self.robot.perception.toggle(['template_matching'])
        self.response_start = self.robot.perception.toggle(['object_recognition'])
 
        if self.response_start.error_code == 0:
            rospy.loginfo("Object recognition has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Object recognition failed to start")
            self.robot.speech.speak("I was not able to start object recognition.")
            return "not_found"

        wait_machine = Wait_query_true(self.robot, query_detect_object, 7)
        wait_result = wait_machine.execute()

        rospy.loginfo("Object recognition will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Object recognition has stopped")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping object recognition")

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

class GoToStartLocation(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "failed"])

        self.robot = robot
        self.preempted = False
    

    def execute(self, userdata=None):
        
        return_result = self.robot.reasoner.query(Compound("starting_pose", "X", "Y", "Z"))
        if return_result:
            rospy.loginfo("Found starting stuff {0}".format(return_result))
        
        

            goal = (float(return_result[0]["X"]), float(return_result[0]["Y"]), float(1.0))
            nav = NavigateGeneric(self.robot, lookat_point_3d=goal, goal_area_radius=0.5)
            nav.execute()
            return 'done'
        else:
            return 'failed'

class FetchAndCarry(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot
            
        query_grabpoint = Conjunction(  Compound("goal", Compound("serve", "Drink")),
                                           Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                           Compound( "position", "ObjectID", Compound("point", "X", "Y", "Z")))

        # Retract all old facts
        robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
        robot.reasoner.query(Compound("retractall", Compound("goal", "X")))
        robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
        robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
        robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
        robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
        robot.reasoner.query(Compound("retractall", Compound("visited", "X")))
        robot.reasoner.query(Compound("retractall", Compound("registered", "X")))
        robot.reasoner.query(Compound("retractall", Compound("type", "X", "Y")))

        # Load locations and objects from knowledge files
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))
        robot.reasoner.assertz(Compound("challenge", "basic_functionalities"))

        # Use reasonar to store start pose
        robot.reasoner.query(Compound("retractall", Compound("starting_pose", "X", "Y", "Z")))

        

        if grasp_arm == "left":
            arm = robot.leftArm
        if grasp_arm == "right":
            arm = robot.rightArm
    	

        with self:
            smach.StateMachine.add('INITIALIZE',
                                    Initialize(robot),
                                    transitions={   'initialized':'TAKE_ORDER',    
                                                    'abort':'Aborted'})

            smach.StateMachine.add( "TAKE_ORDER",
                                    WaitForFetchCarry(robot),
                                    transitions={   "succeeded":"LOOK_FOR_DRINK",
                                                 	"failed":"TAKE_ORDER"})
                              
            smach.StateMachine.add( 'LOOK_FOR_DRINK',
	                                LookForDrink(robot),
	                                transitions={   "looking":"LOOK_FOR_DRINK",
	                                                "found":'PICKUP_DRINK',
	                                                "not_found":'SAY_DRINK_NOT_FOUND'})

            smach.StateMachine.add( 'PICKUP_DRINK',
                                    GrabMachine(arm, robot, query_grabpoint),
                                    transitions={   "succeeded":"NAVIGATE_TO_START_LOCATION",
                                                    "failed":'LOOK_FOR_DRINK' })

            smach.StateMachine.add( 'SAY_DRINK_NOT_FOUND',
                                    Say(robot, "I could not find the drink you wanted."),
                                    transitions={   'spoken':'Aborted' })  

            smach.StateMachine.add( 'SAY_DRINK_NOT_GRASPED',
                                    Say(robot, "I could not pick up the drink you wanted"),
                                    transitions={   'spoken':'Aborted' }) 


            smach.StateMachine.add( 'NAVIGATE_TO_START_LOCATION',
                                    GoToStartLocation(robot),
                                    transitions={   "done":"PEOPLE_LASER_DETECTION",
                                                    "failed":'PEOPLE_LASER_DETECTION'})

            smach.StateMachine.add( 'PEOPLE_LASER_DETECTION',
                                    DriveToClosestPerson(robot),
                                    transitions={   "Done":"VERIFY_HUMAN",
                                                    "Aborted":"HANDOVER_TO_HUMAN",
                                                    "Failed":"HANDOVER_TO_HUMAN"})
            smach.StateMachine.add( 'VERIFY_HUMAN',
                                    Say(robot,"Still need to verify if you are a human"),
                                    transitions={   "spoken":"HANDOVER_TO_HUMAN"})

            smach.StateMachine.add("HANDOVER_TO_HUMAN",
                                    HandoverToHuman(arm, robot),
                                    transitions={   'succeeded':'REINITIALIZE',
                                                    'failed':'REINITIALIZE'})
            smach.StateMachine.add('REINITIALIZE',
                                    Initialize(robot),
                                    transitions={   'initialized':'Done',    
                                                    'abort':'Aborted'})

if __name__ == "__main__":
    rospy.init_node('fetch_and_carry_exec')
    startup(FetchAndCarry)

   
