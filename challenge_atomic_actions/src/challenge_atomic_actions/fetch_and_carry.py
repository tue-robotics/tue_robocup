#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_atomic_actions')
import rospy

# 
from drive_to_person import DriveToClosestPerson
from speech_interpreter.srv import AskUser
import smach

from robot_skills.amigo import Amigo
from robot_smach_states import *

from robot_skills.reasoner  import Conjunction, Compound
from robot_smach_states.util.startup import startup

grasp_arm = "left"
starting_pose = None

class WaitForFetchCarry(smach.State):
    

    def __init__(self, robot, tracking=True, rate=2):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self.robot = robot
        self.preempted = False
        self.ask_user_service_fetch_carry = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata=None):
        global starting_pose       
        pose = self.robot.base.location
        starting_pose = pose
        rospy.loginfo("Starting pose xyz {0}".format(starting_pose))



        # Here you can define how many times you want to try to listen and want the maximum duration is to listen to operator.
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


        #response_answer = "no_answer"
        #for x in range(0,len(self.response.keys)):
        #    if self.response.keys[x] == "answer":
        #        response_answer = self.response.values[x]

        # Check available options from rulebook!
        #if response_answer == "no_answer" or  response_answer == "wrong_answer":
        #    self.robot.speech.speak("I will just bring you a coke")
        #    response_answer = "coke"

        #import ipdb; ipdb.set_trace()
        #self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("serve", response_answer))))
        #return "succeeded"

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

        self.response_start = self.robot.perception.toggle(['object_recognition'])
 
        if self.response_start.error_code == 0:
            rospy.loginfo("Object recognition has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Object recognition failed to start")
            self.robot.speech.speak("I was not able to start object recognition.")
            looked_no = 0;
            self.robot.reasoner.query(Compound("retractall", Compound("looked_drink_no", "X")))
            self.robot.reasoner.query(Compound("assertz",Compound("looked_drink_no", looked_no)))
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
            looked_no = 0;
            self.robot.reasoner.query(Compound("retractall", Compound("looked_drink_no", "X")))
            self.robot.reasoner.query(Compound("assertz",Compound("looked_drink_no", looked_no)))
            return "found"

class CompareWithExpectedPeopleDetector(smach.StateMachine):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done","Aborted"])
        self.robot = robot

    def execute(self, userdata=None):
        return_result = self.robot.reasoner.query(Compound("person", "X"))
        rospy.loginfo('Persons found at {0}'.format(return_result))

class DriveToExpectedPersonLocation(smach.StateMachine):
    """Scan (with the torso laser) for persons, 
    go to the seed that matches an expected location"""

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])

        self.robot = robot
        self.human_query = Conjunction( Compound("instance_of", "ObjectID", "person"), 
                                        Compound( "position", "ObjectID", Compound("point", "X", "Y", "Z")))

        with self:
            smach.StateMachine.add( "TOGGLE_PEOPLE_DETECTION",
                                    TogglePeopleDetector(robot, on=True),
                                    transitions={   "toggled":"WAIT_FOR_DETECTION"})

            smach.StateMachine.add( "WAIT_FOR_DETECTION",
                                    Wait_query_true(robot, self.human_query, timeout=5),
                                    transitions={   "query_true":"TOGGLE_OFF_OK",
                                                    "timed_out":"Failed",
                                                    "preempted":"Aborted"})

            smach.StateMachine.add( "TOGGLE_OFF_OK",
                                    TogglePeopleDetector(robot, on=False),
                                    transitions={   "toggled":"GOTO_PERSON"})

            #smach.StateMachine.add( "CHECK_PERSON",
            #                        CompareWithExpectedPeopleDetector(robot),
            #                        transitions={   "Done":"GOTO_PERSON",
            #                                        "Aborted":"GOTO_PERSON"})

            ### NOW WE HAVE A LIST OF POSSIBLE PEOPLE
            # Do Something smart here
            # And if not go to the next closests one!
            # or in executive!?

            smach.StateMachine.add( "GOTO_PERSON",
                                    NavigateGeneric(robot, lookat_query=self.human_query, xy_dist_to_goal_tuple=(1.0,0)),
                                    transitions={   "arrived":"LOOK_UP_FOR_SAY",
                                                    "unreachable":'SAY_FAILED',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'SAY_FAILED'})
            
            smach.StateMachine.add( "LOOK_UP_FOR_SAY",
                                  ResetHead(robot),
                                  transitions={"done":"SAY_SOMETHING"})

            smach.StateMachine.add( "SAY_SOMETHING",
                                  Say(robot, ["I found someone!"]),
                                  transitions={"spoken":"Done"})

            smach.StateMachine.add( "SAY_FAILED",
                                  Say(robot, ["I didn't find anyone"]),
                                  transitions={"spoken":"LOOK_UP_RESET"})

            smach.StateMachine.add( "LOOK_UP_RESET",
                                  ResetHead(robot),
                                  transitions={"done":"Failed"})

class GoToStartLocation(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "failed"])

        self.robot = robot
        self.preempted = False
        

    def execute(self, userdata=None):
        global starting_pose
        rospy.loginfo("Starting pose xyz {0}".format(starting_pose))

        goal = (float(starting_pose.pose.position.x), float(starting_pose.pose.position.y), float(starting_pose.pose.orientation.z))
        nav = NavigateGeneric(self.robot, goal_pose_2d=goal,goal_area_radius=1.0)
        nav_result = nav.execute()
        return 'done'

class FetchAndCarry(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot

        query_fetch_location = Compound("waypoint", "goal", Compound("pose_2d", "X", "Y", "Phi"))
        query_start_location = Compound("waypoint", "start_location", Compound("pose_2d", "X", "Y", "Phi"))
        #query_grabpoint = Conjunction(  Compound("current_object", "ObjectID"),
        #                               Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))
    
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
        robot.reasoner.assertz(Compound("challenge", "fetch_and_carry"))

        

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
	        
            smach.StateMachine.add( 'SAY_DRINK_NOT_FOUND',
                                    Say(robot, ["I could not find the drink you wanted.", 
                                                "I looked really hard, but I couldn't find your drink."]),
                                    transitions={   'spoken':'Aborted' }) 

            smach.StateMachine.add( 'PICKUP_DRINK',
                                    GrabMachine(arm, robot, query_grabpoint),
                                    transitions={   "succeeded":"NAVIGATE_TO_LOCATION_RADIUS",
                                                    "failed":'LOOK_FOR_DRINK' }) 

            smach.StateMachine.add( 'SAY_DRINK_NOT_GRASPED',
                                    Say(robot, ["I could not pick up the drink you wanted", 
                                                "I failed to grab the object you wanted."]),
                                    transitions={   'spoken':'Aborted' }) 


            smach.StateMachine.add( 'NAVIGATE_TO_LOCATION_RADIUS',
                                    GoToStartLocation(robot),
                                    transitions={   "done":"PEOPLE_LASER_DETECTION",
                                                    "failed":'Aborted'})

            smach.StateMachine.add( 'PEOPLE_LASER_DETECTION',
                                    DriveToClosestPerson(robot),
                                    transitions={   "Done":"HANDOVER_TO_HUMAN",
                                                    "Aborted":'Done',
                                                    "Failed":"Done"})

            smach.StateMachine.add("HANDOVER_TO_HUMAN",
                                    HandoverToHuman(arm, robot),
                                    transitions={   'succeeded':'Done',
                                                    'failed':'Done'})

if __name__ == "__main__":
    rospy.init_node('fetch_and_carry_exec')
    startup(FetchAndCarry)

   
